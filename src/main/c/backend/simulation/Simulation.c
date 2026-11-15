#include "Simulation.h"
#include "../domain/Geometry.h"
#include "../semantic-analysis/SemanticAnalyzer.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* MODULE INTERNAL STATE */

#define EPSILON 1e-6

static Logger * _logger = NULL;
static int _errors = 0;
static bool _aborted = false;
static Model * _model = NULL;

void _shutdownSimulationModule() {
	if (_logger != NULL) {
		logDebugging(_logger, "Destroying module: Simulation...");
		destroyLogger(_logger);
		_logger = NULL;
	}
}

ModuleDestructor initializeSimulationModule() {
	_logger = createLogger("Simulation");
	return _shutdownSimulationModule;
}

#define simulationError(...) do { ++_errors; logError(_logger, __VA_ARGS__); } while (0)

/* PATHS */

typedef struct Path {
	Vec2 * points;
	int count;
	int capacity;
} Path;

static void _pathPush(Path * path, Vec2 point) {
	if (path->count == path->capacity) {
		path->capacity = path->capacity == 0 ? 64 : 2 * path->capacity;
		path->points = realloc(path->points, path->capacity * sizeof(Vec2));
	}
	path->points[path->count++] = point;
}

static void _pathFree(Path * path) {
	free(path->points);
	path->points = NULL;
	path->count = path->capacity = 0;
}

/* KEYFRAMES */

static Keyframe * _last(Aircraft * aircraft) {
	return aircraft->frameCount == 0 ? NULL : &aircraft->frames[aircraft->frameCount - 1];
}

static void _push(Aircraft * aircraft, double t, Vec3 p, bool airborne) {
	Keyframe * last = _last(aircraft);
	if (last != NULL && fabs(last->t - t) < EPSILON && distance3(last->p, p) < EPSILON) {
		last->airborne = airborne;
		return;
	}
	if (aircraft->frameCount == aircraft->frameCapacity) {
		aircraft->frameCapacity = aircraft->frameCapacity == 0 ? 64 : 2 * aircraft->frameCapacity;
		aircraft->frames = realloc(aircraft->frames, aircraft->frameCapacity * sizeof(Keyframe));
	}
	Keyframe frame = {.t = t, .p = p, .airborne = airborne, .energy = 0};
	aircraft->frames[aircraft->frameCount++] = frame;
	aircraft->deployed = 1;
}

/* ROUTING AROUND EXCLUDED ZONES */

#define MAX_ROUTE_POINTS 64

/** True if the segment a-b crosses the interior of the box (Liang-Barsky clipping). */
static bool _segmentHitsBox(Vec2 a, Vec2 b, double x0, double y0, double x1, double y1) {
	const double shrink = 1e-6;
	x0 += shrink; y0 += shrink; x1 -= shrink; y1 -= shrink;
	const double dx = b.x - a.x;
	const double dy = b.y - a.y;
	const double p[4] = {-dx, dx, -dy, dy};
	const double q[4] = {a.x - x0, x1 - a.x, a.y - y0, y1 - a.y};
	double u0 = 0;
	double u1 = 1;
	for (int k = 0; k < 4; ++k) {
		if (fabs(p[k]) < 1e-12) {
			if (q[k] < 0) return false;
		}
		else {
			const double r = q[k] / p[k];
			if (p[k] < 0) { if (u1 < r) return false; if (u0 < r) u0 = r; }
			else { if (r < u0) return false; if (r < u1) u1 = r; }
		}
	}
	return u0 < u1;
}

static void _excludedBox(const Zone * zone, double * x0, double * y0, double * x1, double * y1) {
	zoneBounds(zone, x0, y0, x1, y1);
	*x0 -= SIM_DETOUR_MARGIN; *y0 -= SIM_DETOUR_MARGIN;
	*x1 += SIM_DETOUR_MARGIN; *y1 += SIM_DETOUR_MARGIN;
}

/**
 * Computes the intermediate waypoints (excluding a and b) of a horizontal
 * route from a to b that contours the (expanded) bounding box of every
 * excluded zone, going around the shortest side.
 */
static int _route(Vec2 a, Vec2 b, Vec2 * out, int max, int depth) {
	if (max <= 0 || 6 < depth) return 0;
	for (int z = 0; z < _model->zoneCount; ++z) {
		const Zone * zone = &_model->zones[z];
		if (!zone->excluded) continue;
		double x0, y0, x1, y1;
		_excludedBox(zone, &x0, &y0, &x1, &y1);
		if (!_segmentHitsBox(a, b, x0, y0, x1, y1)) continue;
		const Vec2 corners[4] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
		Vec2 best[2];
		int bestCount = 0;
		double bestLength = DBL_MAX;
		for (int i = 0; i < 4; ++i) {
			// Detour through a single corner.
			if (!_segmentHitsBox(a, corners[i], x0, y0, x1, y1) && !_segmentHitsBox(corners[i], b, x0, y0, x1, y1)) {
				const double length = distance2(a, corners[i]) + distance2(corners[i], b);
				if (length < bestLength) { bestLength = length; best[0] = corners[i]; bestCount = 1; }
			}
			// Detour through two adjacent corners (both orders).
			for (int s = -1; s <= 1; s += 2) {
				const Vec2 c = corners[(i + 4 + s) % 4];
				if (!_segmentHitsBox(a, corners[i], x0, y0, x1, y1) && !_segmentHitsBox(c, b, x0, y0, x1, y1)) {
					const double length = distance2(a, corners[i]) + distance2(corners[i], c) + distance2(c, b);
					if (length < bestLength) { bestLength = length; best[0] = corners[i]; best[1] = c; bestCount = 2; }
				}
			}
		}
		if (bestCount == 0) return 0;	// An endpoint lies inside the box: nothing to contour.
		Vec2 nodes[4] = {a, best[0], best[1], b};
		if (bestCount == 1) nodes[2] = b;
		int written = 0;
		for (int k = 0; k <= bestCount; ++k) {
			written += _route(nodes[k], nodes[k + 1], out + written, max - written, depth + 1);
			if (k < bestCount && written < max) out[written++] = nodes[k + 1];
		}
		return written;
	}
	return 0;
}

/** Length of the horizontal route from a to b. */
static double _routedLength(Vec2 a, Vec2 b) {
	Vec2 points[MAX_ROUTE_POINTS];
	const int count = _route(a, b, points, MAX_ROUTE_POINTS, 0);
	double length = 0;
	Vec2 previous = a;
	for (int k = 0; k < count; ++k) {
		length += distance2(previous, points[k]);
		previous = points[k];
	}
	return length + distance2(previous, b);
}

/** Vertical move at the climb or descent rate. */
static void _verticalTo(Aircraft * aircraft, double z, double speed) {
	Keyframe * last = _last(aircraft);
	const double dz = z - last->p.z;
	if (fabs(dz) < EPSILON) return;
	const double rate = fmin(speed, 0 < dz ? SIM_CLIMB_RATE : SIM_DESCENT_RATE);
	_push(aircraft, last->t + fabs(dz) / rate, (Vec3) {last->p.x, last->p.y, z}, true);
}

/**
 * Moves to the target. The aircraft first changes altitude vertically up to
 * its transit level (target altitude plus "layer", so that the aircraft of a
 * group transit at different altitudes and never cross each other), then flies
 * horizontally at the given speed contouring the excluded zones, and finally
 * descends to the target altitude. Returns the arrival instant.
 */
static double _moveTo(Aircraft * aircraft, Vec3 target, double speed, double layer) {
	Keyframe * last = _last(aircraft);
	const Vec2 a = {last->p.x, last->p.y};
	const Vec2 b = {target.x, target.y};
	const bool horizontal = EPSILON < distance2(a, b);
	const double level = target.z + (horizontal ? layer : 0);
	_verticalTo(aircraft, level, speed);
	last = _last(aircraft);
	if (horizontal) {
		Vec2 points[MAX_ROUTE_POINTS + 1];
		int count = _route(a, b, points, MAX_ROUTE_POINTS, 0);
		points[count++] = b;
		double t = last->t;
		Vec2 previous = a;
		for (int k = 0; k < count; ++k) {
			const double distance = distance2(previous, points[k]);
			t += distance < EPSILON ? 0 : distance / speed;
			_push(aircraft, t, (Vec3) {points[k].x, points[k].y, level}, true);
			previous = points[k];
		}
	}
	_verticalTo(aircraft, target.z, speed);
	return _last(aircraft)->t;
}

/** Transit altitude offset of the k-th aircraft of a group. */
static double _layer(int k) {
	const double step = _model->hasMinSeparation ? fmax(5.0, 1.1 * _model->minSeparation) : 5.0;
	return k * step;
}

static double _cruiseSpeed(const Aircraft * aircraft) {
	return SIM_CRUISE_FACTOR * aircraft->type->maxSpeed;
}

/* STATEMENTS */

typedef struct Context {
	const char * label;
	double start;
} Context;

static double _runList(StatementList * list, double cursor);

/**
 * Parking slot of an aircraft at a station. Slots are assigned by the global
 * index of the aircraft, so two fleets that share a station never overlap.
 */
static Vec2 _groundSlot(const Station * station, const Aircraft * aircraft) {
	const int index = (int) (aircraft - _model->aircraft);
	const int count = _model->aircraftCount;
	double spacing = SIM_GROUND_SLOT;
	if (_model->hasMinSeparation && spacing < 1.5 * _model->minSeparation) spacing = 1.5 * _model->minSeparation;
	Vec2 slot = {station->position.x + (index - (count - 1) / 2.0) * spacing, station->position.y};
	return slot;
}

static Vec2 _centroidOf(const Selection * selection) {
	Vec2 c = {0, 0};
	for (int k = 0; k < selection->count; ++k) {
		Keyframe * last = _last(&_model->aircraft[selection->indexes[k]]);
		c.x += last->p.x / selection->count;
		c.y += last->p.y / selection->count;
	}
	return c;
}

/**
 * Assigns each aircraft of the selection to one of the targets (as many as
 * aircraft), minimizing the sum of the squared distances (Hungarian
 * algorithm, O(n^3)). When every aircraft flies in a straight line to its
 * target and all of them arrive at the same time (as in a morph), this
 * assignment guarantees that no two aircraft get closer than the smallest
 * spacing between the starting or the target points, divided by sqrt(2)
 * (Turpin, Michael & Kumar, CAPT, 2014). In particular, no two paths cross.
 */
static void _assign(const Selection * selection, const Vec2 * targets, int * slotOf) {
	const int n = selection->count;
	if (n <= 0) return;
	const unsigned size = (unsigned) n + 1u;
	double * cost = calloc(size * size, sizeof(double));
	double * u = calloc(size, sizeof(double));
	double * v = calloc(size, sizeof(double));
	double * minv = calloc(size, sizeof(double));
	int * owner = calloc(size, sizeof(int));	// owner[j]: aircraft (1-based) assigned to target j.
	int * way = calloc(size, sizeof(int));
	bool * used = calloc(size, sizeof(bool));
	for (int k = 1; k <= n; ++k) {
		const Keyframe * last = _last(&_model->aircraft[selection->indexes[k - 1]]);
		for (int j = 1; j <= n; ++j) {
			const double dx = last->p.x - targets[j - 1].x;
			const double dy = last->p.y - targets[j - 1].y;
			cost[k * size + j] = dx * dx + dy * dy;
		}
	}
	for (int k = 1; k <= n; ++k) {
		owner[0] = k;
		int j0 = 0;
		for (int j = 0; j <= n; ++j) { minv[j] = DBL_MAX; used[j] = false; }
		do {
			used[j0] = true;
			const int k0 = owner[j0];
			double delta = DBL_MAX;
			int j1 = 0;
			for (int j = 1; j <= n; ++j) {
				if (used[j]) continue;
				const double reduced = cost[k0 * size + j] - u[k0] - v[j];
				if (reduced < minv[j]) { minv[j] = reduced; way[j] = j0; }
				if (minv[j] < delta) { delta = minv[j]; j1 = j; }
			}
			for (int j = 0; j <= n; ++j) {
				if (used[j]) { u[owner[j]] += delta; v[j] -= delta; }
				else minv[j] -= delta;
			}
			j0 = j1;
		} while (owner[j0] != 0);
		do {
			const int j1 = way[j0];
			owner[j0] = owner[j1];
			j0 = j1;
		} while (j0 != 0);
	}
	for (int j = 1; j <= n; ++j) slotOf[owner[j] - 1] = j - 1;
	free(cost); free(u); free(v); free(minv); free(owner); free(way); free(used);
}

static Vec2 _formationOffset(const FormationShape * shape, int index, int count) {
	Vec2 offset = {0, 0};
	switch (shape->kind) {
		case FORMATION_LINE: {
			const double spacing = quantityToSI(shape->spacing);
			offset.x = (index - (count - 1) / 2.0) * spacing;
			break;
		}
		case FORMATION_GRID: {
			const double spacing = quantityToSI(shape->spacing);
			const long row = index / shape->cols;
			const long col = index % shape->cols;
			offset.x = (col - (shape->cols - 1) / 2.0) * spacing;
			offset.y = ((shape->rows - 1) / 2.0 - row) * spacing;
			break;
		}
		case FORMATION_CIRCLE: {
			const double radius = quantityToSI(shape->radius);
			const double angle = 2 * M_PI * index / count;
			offset.x = radius * cos(angle);
			offset.y = radius * sin(angle);
			break;
		}
	}
	return offset;
}

/** Closed ring of a convex polygon scaled around its centroid. */
static void _ring(const Zone * zone, double factor, Path * path) {
	for (int k = 0; k <= zone->vertexCount; ++k) {
		const Vec2 v = zone->vertices[k % zone->vertexCount];
		Vec2 p = {zone->center.x + factor * (v.x - zone->center.x), zone->center.y + factor * (v.y - zone->center.y)};
		_pathPush(path, p);
	}
}

/** Appends a lawnmower pass from xs to xe at height y, contouring excluded zones. */
static void _pass(Path * path, double y, double xs, double xe) {
	const double direction = xs <= xe ? 1 : -1;
	double lo[MAX_ZONES], hi[MAX_ZONES], detour[MAX_ZONES];
	int count = 0;
	for (int z = 0; z < _model->zoneCount; ++z) {
		const Zone * zone = &_model->zones[z];
		if (!zone->excluded) continue;
		double minX, minY, maxX, maxY;
		zoneBounds(zone, &minX, &minY, &maxX, &maxY);
		minX -= SIM_DETOUR_MARGIN; maxX += SIM_DETOUR_MARGIN;
		minY -= SIM_DETOUR_MARGIN; maxY += SIM_DETOUR_MARGIN;
		const double a = fmin(xs, xe);
		const double b = fmax(xs, xe);
		if (y < minY || maxY < y || maxX < a || b < minX) continue;
		lo[count] = minX;
		hi[count] = maxX;
		detour[count] = (y - minY) < (maxY - y) ? minY : maxY;
		++count;
	}
	// Order the obstacles along the direction of travel.
	for (int i = 0; i < count; ++i) {
		for (int j = i + 1; j < count; ++j) {
			if (direction * lo[j] < direction * lo[i]) {
				double t;
				t = lo[i]; lo[i] = lo[j]; lo[j] = t;
				t = hi[i]; hi[i] = hi[j]; hi[j] = t;
				t = detour[i]; detour[i] = detour[j]; detour[j] = t;
			}
		}
	}
	_pathPush(path, (Vec2) {xs, y});
	for (int k = 0; k < count; ++k) {
		const double enter = direction > 0 ? lo[k] : hi[k];
		const double leave = direction > 0 ? hi[k] : lo[k];
		_pathPush(path, (Vec2) {enter, y});
		_pathPush(path, (Vec2) {enter, detour[k]});
		_pathPush(path, (Vec2) {leave, detour[k]});
		_pathPush(path, (Vec2) {leave, y});
	}
	_pathPush(path, (Vec2) {xe, y});
}

/** Builds one path per aircraft for the requested sweep pattern. */
static void _sweepPaths(const Zone * zone, SweepPattern pattern, double spacing, int n, Path * paths) {
	double minX, minY, maxX, maxY;
	zoneBounds(zone, &minX, &minY, &maxX, &maxY);
	double inradius = zone->isCircle ? zone->radius : distanceToEdges(zone->center, zone->vertices, zone->vertexCount);
	if (inradius < EPSILON) inradius = EPSILON;
	switch (pattern) {
		case PATTERN_LAWNMOWER: {
			double ys[4096], xa[4096], xb[4096];
			int lanes = 0;
			for (double y = minY + spacing / 2; y < maxY && lanes < 4096; y += spacing) {
				if (horizontalChord(zone->vertices, zone->vertexCount, y, &xa[lanes], &xb[lanes])) {
					ys[lanes++] = y;
				}
			}
			if (lanes == 0 && horizontalChord(zone->vertices, zone->vertexCount, zone->center.y, &xa[0], &xb[0])) {
				ys[lanes++] = zone->center.y;
			}
			for (int i = 0; i < n; ++i) {
				const int from = i * lanes / n;
				const int to = (i + 1) * lanes / n;
				for (int lane = from; lane < to; ++lane) {
					if ((lane - from) % 2 == 0) _pass(&paths[i], ys[lane], xa[lane], xb[lane]);
					else _pass(&paths[i], ys[lane], xb[lane], xa[lane]);
				}
			}
			break;
		}
		case PATTERN_SPIRAL: {
			double factors[4096];
			int rings = 0;
			for (int k = 0; rings < 4096; ++k) {
				const double factor = 1 - (spacing / 2 + k * spacing) / inradius;
				if (factor <= 0.02) break;
				factors[rings++] = factor;
			}
			for (int i = 0; i < n; ++i) {
				const int from = i * rings / n;
				const int to = (i + 1) * rings / n;
				for (int ring = from; ring < to; ++ring) _ring(zone, factors[ring], &paths[i]);
				if (rings == 0 && i == 0) _pathPush(&paths[i], zone->center);
			}
			break;
		}
		case PATTERN_PERIMETER: {
			double factor = 1 - (spacing / 2) / inradius;
			if (factor <= 0) factor = 0.5;
			Path ring = {0};
			_ring(zone, factor, &ring);
			double length = 0;
			for (int k = 1; k < ring.count; ++k) length += distance2(ring.points[k - 1], ring.points[k]);
			for (int i = 0; i < n; ++i) {
				const double from = length * i / n;
				const double to = length * (i + 1) / n;
				double walked = 0;
				for (int k = 1; k < ring.count; ++k) {
					const Vec2 a = ring.points[k - 1];
					const Vec2 b = ring.points[k];
					const double segment = distance2(a, b);
					const double s0 = walked;
					const double s1 = walked + segment;
					walked = s1;
					if (s1 < from || to < s0 || segment < EPSILON) continue;
					const double u0 = fmax(0, (from - s0) / segment);
					const double u1 = fmin(1, (to - s0) / segment);
					_pathPush(&paths[i], (Vec2) {a.x + u0 * (b.x - a.x), a.y + u0 * (b.y - a.y)});
					_pathPush(&paths[i], (Vec2) {a.x + u1 * (b.x - a.x), a.y + u1 * (b.y - a.y)});
				}
			}
			_pathFree(&ring);
			break;
		}
	}
}

static const char * _maneuverName(StatementKind kind) {
	switch (kind) {
		case STMT_TAKEOFF: return "takeoff";
		case STMT_LAND: return "land";
		case STMT_MORPH: return "morph";
		case STMT_SWEEP: return "sweep";
		case STMT_ORBIT: return "orbit";
		case STMT_HOLD: return "hold";
		case STMT_RETURN: return "return";
		case STMT_PARALLEL: return "parallel";
		case STMT_SEQUENCE: return "sequence";
	}
	return "?";
}

static void _recordPhase(const char * label, StatementKind kind, double start, double end) {
	if (_model->phaseCount == MAX_PHASES) return;
	PhaseRecord * record = &_model->phases[_model->phaseCount++];
	snprintf(record->label, sizeof(record->label), "%s", label);
	snprintf(record->maneuver, sizeof(record->maneuver), "%s", _maneuverName(kind));
	record->start = start;
	record->end = end;
}

/**
 * Collective, deconflicted transit of a group. Each aircraft climbs (or
 * descends) vertically to its own transit level (the highest of its current
 * and target altitudes, plus its layer) and hovers there until every aircraft
 * of the group has reached its level, so nobody flies horizontally next to a
 * neighbor that is still climbing. Then each one flies horizontally to its
 * target contouring excluded zones, and hovers there until every aircraft of
 * the group has arrived. Only then, all of them change altitude vertically to
 * their target altitude, so no trajectory crosses another one at the same
 * altitude. If finishAt is non-negative, each vertical change is scheduled to
 * end exactly at that instant. Returns the instant in which the group ends.
 */
static double _groupTransit(const Selection * selection, const Vec2 * targets, const double * targetZ, double finishAt) {
	double climbed = 0;
	for (int k = 0; k < selection->count; ++k) {
		Aircraft * aircraft = &_model->aircraft[selection->indexes[k]];
		const Keyframe * last = _last(aircraft);
		_verticalTo(aircraft, fmax(last->p.z, targetZ[k]) + _layer(k), _cruiseSpeed(aircraft));
		climbed = fmax(climbed, _last(aircraft)->t);
	}
	double sync = 0;
	for (int k = 0; k < selection->count; ++k) {
		Aircraft * aircraft = &_model->aircraft[selection->indexes[k]];
		const Keyframe * last = _last(aircraft);
		_push(aircraft, climbed, last->p, true);
		_moveTo(aircraft, (Vec3) {targets[k].x, targets[k].y, last->p.z}, _cruiseSpeed(aircraft), 0);
		sync = fmax(sync, _last(aircraft)->t);
	}
	double end = sync;
	for (int k = 0; k < selection->count; ++k) {
		Aircraft * aircraft = &_model->aircraft[selection->indexes[k]];
		const Keyframe * last = _last(aircraft);
		double begin = sync;
		if (0 <= finishAt) {
			const double dz = last->p.z - targetZ[k];
			const double rate = 0 < dz ? SIM_DESCENT_RATE : SIM_CLIMB_RATE;
			begin = fmax(sync, finishAt - fabs(dz) / rate);
		}
		_push(aircraft, begin, last->p, true);
		_verticalTo(aircraft, targetZ[k], DBL_MAX);
		end = fmax(end, _last(aircraft)->t);
	}
	return end;
}

/** Duration of _groupTransit for the given targets (without executing it). */
static double _groupTransitDuration(const Selection * selection, const Vec2 * targets, const double * targetZ) {
	double climb = 0;
	double transit = 0;
	double vertical = 0;
	for (int k = 0; k < selection->count; ++k) {
		Aircraft * aircraft = &_model->aircraft[selection->indexes[k]];
		const Keyframe * last = _last(aircraft);
		if (last == NULL) continue;
		const double level = fmax(last->p.z, targetZ[k]) + _layer(k);
		const double up = level - last->p.z;
		const double horizontal = _routedLength((Vec2) {last->p.x, last->p.y}, targets[k]);
		const double speed = _cruiseSpeed(aircraft);
		climb = fmax(climb, fabs(up) / fmin(speed, 0 < up ? SIM_CLIMB_RATE : SIM_DESCENT_RATE));
		transit = fmax(transit, horizontal / speed);
		vertical = fmax(vertical, fabs(level - targetZ[k]) / SIM_DESCENT_RATE);
	}
	return climb + transit + vertical;
}

static void _landingTargets(const Selection * selection, const Station * station, Vec2 * targets, double * targetZ) {
	for (int k = 0; k < selection->count; ++k) {
		targets[k] = _groundSlot(station, &_model->aircraft[selection->indexes[k]]);
		targetZ[k] = station->altitude;
	}
}

static double _approachDuration(const Selection * selection, const Station * station) {
	Vec2 targets[MAX_AIRCRAFT];
	double targetZ[MAX_AIRCRAFT];
	_landingTargets(selection, station, targets, targetZ);
	return _groupTransitDuration(selection, targets, targetZ);
}

/**
 * Brings every aircraft of the selection back to its ground slot at the
 * station. If touchdown is non-negative, all of them touch down exactly at
 * that instant.
 */
static double _land(const Selection * selection, const Station * station, double start, double touchdown) {
	(void) start;
	Vec2 targets[MAX_AIRCRAFT];
	double targetZ[MAX_AIRCRAFT];
	_landingTargets(selection, station, targets, targetZ);
	const double end = _groupTransit(selection, targets, targetZ, touchdown);
	for (int k = 0; k < selection->count; ++k) {
		Aircraft * aircraft = &_model->aircraft[selection->indexes[k]];
		_last(aircraft)->airborne = false;
	}
	return end;
}

static double _runLeaf(Statement * statement, double cursor) {
	char ref[64];
	char label[160];
	fleetRefText(statement->fleet, ref, sizeof(ref));
	switch (statement->kind) {
		case STMT_SWEEP: case STMT_ORBIT:
			snprintf(label, sizeof(label), "%s %s %s", ref, _maneuverName(statement->kind), statement->zoneName);
			break;
		case STMT_TAKEOFF: case STMT_LAND: case STMT_RETURN:
			snprintf(label, sizeof(label), "%s %s %s", ref, _maneuverName(statement->kind), statement->stationName);
			break;
		case STMT_MORPH:
			snprintf(label, sizeof(label), "%s morph to %s", ref, statement->formation->name != NULL ? statement->formation->name : "(anónima)");
			break;
		default:
			snprintf(label, sizeof(label), "%s %s", ref, _maneuverName(statement->kind));
			break;
	}

	Selection selection;
	resolveFleetRef(_model, statement->fleet, &selection);

	double start = cursor;
	if (statement->hasTimeAt && statement->kind == STMT_TAKEOFF) {
		// "takeoff ... at T": T is the instant in which the aircraft leave the ground.
		const double at = quantityToSI(statement->timeAt);
		if (at < cursor - EPSILON) {
			simulationError("La maniobra '%s' fija su inicio en %.2f [s], pero la maniobra anterior termina en %.2f [s].", label, at, cursor);
			_aborted = true;
			return cursor;
		}
		start = at;
	}
	else if (statement->hasTimeAt && statement->kind == STMT_LAND) {
		// "land ... at T": T is the touchdown instant; the approach is computed backwards.
		const double at = quantityToSI(statement->timeAt);
		const double approach = _approachDuration(&selection, findStation(_model, statement->stationName));
		start = at - approach;
		if (start < cursor - EPSILON) {
			simulationError("La maniobra '%s' exige tocar tierra en %.2f [s]: la aproximación demora %.2f [s] y debería comenzar en %.2f [s], pero la maniobra anterior termina en %.2f [s].",
				label, at, approach, start, cursor);
			_aborted = true;
			return cursor;
		}
	}

	// Pre-conditions: state of each aircraft, and synchronization to "start".
	for (int k = 0; k < selection.count; ++k) {
		Aircraft * aircraft = &_model->aircraft[selection.indexes[k]];
		Keyframe * last = _last(aircraft);
		const bool airborne = last != NULL && last->airborne;
		if (statement->kind == STMT_TAKEOFF) {
			if (airborne) {
				simulationError("La maniobra '%s' no es válida: %s ya se encuentra en vuelo.", label, aircraft->id);
				_aborted = true;
			}
		}
		else if (!airborne) {
			simulationError("La maniobra '%s' no es válida: %s no está en vuelo (falta un 'takeoff' previo) en t = %.2f [s].", label, aircraft->id, start);
			_aborted = true;
		}
		if (last != NULL && start < last->t - EPSILON) {
			simulationError("La maniobra '%s' comienza en %.2f [s], pero %s está ocupada hasta %.2f [s].", label, start, aircraft->id, last->t);
			_aborted = true;
		}
	}
	if (_aborted) return cursor;
	for (int k = 0; k < selection.count; ++k) {
		Aircraft * aircraft = &_model->aircraft[selection.indexes[k]];
		Keyframe * last = _last(aircraft);
		if (last != NULL) _push(aircraft, start, last->p, last->airborne);
	}

	double end = start;
	switch (statement->kind) {
		case STMT_TAKEOFF: {
			const Station * station = findStation(_model, statement->stationName);
			for (int k = 0; k < selection.count; ++k) {
				Aircraft * aircraft = &_model->aircraft[selection.indexes[k]];
				const Vec2 slot = _groundSlot(station, aircraft);
				Vec3 ground = {slot.x, slot.y, station->altitude};
				Keyframe * last = _last(aircraft);
				if (last != NULL && 1.0 < distance3(last->p, ground)) {
					// Parked elsewhere: it is moved to the new station on the ground.
					logWarning(_logger, "%s se reubica en tierra hasta la estación '%s'.", aircraft->id, station->name);
				}
				_push(aircraft, start, ground, false);
				Vec3 air = {slot.x, slot.y, station->altitude + SIM_CRUISE_ALTITUDE};
				_push(aircraft, start + SIM_CRUISE_ALTITUDE / SIM_CLIMB_RATE, air, true);
				end = fmax(end, _last(aircraft)->t);
			}
			break;
		}
		case STMT_LAND:
		case STMT_RETURN:
			end = _land(&selection, findStation(_model, statement->stationName), start,
				statement->kind == STMT_LAND && statement->hasTimeAt ? quantityToSI(statement->timeAt) : -1);
			if (statement->kind == STMT_RETURN) {
				const double before = quantityToSI(statement->timeBefore);
				if (before + EPSILON < end) {
					simulationError("La maniobra '%s' termina en %.2f [s], después de su fecha límite 'before %.2f [s]'.", label, end, before);
				}
			}
			break;
		case STMT_MORPH: {
			const FormationShape * shape = statement->formation->shape != NULL
				? statement->formation->shape
				: findFormation(_model, statement->formation->name)->shape;
			const double duration = quantityToSI(statement->duration);
			// Positions are relative to the center of the commanded group (fleet or sub-fleet).
			const Vec2 center = _centroidOf(&selection);
			Vec2 * slots = calloc((unsigned) selection.count + 1u, sizeof(Vec2));
			int * slotOf = calloc((unsigned) selection.count + 1u, sizeof(int));
			for (int j = 0; j < selection.count; ++j) {
				const Vec2 offset = _formationOffset(shape, j, selection.count);
				slots[j] = (Vec2) {center.x + offset.x, center.y + offset.y};
			}
			_assign(&selection, slots, slotOf);
			for (int k = 0; k < selection.count; ++k) {
				Aircraft * aircraft = &_model->aircraft[selection.indexes[k]];
				const Keyframe * last = _last(aircraft);
				Vec3 target = {slots[slotOf[k]].x, slots[slotOf[k]].y, last->p.z};
				const double distance = distance3(last->p, target);
				const double requiredSpeed = distance / duration;
				const double requiredAccel = 4 * distance / (duration * duration);
				if (aircraft->type->maxSpeed + EPSILON < requiredSpeed) {
					simulationError("La maniobra '%s' exige que %s recorra %.2f [m] en %.2f [s] (%.2f [m/s]), superando su max_speed de %.2f [m/s] (t = %.2f [s]).",
						label, aircraft->id, distance, duration, requiredSpeed, aircraft->type->maxSpeed, start);
				}
				else if (aircraft->type->hasMaxAccel && aircraft->type->maxAccel + EPSILON < requiredAccel) {
					simulationError("La maniobra '%s' exige que %s acelere a %.2f [m/s^2], superando su max_accel de %.2f [m/s^2] (t = %.2f [s]).",
						label, aircraft->id, requiredAccel, aircraft->type->maxAccel, start);
				}
				_push(aircraft, start + duration, target, true);
			}
			free(slots);
			free(slotOf);
			end = start + duration;
			break;
		}
		case STMT_HOLD: {
			const double duration = quantityToSI(statement->duration);
			for (int k = 0; k < selection.count; ++k) {
				Aircraft * aircraft = &_model->aircraft[selection.indexes[k]];
				_push(aircraft, start + duration, _last(aircraft)->p, true);
			}
			end = start + duration;
			break;
		}
		case STMT_SWEEP: {
			const Zone * zone = findZone(_model, statement->zoneName);
			const double spacing = quantityToSI(statement->spacing);
			const double altitude = quantityToSI(statement->altitude);
			Path * paths = calloc((unsigned) selection.count + 1u, sizeof(Path));
			_sweepPaths(zone, statement->pattern, spacing, selection.count, paths);
			Vec2 * starts = calloc((unsigned) selection.count + 1u, sizeof(Vec2));
			int * slotOf = calloc((unsigned) selection.count + 1u, sizeof(int));
			for (int j = 0; j < selection.count; ++j) {
				starts[j] = paths[j].count == 0 ? zone->center : paths[j].points[0];
			}
			_assign(&selection, starts, slotOf);
			Vec2 targets[MAX_AIRCRAFT];
			double targetZ[MAX_AIRCRAFT];
			for (int k = 0; k < selection.count; ++k) {
				const Path * path = &paths[slotOf[k]];
				const Keyframe * last = _last(&_model->aircraft[selection.indexes[k]]);
				targets[k] = path->count == 0 ? (Vec2) {last->p.x, last->p.y} : path->points[0];
				targetZ[k] = altitude;
			}
			// Everyone starts the pattern together, once the whole group has
			// descended to the sweep altitude: an aircraft that is already there
			// must not fly under a neighbor that is still descending.
			const double patternStart = _groupTransit(&selection, targets, targetZ, -1);
			end = patternStart;
			for (int k = 0; k < selection.count; ++k) {
				Aircraft * aircraft = &_model->aircraft[selection.indexes[k]];
				const Path * path = &paths[slotOf[k]];
				_push(aircraft, patternStart, _last(aircraft)->p, true);
				for (int p = 1; p < path->count; ++p) {
					Vec3 target = {path->points[p].x, path->points[p].y, altitude};
					_moveTo(aircraft, target, _cruiseSpeed(aircraft), 0);
				}
				if (path->count == 0) {
					logWarning(_logger, "En '%s', %s no recibe ningún tramo del patrón (la zona es pequeña para tantas aeronaves).", label, aircraft->id);
				}
				end = fmax(end, _last(aircraft)->t);
			}
			for (int j = 0; j < selection.count; ++j) _pathFree(&paths[j]);
			free(paths);
			free(starts);
			free(slotOf);
			break;
		}
		case STMT_ORBIT: {
			const Zone * zone = findZone(_model, statement->zoneName);
			const double radius = quantityToSI(statement->radius);
			const double altitude = quantityToSI(statement->altitude);
			const double duration = quantityToSI(statement->duration);
			double speed = DBL_MAX;
			for (int k = 0; k < selection.count; ++k) {
				speed = fmin(speed, SIM_ORBIT_FACTOR * _model->aircraft[selection.indexes[k]].type->maxSpeed);
			}
			const double omega = speed / radius;
			double orbitStart = start;
			Vec2 * entries = calloc((unsigned) selection.count + 1u, sizeof(Vec2));
			int * slotOf = calloc((unsigned) selection.count + 1u, sizeof(int));
			for (int j = 0; j < selection.count; ++j) {
				const double angle = 2 * M_PI * j / selection.count;
				entries[j] = (Vec2) {zone->center.x + radius * cos(angle), zone->center.y + radius * sin(angle)};
			}
			_assign(&selection, entries, slotOf);
			Vec2 targets[MAX_AIRCRAFT];
			double targetZ[MAX_AIRCRAFT];
			for (int k = 0; k < selection.count; ++k) {
				targets[k] = entries[slotOf[k]];
				targetZ[k] = altitude;
			}
			orbitStart = fmax(orbitStart, _groupTransit(&selection, targets, targetZ, -1));
			const double step = fmin(1.0, (M_PI / 36) / omega);
			for (int k = 0; k < selection.count; ++k) {
				Aircraft * aircraft = &_model->aircraft[selection.indexes[k]];
				const double phase = 2 * M_PI * slotOf[k] / selection.count;
				_push(aircraft, orbitStart, _last(aircraft)->p, true);
				for (double t = step; t < duration + step / 2; t += step) {
					const double tt = fmin(t, duration);
					Vec3 p = {zone->center.x + radius * cos(phase + omega * tt), zone->center.y + radius * sin(phase + omega * tt), altitude};
					_push(aircraft, orbitStart + tt, p, true);
				}
			}
			free(entries);
			free(slotOf);
			end = orbitStart + duration;
			break;
		}
		default:
			break;
	}
	_recordPhase(label, statement->kind, start, end);
	if (_model->windowEnd + EPSILON < end) {
		simulationError("La maniobra '%s' termina en %.2f [s], fuera de la ventana de la misión [%.2f [s], %.2f [s]].",
			label, end, _model->windowStart, _model->windowEnd);
	}
	return end;
}

static double _run(Statement * statement, double cursor) {
	if (_aborted) return cursor;
	switch (statement->kind) {
		case STMT_SEQUENCE: {
			const double end = _runList(statement->body, cursor);
			_recordPhase("sequence", STMT_SEQUENCE, cursor, end);
			return end;
		}
		case STMT_PARALLEL: {
			double end = cursor;
			for (StatementList * node = statement->body; node != NULL; node = node->next) {
				end = fmax(end, _run(node->statement, cursor));
			}
			_recordPhase("parallel", STMT_PARALLEL, cursor, end);
			return end;
		}
		default:
			return _runLeaf(statement, cursor);
	}
}

static double _runList(StatementList * list, double cursor) {
	for (StatementList * node = list; node != NULL && !_aborted; node = node->next) {
		cursor = _run(node->statement, cursor);
	}
	return cursor;
}

/* SAMPLING */

static FlightPhase _segmentPhase(const Keyframe * a, const Keyframe * b) {
	if (!a->airborne && !b->airborne) return PHASE_GROUND;
	const double dt = b->t - a->t;
	if (dt < EPSILON) return PHASE_HOVER;
	const double vz = (b->p.z - a->p.z) / dt;
	const double vh = hypot(b->p.x - a->p.x, b->p.y - a->p.y) / dt;
	if (0.05 < vz && vh < fabs(vz)) return PHASE_CLIMB;
	if (vz < -0.05 && vh < fabs(vz)) return PHASE_DESCENT;
	if (0.2 < vh || 0.05 < fabs(vz)) return PHASE_CRUISE;
	return PHASE_HOVER;
}

static double _phaseFactor(FlightPhase phase) {
	switch (phase) {
		case PHASE_GROUND: return 0;
		case PHASE_CLIMB: return SIM_ENERGY_CLIMB;
		case PHASE_DESCENT: return SIM_ENERGY_DESCENT;
		case PHASE_CRUISE: return SIM_ENERGY_CRUISE;
		case PHASE_HOVER: return SIM_ENERGY_HOVER;
	}
	return 1;
}

bool sampleAircraft(const Aircraft * aircraft, double t, Vec3 * position, bool * airborne, FlightPhase * phase, double * battery) {
	if (aircraft->frameCount == 0 || t < aircraft->frames[0].t - EPSILON) return false;
	const Keyframe * frames = aircraft->frames;
	const int n = aircraft->frameCount;
	if (frames[n - 1].t <= t) {
		*position = frames[n - 1].p;
		*airborne = frames[n - 1].airborne;
		*phase = frames[n - 1].airborne ? PHASE_HOVER : PHASE_GROUND;
		*battery = 1 - frames[n - 1].energy;
		return true;
	}
	// Binary search of the segment that contains t.
	int lo = 0;
	int hi = n - 1;
	while (lo + 1 < hi) {
		const int mid = (lo + hi) / 2;
		if (frames[mid].t <= t) lo = mid;
		else hi = mid;
	}
	const Keyframe * a = &frames[lo];
	const Keyframe * b = &frames[hi];
	const double dt = b->t - a->t;
	const double u = dt < EPSILON ? 1 : (t - a->t) / dt;
	position->x = a->p.x + u * (b->p.x - a->p.x);
	position->y = a->p.y + u * (b->p.y - a->p.y);
	position->z = a->p.z + u * (b->p.z - a->p.z);
	*airborne = a->airborne || b->airborne;
	*phase = _segmentPhase(a, b);
	*battery = 1 - (a->energy + u * (b->energy - a->energy));
	return true;
}

/* VERIFICATIONS */

static void _checkKinematicsAndEnergy(void) {
	for (int k = 0; k < _model->aircraftCount; ++k) {
		Aircraft * aircraft = &_model->aircraft[k];
		if (aircraft->frameCount == 0) continue;
		bool speedReported = false;
		bool batteryReported = false;
		double energy = 0;
		aircraft->frames[0].energy = 0;
		for (int f = 1; f < aircraft->frameCount; ++f) {
			Keyframe * a = &aircraft->frames[f - 1];
			Keyframe * b = &aircraft->frames[f];
			const double dt = b->t - a->t;
			const double distance = distance3(a->p, b->p);
			const FlightPhase phase = _segmentPhase(a, b);
			if (phase != PHASE_GROUND) {
				aircraft->flightTime += dt;
				aircraft->distance += distance;
			}
			if (!speedReported && EPSILON < dt && aircraft->type->maxSpeed * (1 + 1e-6) < distance / dt) {
				simulationError("%s supera su max_speed: %.2f [m/s] > %.2f [m/s] entre t = %.2f [s] y t = %.2f [s].",
					aircraft->id, distance / dt, aircraft->type->maxSpeed, a->t, b->t);
				speedReported = true;
			}
			const double rate = _phaseFactor(phase) / aircraft->type->endurance;
			const double previous = energy;
			energy += dt * rate;
			b->energy = energy;
			if (!batteryReported && 1 < energy) {
				/* The consumption is linear inside a segment, so the battery runs out before its end. */
				const double depletion = a->t + (1 - previous) / rate;
				simulationError("%s agota su batería (endurance %.0f [s]) en t = %.2f [s], antes de completar la misión.",
					aircraft->id, aircraft->type->endurance, depletion);
				batteryReported = true;
			}
		}
		aircraft->batteryAtEnd = 1 - energy;
		const Keyframe * last = _last(aircraft);
		if (last->airborne) {
			simulationError("%s permanece en vuelo al finalizar la misión (t = %.2f [s]); toda aeronave debe aterrizar o retornar a una estación.",
				aircraft->id, last->t);
		}
		else if (!batteryReported && _model->hasBatteryReserve && aircraft->batteryAtEnd + EPSILON < _model->batteryReserve) {
			simulationError("%s aterriza con %.1f [%%] de batería, por debajo de la reserva exigida de %.1f [%%].",
				aircraft->id, 100 * aircraft->batteryAtEnd, 100 * _model->batteryReserve);
		}
	}
}

static void _checkSampledTrajectories(void) {
	const int n = _model->aircraftCount;
	Vec3 * positions = calloc(n, sizeof(Vec3));
	bool * airborne = calloc(n, sizeof(bool));
	bool * exclusionReported = calloc(n, sizeof(bool));
	bool * linkReported = calloc(n, sizeof(bool));
	bool separationReported = false;
	for (double t = _model->windowStart; t <= _model->missionEnd + EPSILON; t += SIM_SAMPLE_DT) {
		for (int k = 0; k < n; ++k) {
			FlightPhase phase;
			double battery;
			if (!sampleAircraft(&_model->aircraft[k], t, &positions[k], &airborne[k], &phase, &battery)) airborne[k] = false;
		}
		for (int k = 0; k < n; ++k) {
			if (!airborne[k]) continue;
			Aircraft * aircraft = &_model->aircraft[k];
			const Vec2 ground = {positions[k].x, positions[k].y};
			if (!exclusionReported[k]) {
				for (int z = 0; z < _model->zoneCount; ++z) {
					if (_model->zones[z].excluded && pointInZone(ground, &_model->zones[z])) {
						simulationError("%s ingresa a la zona excluida '%s' en t = %.2f [s], posición (%.1f, %.1f).",
							aircraft->id, _model->zones[z].name, t, ground.x, ground.y);
						exclusionReported[k] = true;
						break;
					}
				}
			}
			if (!linkReported[k] && aircraft->type->hasLinkRange && 0 < _model->stationCount) {
				double nearest = DBL_MAX;
				for (int s = 0; s < _model->stationCount; ++s) {
					const Station * station = &_model->stations[s];
					Vec3 p = {station->position.x, station->position.y, station->altitude};
					nearest = fmin(nearest, distance3(p, positions[k]));
				}
				if (aircraft->type->linkRange + EPSILON < nearest) {
					simulationError("%s pierde enlace en t = %.2f [s]: está a %.1f [m] de la estación más cercana, y su link_range es de %.1f [m].",
						aircraft->id, t, nearest, aircraft->type->linkRange);
					linkReported[k] = true;
				}
			}
			if (_model->hasMinSeparation && !separationReported) {
				for (int j = k + 1; j < n; ++j) {
					if (!airborne[j]) continue;
					const double d = distance3(positions[k], positions[j]);
					if (d + EPSILON < _model->minSeparation) {
						simulationError("%s y %s están a %.2f [m] en t = %.2f [s], por debajo de la separación mínima de %.2f [m].",
							aircraft->id, _model->aircraft[j].id, d, t, _model->minSeparation);
						separationReported = true;
						break;
					}
				}
			}
		}
	}
	free(positions);
	free(airborne);
	free(exclusionReported);
	free(linkReported);
}

static void _computeCoverage(Zone * zone) {
	double minX, minY, maxX, maxY;
	zoneBounds(zone, &minX, &minY, &maxX, &maxY);
	const double cell = fmax(maxX - minX, maxY - minY) / 160.0;
	const int cols = (int) ceil((maxX - minX) / cell);
	const int rows = (int) ceil((maxY - minY) / cell);
	// 0: outside; 1: pending; 2: observed.
	unsigned char * grid = calloc((size_t) rows * cols, 1);
	int total = 0;
	for (int r = 0; r < rows; ++r) {
		for (int c = 0; c < cols; ++c) {
			const Vec2 p = {minX + (c + 0.5) * cell, minY + (r + 0.5) * cell};
			if (!pointInZone(p, zone)) continue;
			bool excluded = false;
			for (int z = 0; z < _model->zoneCount && !excluded; ++z) {
				excluded = _model->zones[z].excluded && pointInZone(p, &_model->zones[z]);
			}
			if (excluded) continue;
			grid[r * cols + c] = 1;
			++total;
		}
	}
	int observed = 0;
	for (int k = 0; k < _model->aircraftCount; ++k) {
		const Aircraft * aircraft = &_model->aircraft[k];
		if (!aircraft->type->hasPayload || aircraft->frameCount == 0) continue;
		const double halfFov = tan(aircraft->type->fov / 2);
		for (double t = aircraft->frames[0].t; t <= _model->missionEnd + EPSILON; t += SIM_SAMPLE_DT) {
			Vec3 p;
			bool airborne;
			FlightPhase phase;
			double battery;
			if (!sampleAircraft(aircraft, t, &p, &airborne, &phase, &battery) || !airborne) continue;
			const double radius = p.z * halfFov;
			if (radius < cell / 2) continue;
			const int c0 = (int) floor((p.x - radius - minX) / cell);
			const int c1 = (int) ceil((p.x + radius - minX) / cell);
			const int r0 = (int) floor((p.y - radius - minY) / cell);
			const int r1 = (int) ceil((p.y + radius - minY) / cell);
			for (int r = r0 < 0 ? 0 : r0; r < rows && r <= r1; ++r) {
				for (int c = c0 < 0 ? 0 : c0; c < cols && c <= c1; ++c) {
					if (grid[r * cols + c] != 1) continue;
					const double cx = minX + (c + 0.5) * cell;
					const double cy = minY + (r + 0.5) * cell;
					if (hypot(cx - p.x, cy - p.y) <= radius) {
						grid[r * cols + c] = 2;
						++observed;
					}
				}
			}
		}
	}
	free(grid);
	zone->coverage = total == 0 ? 0 : (double) observed / total;
}

static void _checkCoverage(void) {
	for (int z = 0; z < _model->zoneCount; ++z) {
		if (_model->zones[z].swept) _computeCoverage(&_model->zones[z]);
	}
	for (int k = 0; k < _model->coverageCount; ++k) {
		const CoverageRequirement * requirement = &_model->coverage[k];
		if (requirement->zone->coverage + EPSILON < requirement->threshold) {
			simulationError("La cobertura lograda sobre la zona '%s' es de %.1f [%%], inferior al %.1f [%%] exigido.",
				requirement->zone->name, 100 * requirement->zone->coverage, 100 * requirement->threshold);
		}
	}
}

/* PUBLIC FUNCTIONS */

CompilationStatus executeSimulation(Model * model) {
	_model = model;
	_errors = 0;
	_aborted = false;
	if (model->mission == NULL) {
		logWarning(_logger, "El programa no declara ninguna misión: no hay trayectorias que simular.");
		return SUCCEEDED;
	}
	model->missionEnd = _runList(model->mission->body, model->windowStart);
	for (int k = 0; k < model->aircraftCount; ++k) {
		const Keyframe * last = _last(&model->aircraft[k]);
		if (last != NULL && model->missionEnd < last->t) model->missionEnd = last->t;
	}
	// The trajectory checks are only meaningful (and bounded) over a mission
	// that respects its order and its window.
	if (!_aborted && _errors == 0) {
		_checkKinematicsAndEnergy();
		_checkSampledTrajectories();
		_checkCoverage();
	}
	if (0 < _errors) {
		logError(_logger, "La verificación de la misión encontró %d error(es).", _errors);
		return FAILED;
	}
	for (int z = 0; z < model->zoneCount; ++z) {
		if (model->zones[z].swept) {
			logInformation(_logger, "Cobertura sobre la zona '%s': %.1f [%%].", model->zones[z].name, 100 * model->zones[z].coverage);
		}
	}
	return SUCCEEDED;
}
