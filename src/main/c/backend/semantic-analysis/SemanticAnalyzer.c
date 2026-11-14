#include "SemanticAnalyzer.h"
#include "../domain/Geometry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* MODULE INTERNAL STATE */

static Logger * _logger = NULL;
static int _errors = 0;

/** Per-zone bookkeeping used to validate coverage requirements. */
static int _sweepsOverZone[MAX_ZONES];
static int _payloadSweepsOverZone[MAX_ZONES];

void _shutdownSemanticAnalyzerModule() {
	if (_logger != NULL) {
		logDebugging(_logger, "Destroying module: SemanticAnalyzer...");
		destroyLogger(_logger);
		_logger = NULL;
	}
}

ModuleDestructor initializeSemanticAnalyzerModule() {
	_logger = createLogger("SemanticAnalyzer");
	return _shutdownSemanticAnalyzerModule;
}

/* ERRORS */

#define semanticError(...) do { ++_errors; logError(_logger, __VA_ARGS__); } while (0)

/* UNITS AND DIMENSIONS */

typedef enum Dimension {
	DIM_TIME,
	DIM_LENGTH,
	DIM_SPEED,
	DIM_ACCELERATION,
	DIM_ANGLE,
	DIM_RESOLUTION,
	DIM_RATIO
} Dimension;

static Dimension _dimensionOf(UnitKind unit) {
	switch (unit) {
		case UNIT_SECONDS: case UNIT_MINUTES: return DIM_TIME;
		case UNIT_METERS: case UNIT_KILOMETERS: return DIM_LENGTH;
		case UNIT_METERS_PER_SECOND: case UNIT_KILOMETERS_PER_HOUR: return DIM_SPEED;
		case UNIT_METERS_PER_SECOND_SQUARED: return DIM_ACCELERATION;
		case UNIT_DEGREES: case UNIT_RADIANS: return DIM_ANGLE;
		// [px] and [%] are distinct dimensions: "resolution 95 [%]" is rejected.
		case UNIT_PIXELS: return DIM_RESOLUTION;
		case UNIT_PERCENT: return DIM_RATIO;
	}
	return DIM_RATIO;
}

static double _factorOf(UnitKind unit) {
	switch (unit) {
		case UNIT_MINUTES: return 60.0;
		case UNIT_KILOMETERS: return 1000.0;
		case UNIT_KILOMETERS_PER_HOUR: return 1000.0 / 3600.0;
		case UNIT_DEGREES: return M_PI / 180.0;
		case UNIT_PERCENT: return 0.01;
		default: return 1.0;
	}
}

static const char * _unitText(UnitKind unit) {
	switch (unit) {
		case UNIT_SECONDS: return "[s]";
		case UNIT_MINUTES: return "[min]";
		case UNIT_METERS: return "[m]";
		case UNIT_KILOMETERS: return "[km]";
		case UNIT_METERS_PER_SECOND: return "[m/s]";
		case UNIT_KILOMETERS_PER_HOUR: return "[km/h]";
		case UNIT_METERS_PER_SECOND_SQUARED: return "[m/s^2]";
		case UNIT_DEGREES: return "[deg]";
		case UNIT_RADIANS: return "[rad]";
		case UNIT_PIXELS: return "[px]";
		case UNIT_PERCENT: return "[%]";
	}
	return "[?]";
}

static const char * _dimensionText(Dimension dimension) {
	switch (dimension) {
		case DIM_TIME: return "tiempo ([s], [min])";
		case DIM_LENGTH: return "longitud ([m], [km])";
		case DIM_SPEED: return "velocidad ([m/s], [km/h])";
		case DIM_ACCELERATION: return "aceleración ([m/s^2])";
		case DIM_ANGLE: return "ángulo ([deg], [rad])";
		case DIM_RESOLUTION: return "resolución ([px])";
		case DIM_RATIO: return "porcentaje ([%])";
	}
	return "?";
}

static double _numeric(Numeric n) {
	return n.kind == NUMERIC_INTEGER ? (double) n.integerValue : n.decimalValue;
}

double quantityToSI(Quantity q) {
	return _numeric(q.number) * _factorOf(q.unit);
}

/**
 * Validates the dimension of a magnitude and converts it to SI. Logs an error
 * and returns false on a dimensional mismatch.
 */
static bool _quantity(Quantity q, Dimension expected, const char * context, double * out) {
	if (_dimensionOf(q.unit) != expected) {
		semanticError("%s debe expresarse en %s, pero se utilizó %s.", context, _dimensionText(expected), _unitText(q.unit));
		*out = 0;
		return false;
	}
	*out = quantityToSI(q);
	if (!isfinite(*out) || 1e9 < fabs(*out)) {
		semanticError("%s tiene un valor fuera de rango.", context);
		return false;
	}
	return true;
}

/** Checks that the coordinates of a point are finite and in range. */
static bool _pointInRange(Point p, const char * where) {
	const double x = _numeric(p.x);
	const double y = _numeric(p.y);
	if (!isfinite(x) || !isfinite(y) || 1e7 < fabs(x) || 1e7 < fabs(y)) {
		semanticError("%s: las coordenadas del punto están fuera de rango (máximo 10^7 [m]).", where);
		return false;
	}
	return true;
}

/** Same as _quantity, but also requires a strictly positive value. */
static bool _positive(Quantity q, Dimension expected, const char * context, double * out) {
	if (!_quantity(q, expected, context, out)) return false;
	if (*out <= 0) {
		semanticError("%s debe ser mayor que cero.", context);
		return false;
	}
	return true;
}

/* NAMES */

static const char * _kindOfName(Model * model, const char * name) {
	if (findDroneType(model, name) != NULL) return "un tipo de dron";
	if (findFleet(model, name) != NULL) return "una flota";
	if (findZone(model, name) != NULL) return "una zona";
	if (findStation(model, name) != NULL) return "una estación";
	if (findFormation(model, name) != NULL) return "una formación";
	if (model->mission != NULL && strcmp(model->mission->name, name) == 0) return "una misión";
	return NULL;
}

/** Checks that a new name is not already in use. */
static bool _isFresh(Model * model, const char * name) {
	const char * kind = _kindOfName(model, name);
	if (kind != NULL) {
		semanticError("El identificador '%s' ya fue declarado previamente como %s.", name, kind);
		return false;
	}
	return true;
}

/** Explains why a name could not be resolved as the expected kind. */
static void _unresolved(Model * model, const char * name, const char * expected, const char * where) {
	const char * kind = _kindOfName(model, name);
	if (kind == NULL) {
		semanticError("%s: '%s' no fue declarado (se esperaba %s declarado previamente).", where, name, expected);
	}
	else {
		semanticError("%s: '%s' es %s, pero se esperaba %s.", where, name, kind, expected);
	}
}

/* FLEET REFERENCES */

const char * fleetRefText(const FleetRef * ref, char * buffer, size_t size) {
	if (ref->isAll) snprintf(buffer, size, "all");
	else if (ref->subFleetTypeName != NULL) snprintf(buffer, size, "%s.%s", ref->fleetName, ref->subFleetTypeName);
	else snprintf(buffer, size, "%s", ref->fleetName);
	return buffer;
}

static void _selectRange(Selection * selection, int first, int count) {
	for (int k = 0; k < count; ++k) {
		selection->indexes[selection->count++] = first + k;
	}
}

bool resolveFleetRef(Model * model, const FleetRef * ref, Selection * selection) {
	selection->count = 0;
	if (ref->isAll) {
		_selectRange(selection, 0, model->aircraftCount);
		return 0 < selection->count;
	}
	Fleet * fleet = findFleet(model, ref->fleetName);
	if (fleet == NULL) return false;
	if (ref->subFleetTypeName == NULL) {
		_selectRange(selection, fleet->first, fleet->count);
		return true;
	}
	for (int g = 0; g < fleet->groupCount; ++g) {
		if (strcmp(fleet->groups[g].type->name, ref->subFleetTypeName) == 0) {
			_selectRange(selection, fleet->groups[g].first, fleet->groups[g].count);
			return true;
		}
	}
	return false;
}

/** Same as resolveFleetRef, but explains the failure. */
static bool _resolveFleetRefOrFail(Model * model, const FleetRef * ref, Selection * selection, const char * where) {
	if (resolveFleetRef(model, ref, selection)) return true;
	if (ref->isAll) {
		semanticError("%s: 'all' no denota ninguna aeronave porque no se declaró ninguna flota.", where);
	}
	else if (findFleet(model, ref->fleetName) == NULL) {
		_unresolved(model, ref->fleetName, "una flota", where);
	}
	else {
		semanticError("%s: la flota '%s' no contiene aeronaves del tipo '%s'.", where, ref->fleetName, ref->subFleetTypeName);
	}
	return false;
}

/* DECLARATIONS */

static void _analyzeDrone(Model * model, DroneDef * def) {
	if (!_isFresh(model, def->name)) return;
	if (model->droneTypeCount == MAX_DRONE_TYPES) {
		semanticError("Se superó la cantidad máxima de tipos de dron (%d).", MAX_DRONE_TYPES);
		return;
	}
	DroneType * type = &model->droneTypes[model->droneTypeCount++];
	type->name = def->name;
	type->nextSerial = 1;
	bool seen[5] = {false, false, false, false, false};
	static const char * fieldNames[] = {"endurance", "max_speed", "max_accel", "payload", "link_range"};
	char context[256];
	for (DroneFieldList * node = def->fields; node != NULL; node = node->next) {
		DroneField * field = node->field;
		if (seen[field->kind]) {
			semanticError("El campo '%s' del dron '%s' está repetido.", fieldNames[field->kind], def->name);
			continue;
		}
		seen[field->kind] = true;
		snprintf(context, sizeof(context), "El campo '%s' del dron '%s'", fieldNames[field->kind], def->name);
		switch (field->kind) {
			case FIELD_ENDURANCE:
				_positive(field->quantity, DIM_TIME, context, &type->endurance);
				break;
			case FIELD_MAX_SPEED:
				_positive(field->quantity, DIM_SPEED, context, &type->maxSpeed);
				break;
			case FIELD_MAX_ACCEL:
				type->hasMaxAccel = _positive(field->quantity, DIM_ACCELERATION, context, &type->maxAccel);
				break;
			case FIELD_LINK_RANGE:
				type->hasLinkRange = _positive(field->quantity, DIM_LENGTH, context, &type->linkRange);
				break;
			case FIELD_PAYLOAD: {
				PayloadDef * payload = field->payload;
				type->sensor = payload->sensorType;
				snprintf(context, sizeof(context), "La resolución del sensor del dron '%s'", def->name);
				bool ok = _positive(payload->resolution, DIM_RESOLUTION, context, &type->resolution);
				snprintf(context, sizeof(context), "El campo de visión (fov) del sensor del dron '%s'", def->name);
				ok = _positive(payload->fov, DIM_ANGLE, context, &type->fov) && ok;
				if (ok && M_PI <= type->fov) {
					semanticError("%s debe ser menor que 180 [deg].", context);
					ok = false;
				}
				type->hasPayload = ok;
				break;
			}
		}
	}
	if (!seen[FIELD_ENDURANCE]) semanticError("El dron '%s' no declara el campo obligatorio 'endurance'.", def->name);
	if (!seen[FIELD_MAX_SPEED]) semanticError("El dron '%s' no declara el campo obligatorio 'max_speed'.", def->name);
}

static void _analyzeFleet(Model * model, FleetDef * def) {
	if (!_isFresh(model, def->name)) return;
	if (model->fleetCount == MAX_FLEETS) {
		semanticError("Se superó la cantidad máxima de flotas (%d).", MAX_FLEETS);
		return;
	}
	Fleet * fleet = &model->fleets[model->fleetCount++];
	fleet->name = def->name;
	fleet->first = model->aircraftCount;
	char where[256];
	snprintf(where, sizeof(where), "En la flota '%s'", def->name);
	for (FleetComponentList * node = def->components; node != NULL; node = node->next) {
		FleetComponent * component = node->component;
		DroneType * type = findDroneType(model, component->droneTypeName);
		if (type == NULL) {
			_unresolved(model, component->droneTypeName, "un tipo de dron", where);
			continue;
		}
		if (component->count <= 0) {
			semanticError("%s: la cantidad de aeronaves del tipo '%s' debe ser mayor que cero.", where, type->name);
			continue;
		}
		bool repeated = false;
		for (int g = 0; g < fleet->groupCount; ++g) {
			if (fleet->groups[g].type == type) repeated = true;
		}
		if (repeated) {
			semanticError("%s: el tipo '%s' aparece más de una vez; sume las cantidades en un único término.", where, type->name);
			continue;
		}
		if (fleet->groupCount == MAX_FLEET_GROUPS || MAX_AIRCRAFT < model->aircraftCount + component->count) {
			semanticError("%s: se superó la cantidad máxima de aeronaves (%d).", where, MAX_AIRCRAFT);
			continue;
		}
		FleetGroup * group = &fleet->groups[fleet->groupCount++];
		group->type = type;
		group->first = model->aircraftCount;
		group->count = (int) component->count;
		for (int k = 0; k < group->count; ++k) {
			Aircraft * aircraft = &model->aircraft[model->aircraftCount++];
			snprintf(aircraft->id, sizeof(aircraft->id), "%s#%d", type->name, type->nextSerial++);
			aircraft->type = type;
			aircraft->fleet = model->fleetCount - 1;
			aircraft->batteryAtEnd = 1.0;
		}
	}
	fleet->count = model->aircraftCount - fleet->first;
}

static void _analyzeZone(Model * model, ZoneDef * def) {
	if (!_isFresh(model, def->name)) return;
	if (model->zoneCount == MAX_ZONES) {
		semanticError("Se superó la cantidad máxima de zonas (%d).", MAX_ZONES);
		return;
	}
	Zone * zone = &model->zones[model->zoneCount];
	zone->name = def->name;
	if (def->kind == ZONE_CIRCLE) {
		char context[256];
		snprintf(context, sizeof(context), "El radio de la zona '%s'", def->name);
		if (!_positive(def->radius, DIM_LENGTH, context, &zone->radius)) return;
		if (!_pointInRange(def->center, def->name)) return;
		zone->isCircle = true;
		zone->center.x = _numeric(def->center.x);
		zone->center.y = _numeric(def->center.y);
		zone->vertexCount = CIRCLE_ZONE_SEGMENTS;
		for (int k = 0; k < CIRCLE_ZONE_SEGMENTS; ++k) {
			const double angle = 2 * M_PI * k / CIRCLE_ZONE_SEGMENTS;
			zone->vertices[k].x = zone->center.x + zone->radius * cos(angle);
			zone->vertices[k].y = zone->center.y + zone->radius * sin(angle);
		}
	}
	else {
		int count = 0;
		for (PointList * node = def->vertices; node != NULL; node = node->next) {
			if (count == MAX_ZONE_VERTICES) {
				semanticError("La zona '%s' supera la cantidad máxima de vértices (%d).", def->name, MAX_ZONE_VERTICES);
				return;
			}
			if (!_pointInRange(*node->point, def->name)) return;
			zone->vertices[count].x = _numeric(node->point->x);
			zone->vertices[count].y = _numeric(node->point->y);
			++count;
		}
		zone->vertexCount = count;
		if (count < 3) {
			semanticError("La zona poligonal '%s' tiene %d vértice(s), pero un polígono requiere al menos 3.", def->name, count);
			return;
		}
		if (!isConvexPolygon(zone->vertices, count)) {
			semanticError("La zona poligonal '%s' no es convexa (o es degenerada); solo se admiten polígonos convexos.", def->name);
			return;
		}
		zone->center = polygonCentroid(zone->vertices, count);
	}
	model->zoneCount++;
}

static void _analyzeStation(Model * model, StationDef * def) {
	if (!_isFresh(model, def->name)) return;
	if (model->stationCount == MAX_STATIONS) {
		semanticError("Se superó la cantidad máxima de estaciones (%d).", MAX_STATIONS);
		return;
	}
	if (!_pointInRange(def->position, def->name)) return;
	Station * station = &model->stations[model->stationCount];
	station->name = def->name;
	station->position.x = _numeric(def->position.x);
	station->position.y = _numeric(def->position.y);
	char context[256];
	snprintf(context, sizeof(context), "La altitud de la estación '%s'", def->name);
	if (!_quantity(def->altitude, DIM_LENGTH, context, &station->altitude)) return;
	if (station->altitude < 0) {
		semanticError("%s no puede ser negativa.", context);
		return;
	}
	model->stationCount++;
}

static bool _analyzeFormationShape(FormationShape * shape, const char * name) {
	char context[256];
	double value;
	switch (shape->kind) {
		case FORMATION_LINE:
			snprintf(context, sizeof(context), "El espaciado de la formación %s", name);
			return _positive(shape->spacing, DIM_LENGTH, context, &value);
		case FORMATION_GRID:
			if (shape->rows <= 0 || shape->cols <= 0 || MAX_AIRCRAFT < shape->rows || MAX_AIRCRAFT < shape->cols) {
				semanticError("Las dimensiones de la grilla %s deben estar entre 1 y %d.", name, MAX_AIRCRAFT);
				return false;
			}
			snprintf(context, sizeof(context), "El espaciado de la formación %s", name);
			return _positive(shape->spacing, DIM_LENGTH, context, &value);
		case FORMATION_CIRCLE:
			snprintf(context, sizeof(context), "El radio de la formación %s", name);
			return _positive(shape->radius, DIM_LENGTH, context, &value);
	}
	return false;
}

static void _analyzeFormation(Model * model, FormationDef * def) {
	if (!_isFresh(model, def->name)) return;
	if (model->formationCount == MAX_FORMATIONS) {
		semanticError("Se superó la cantidad máxima de formaciones (%d).", MAX_FORMATIONS);
		return;
	}
	char name[256];
	snprintf(name, sizeof(name), "'%s'", def->name);
	if (!_analyzeFormationShape(def->shape, name)) return;
	Formation * formation = &model->formations[model->formationCount++];
	formation->name = def->name;
	formation->shape = def->shape;
}

/* MISSION */

static const char * _statementName(StatementKind kind) {
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

/** Checks that an absolute instant lies inside the mission window. */
static void _checkInstant(Model * model, Quantity q, const char * where, const char * clause) {
	char context[512];
	snprintf(context, sizeof(context), "%s: el instante de la cláusula '%s'", where, clause);
	double t;
	if (!_quantity(q, DIM_TIME, context, &t)) return;
	if (t < model->windowStart || model->windowEnd < t) {
		semanticError("%s (%g [s]) queda fuera de la ventana de la misión [%g [s], %g [s]].",
			context, t, model->windowStart, model->windowEnd);
	}
}

/** A declared duration must be positive and fit inside the mission window. */
static void _checkDuration(Model * model, Quantity q, const char * context) {
	double value;
	if (!_positive(q, DIM_TIME, context, &value)) return;
	if (model->windowEnd - model->windowStart < value) {
		semanticError("%s (%g [s]) es mayor que la ventana de la misión (%g [s]).", context, value, model->windowEnd - model->windowStart);
	}
}

static void _analyzeStatementList(Model * model, StatementList * list, bool * touched, int depth);

/**
 * Analyzes a statement and marks, in "touched", every aircraft that the
 * statement (or any of its nested statements) commands.
 */
#define MAX_NESTING 64

static void _analyzeStatement(Model * model, Statement * statement, bool * touched, int depth) {
	char where[512];
	char ref[160];
	char context[640];
	double value;
	Selection selection;
	selection.count = 0;

	if (MAX_NESTING < depth) {
		semanticError("Los bloques parallel/sequence superan la profundidad máxima de anidamiento (%d).", MAX_NESTING);
		return;
	}
	if (statement->kind == STMT_PARALLEL) {
		// The same aircraft cannot execute two maneuvers at the same time.
		// Allocated per call: parallel blocks can be nested.
		bool (* branchSets)[MAX_AIRCRAFT] = calloc(64, sizeof(* branchSets));
		Statement * branches[64];
		int branchCount = 0;
		for (StatementList * node = statement->body; node != NULL; node = node->next) {
			if (branchCount == 64) {
				semanticError("Un bloque parallel admite como máximo 64 sentencias.");
				break;
			}
			_analyzeStatement(model, node->statement, branchSets[branchCount], depth + 1);
			branches[branchCount++] = node->statement;
		}
		for (int i = 0; i < branchCount; ++i) {
			for (int j = i + 1; j < branchCount; ++j) {
				for (int a = 0; a < model->aircraftCount; ++a) {
					if (branchSets[i][a] && branchSets[j][a]) {
						semanticError("Dentro de un bloque parallel, las sentencias %d ('%s') y %d ('%s') comandan simultáneamente a la aeronave %s.",
							i + 1, _statementName(branches[i]->kind), j + 1, _statementName(branches[j]->kind), model->aircraft[a].id);
						break;
					}
				}
			}
		}
		for (int i = 0; i < branchCount; ++i) {
			for (int a = 0; a < model->aircraftCount; ++a) {
				touched[a] = touched[a] || branchSets[i][a];
			}
		}
		free(branchSets);
		return;
	}
	if (statement->kind == STMT_SEQUENCE) {
		_analyzeStatementList(model, statement->body, touched, depth + 1);
		return;
	}

	snprintf(where, sizeof(where), "En la maniobra '%s %s'", fleetRefText(statement->fleet, ref, sizeof(ref)), _statementName(statement->kind));
	if (!_resolveFleetRefOrFail(model, statement->fleet, &selection, where)) return;
	for (int k = 0; k < selection.count; ++k) touched[selection.indexes[k]] = true;

	switch (statement->kind) {
		case STMT_TAKEOFF:
		case STMT_LAND:
			if (findStation(model, statement->stationName) == NULL) {
				_unresolved(model, statement->stationName, "una estación", where);
			}
			_checkInstant(model, statement->timeAt, where, "at");
			break;
		case STMT_RETURN:
			if (findStation(model, statement->stationName) == NULL) {
				_unresolved(model, statement->stationName, "una estación", where);
			}
			_checkInstant(model, statement->timeBefore, where, "before");
			break;
		case STMT_MORPH: {
			FormationDef * target = statement->formation;
			FormationShape * shape = target->shape;
			if (shape == NULL) {
				Formation * formation = findFormation(model, target->name);
				if (formation == NULL) {
					_unresolved(model, target->name, "una formación", where);
				}
				else {
					shape = formation->shape;
				}
			}
			else if (!_analyzeFormationShape(shape, "anónima")) {
				shape = NULL;
			}
			if (shape != NULL && shape->kind == FORMATION_GRID && shape->rows * shape->cols != selection.count) {
				semanticError("%s: la grilla %ld x %ld define %ld posiciones, pero '%s' tiene %d aeronave(s).",
					where, shape->rows, shape->cols, shape->rows * shape->cols, ref, selection.count);
			}
			snprintf(context, sizeof(context), "%s: la duración de la cláusula 'over'", where);
			_checkDuration(model, statement->duration, context);
			break;
		}
		case STMT_SWEEP: {
			Zone * zone = findZone(model, statement->zoneName);
			if (zone == NULL) {
				_unresolved(model, statement->zoneName, "una zona", where);
			}
			else {
				const int index = (int) (zone - model->zones);
				zone->swept = true;
				_sweepsOverZone[index]++;
				for (int k = 0; k < selection.count; ++k) {
					if (model->aircraft[selection.indexes[k]].type->hasPayload) {
						_payloadSweepsOverZone[index]++;
						break;
					}
				}
				if (zone->excluded) {
					semanticError("%s: la zona '%s' está excluida en el bloque constraints y no puede barrerse.", where, zone->name);
				}
			}
			snprintf(context, sizeof(context), "%s: el espaciado", where);
			_positive(statement->spacing, DIM_LENGTH, context, &value);
			snprintf(context, sizeof(context), "%s: la altitud", where);
			_positive(statement->altitude, DIM_LENGTH, context, &value);
			break;
		}
		case STMT_ORBIT:
			if (findZone(model, statement->zoneName) == NULL) {
				_unresolved(model, statement->zoneName, "una zona", where);
			}
			snprintf(context, sizeof(context), "%s: el radio", where);
			if (_positive(statement->radius, DIM_LENGTH, context, &value) && value < 1.0) {
				semanticError("%s debe ser de al menos 1 [m].", context);
			}
			snprintf(context, sizeof(context), "%s: la altitud", where);
			_positive(statement->altitude, DIM_LENGTH, context, &value);
			snprintf(context, sizeof(context), "%s: la duración de la cláusula 'for'", where);
			_checkDuration(model, statement->duration, context);
			break;
		case STMT_HOLD:
			snprintf(context, sizeof(context), "%s: la duración de la cláusula 'for'", where);
			_checkDuration(model, statement->duration, context);
			break;
		default:
			break;
	}
}

static void _analyzeStatementList(Model * model, StatementList * list, bool * touched, int depth) {
	for (StatementList * node = list; node != NULL; node = node->next) {
		_analyzeStatement(model, node->statement, touched, depth);
	}
}

static void _analyzeConstraints(Model * model, MissionDef * mission) {
	char context[256];
	for (ConstraintList * node = mission->constraints; node != NULL; node = node->next) {
		Constraint * constraint = node->constraint;
		switch (constraint->kind) {
			case CONSTRAINT_MIN_SEPARATION:
				if (model->hasMinSeparation) {
					semanticError("La restricción 'min_separation' está repetida.");
					break;
				}
				model->hasMinSeparation = _positive(constraint->quantity, DIM_LENGTH, "La restricción 'min_separation'", &model->minSeparation);
				break;
			case CONSTRAINT_BATTERY_RESERVE:
				if (model->hasBatteryReserve) {
					semanticError("La restricción 'battery_reserve' está repetida.");
					break;
				}
				if (_quantity(constraint->quantity, DIM_RATIO, "La restricción 'battery_reserve'", &model->batteryReserve)) {
					if (model->batteryReserve < 0 || 1 <= model->batteryReserve) {
						semanticError("La restricción 'battery_reserve' debe estar en el intervalo [0 [%%], 100 [%%]).");
					}
					else {
						model->hasBatteryReserve = true;
					}
				}
				break;
			case CONSTRAINT_EXCLUDE: {
				Zone * zone = findZone(model, constraint->zoneName);
				if (zone == NULL) {
					_unresolved(model, constraint->zoneName, "una zona", "En la restricción 'exclude'");
				}
				else if (zone->excluded) {
					semanticError("La zona '%s' se excluye más de una vez.", zone->name);
				}
				else {
					zone->excluded = true;
				}
				break;
			}
			case CONSTRAINT_REQUIRE_COVERAGE: {
				Zone * zone = findZone(model, constraint->zoneName);
				if (zone == NULL) {
					_unresolved(model, constraint->zoneName, "una zona", "En la restricción 'require coverage'");
					break;
				}
				snprintf(context, sizeof(context), "El umbral de cobertura de la zona '%s'", zone->name);
				double threshold;
				if (!_positive(constraint->quantity, DIM_RATIO, context, &threshold)) break;
				if (1 < threshold) {
					semanticError("%s no puede superar 100 [%%].", context);
					break;
				}
				if (model->coverageCount == MAX_COVERAGE_REQUIREMENTS) {
					semanticError("Se superó la cantidad máxima de requisitos de cobertura (%d).", MAX_COVERAGE_REQUIREMENTS);
					break;
				}
				model->coverage[model->coverageCount].zone = zone;
				model->coverage[model->coverageCount].threshold = threshold;
				model->coverageCount++;
				break;
			}
		}
	}
}

static void _analyzeMission(Model * model, MissionDef * mission) {
	if (model->mission != NULL) {
		semanticError("El programa declara más de una misión ('%s' y '%s'); solo se admite una misión por programa.",
			model->mission->name, mission->name);
		return;
	}
	if (!_isFresh(model, mission->name)) return;
	model->mission = mission;
	bool okStart = _quantity(mission->windowStart, DIM_TIME, "El inicio de la ventana de la misión", &model->windowStart);
	bool okEnd = _quantity(mission->windowEnd, DIM_TIME, "El fin de la ventana de la misión", &model->windowEnd);
	if (okStart && okEnd && model->windowEnd <= model->windowStart) {
		semanticError("La ventana de la misión '%s' es vacía: %g [s] .. %g [s].", mission->name, model->windowStart, model->windowEnd);
	}
	if (okStart && model->windowStart < 0) {
		semanticError("La ventana de la misión '%s' no puede comenzar en un instante negativo.", mission->name);
	}
	// Constraints first: sweeps must know which zones are excluded.
	_analyzeConstraints(model, mission);
	bool * touched = calloc(MAX_AIRCRAFT, sizeof(bool));
	_analyzeStatementList(model, mission->body, touched, 0);
	free(touched);
	for (int k = 0; k < model->coverageCount; ++k) {
		Zone * zone = model->coverage[k].zone;
		const int index = (int) (zone - model->zones);
		if (_sweepsOverZone[index] == 0) {
			semanticError("Se exige cobertura sobre la zona '%s', pero ninguna maniobra 'sweep' la barre.", zone->name);
		}
		else if (_payloadSweepsOverZone[index] == 0) {
			semanticError("Se exige cobertura sobre la zona '%s', pero ninguna de las aeronaves que la barren declara 'payload' (sin sensor no hay cobertura).", zone->name);
		}
		if (zone->excluded) {
			semanticError("Se exige cobertura sobre la zona '%s', pero la zona está excluida.", zone->name);
		}
	}
}

/* PUBLIC FUNCTIONS */

CompilationStatus executeSemanticAnalysis(Program * program, Model * model) {
	_errors = 0;
	memset(_sweepsOverZone, 0, sizeof(_sweepsOverZone));
	memset(_payloadSweepsOverZone, 0, sizeof(_payloadSweepsOverZone));
	if (program == NULL) return FAILED;
	for (DeclarationList * node = program->declarations; node != NULL; node = node->next) {
		Declaration * declaration = node->declaration;
		switch (declaration->kind) {
			case DECL_DRONE: _analyzeDrone(model, declaration->drone); break;
			case DECL_FLEET: _analyzeFleet(model, declaration->fleet); break;
			case DECL_ZONE: _analyzeZone(model, declaration->zone); break;
			case DECL_STATION: _analyzeStation(model, declaration->station); break;
			case DECL_FORMATION: _analyzeFormation(model, declaration->formation); break;
			case DECL_MISSION: _analyzeMission(model, declaration->mission); break;
		}
	}
	if (0 < _errors) {
		logError(_logger, "El análisis semántico encontró %d error(es).", _errors);
		return FAILED;
	}
	return SUCCEEDED;
}
