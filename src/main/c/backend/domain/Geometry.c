#include "Geometry.h"
#include <float.h>
#include <math.h>

double distance2(Vec2 a, Vec2 b) {
	return hypot(a.x - b.x, a.y - b.y);
}

double distance3(Vec3 a, Vec3 b) {
	const double dx = a.x - b.x;
	const double dy = a.y - b.y;
	const double dz = a.z - b.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

double polygonArea(const Vec2 * v, int n) {
	double area = 0;
	for (int k = 0; k < n; ++k) {
		const Vec2 a = v[k];
		const Vec2 b = v[(k + 1) % n];
		area += a.x * b.y - b.x * a.y;
	}
	return area / 2.0;
}

Vec2 polygonCentroid(const Vec2 * v, int n) {
	const double area = polygonArea(v, n);
	Vec2 c = {0, 0};
	if (fabs(area) < 1e-9) {
		for (int k = 0; k < n; ++k) {
			c.x += v[k].x / n;
			c.y += v[k].y / n;
		}
		return c;
	}
	for (int k = 0; k < n; ++k) {
		const Vec2 a = v[k];
		const Vec2 b = v[(k + 1) % n];
		const double cross = a.x * b.y - b.x * a.y;
		c.x += (a.x + b.x) * cross;
		c.y += (a.y + b.y) * cross;
	}
	c.x /= 6.0 * area;
	c.y /= 6.0 * area;
	return c;
}

bool isConvexPolygon(const Vec2 * v, int n) {
	if (n < 3 || fabs(polygonArea(v, n)) < 1e-9) return false;
	int sign = 0;
	for (int k = 0; k < n; ++k) {
		const Vec2 a = v[k];
		const Vec2 b = v[(k + 1) % n];
		const Vec2 c = v[(k + 2) % n];
		const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
		if (fabs(cross) < 1e-9) continue;
		const int current = cross > 0 ? 1 : -1;
		if (sign == 0) sign = current;
		else if (sign != current) return false;
	}
	// A star-shaped polygon has turns of a single sign but winds twice.
	double turning = 0;
	for (int k = 0; k < n; ++k) {
		const Vec2 a = v[k];
		const Vec2 b = v[(k + 1) % n];
		const Vec2 c = v[(k + 2) % n];
		const double a1 = atan2(b.y - a.y, b.x - a.x);
		const double a2 = atan2(c.y - b.y, c.x - b.x);
		double d = a2 - a1;
		while (d > M_PI) d -= 2 * M_PI;
		while (d < -M_PI) d += 2 * M_PI;
		turning += d;
	}
	return fabs(fabs(turning) - 2 * M_PI) < 1e-6;
}

bool pointInPolygon(Vec2 p, const Vec2 * v, int n) {
	bool inside = false;
	for (int i = 0, j = n - 1; i < n; j = i++) {
		if (((v[i].y > p.y) != (v[j].y > p.y))
			&& (p.x < (v[j].x - v[i].x) * (p.y - v[i].y) / (v[j].y - v[i].y) + v[i].x)) {
			inside = !inside;
		}
	}
	return inside;
}

bool pointInZone(Vec2 p, const Zone * zone) {
	if (zone->isCircle) return distance2(p, zone->center) < zone->radius;
	return pointInPolygon(p, zone->vertices, zone->vertexCount);
}

void zoneBounds(const Zone * zone, double * minX, double * minY, double * maxX, double * maxY) {
	*minX = *minY = DBL_MAX;
	*maxX = *maxY = -DBL_MAX;
	for (int k = 0; k < zone->vertexCount; ++k) {
		const Vec2 p = zone->vertices[k];
		if (p.x < *minX) *minX = p.x;
		if (p.y < *minY) *minY = p.y;
		if (p.x > *maxX) *maxX = p.x;
		if (p.y > *maxY) *maxY = p.y;
	}
	if (zone->isCircle) {
		*minX = zone->center.x - zone->radius;
		*maxX = zone->center.x + zone->radius;
		*minY = zone->center.y - zone->radius;
		*maxY = zone->center.y + zone->radius;
	}
}

bool horizontalChord(const Vec2 * v, int n, double y, double * xa, double * xb) {
	double lo = DBL_MAX;
	double hi = -DBL_MAX;
	for (int k = 0; k < n; ++k) {
		const Vec2 a = v[k];
		const Vec2 b = v[(k + 1) % n];
		if ((a.y <= y && y <= b.y) || (b.y <= y && y <= a.y)) {
			double x;
			if (fabs(b.y - a.y) < 1e-12) {
				if (a.x < lo) lo = a.x;
				if (a.x > hi) hi = a.x;
				x = b.x;
			}
			else {
				x = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
			}
			if (x < lo) lo = x;
			if (x > hi) hi = x;
		}
	}
	if (lo > hi) return false;
	*xa = lo;
	*xb = hi;
	return true;
}

double distanceToEdges(Vec2 p, const Vec2 * v, int n) {
	double best = DBL_MAX;
	for (int k = 0; k < n; ++k) {
		const Vec2 a = v[k];
		const Vec2 b = v[(k + 1) % n];
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double len2 = dx * dx + dy * dy;
		double t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0;
		if (t < 0) t = 0;
		if (t > 1) t = 1;
		const Vec2 q = {a.x + t * dx, a.y + t * dy};
		const double d = distance2(p, q);
		if (d < best) best = d;
	}
	return best;
}
