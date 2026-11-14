#ifndef GEOMETRY_HEADER
#define GEOMETRY_HEADER

#include "Model.h"
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double distance2(Vec2 a, Vec2 b);
double distance3(Vec3 a, Vec3 b);

/** Signed area of a polygon (positive if counter-clockwise). */
double polygonArea(const Vec2 * vertices, int count);

/** Area centroid of a simple polygon. */
Vec2 polygonCentroid(const Vec2 * vertices, int count);

/** True if the polygon is strictly convex and non-degenerate. */
bool isConvexPolygon(const Vec2 * vertices, int count);

/** Point-in-polygon test (ray casting; works for any simple polygon). */
bool pointInPolygon(Vec2 point, const Vec2 * vertices, int count);

/** True if the point lies inside the zone (exact test for circles). */
bool pointInZone(Vec2 point, const Zone * zone);

/** Bounding box of the zone. */
void zoneBounds(const Zone * zone, double * minX, double * minY, double * maxX, double * maxY);

/**
 * Intersects the horizontal line y = constant with a convex polygon. Returns
 * false if there is no intersection; otherwise, [*xa, *xb] is the chord.
 */
bool horizontalChord(const Vec2 * vertices, int count, double y, double * xa, double * xb);

/** Minimum distance from a point to the edges of a polygon. */
double distanceToEdges(Vec2 point, const Vec2 * vertices, int count);

#endif
