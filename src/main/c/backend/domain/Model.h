#ifndef MODEL_HEADER
#define MODEL_HEADER

#include "../../frontend/syntactic-analysis/AbstractSyntaxTree.h"
#include <stdbool.h>

/**
 * The semantic model of a Murmur program. The semantic analyzer resolves the
 * AST into this structure (every magnitude converted to SI units, every
 * identifier bound to its declaration), and the later phases (simulation and
 * code generation) work exclusively over it.
 */

#define MAX_DRONE_TYPES 64
#define MAX_FLEETS 64
#define MAX_FLEET_GROUPS 16
#define MAX_ZONES 64
#define MAX_STATIONS 32
#define MAX_FORMATIONS 64
#define MAX_AIRCRAFT 256
#define MAX_COVERAGE_REQUIREMENTS 32
#define MAX_PHASES 512
#define CIRCLE_ZONE_SEGMENTS 48
#define MAX_ZONE_VERTICES 128

typedef struct Vec2 {
	double x;
	double y;
} Vec2;

typedef struct Vec3 {
	double x;
	double y;
	double z;
} Vec3;

typedef struct DroneType {
	char * name;
	double endurance;		// [s]
	double maxSpeed;		// [m/s]
	bool hasMaxAccel;
	double maxAccel;		// [m/s^2]
	bool hasPayload;
	char * sensor;
	double resolution;		// [px]
	double fov;				// [rad]
	bool hasLinkRange;
	double linkRange;		// [m]
	int nextSerial;			// Next "Type#N" serial to assign.
} DroneType;

typedef enum FlightPhase {
	PHASE_GROUND,
	PHASE_CLIMB,
	PHASE_CRUISE,
	PHASE_HOVER,
	PHASE_DESCENT
} FlightPhase;

typedef struct Keyframe {
	double t;
	Vec3 p;
	bool airborne;
	double energy;			// Cumulative battery fraction consumed up to t.
} Keyframe;

typedef struct Aircraft {
	char id[64];
	DroneType * type;
	int fleet;
	int deployed;			// Has any keyframe.
	Keyframe * frames;
	int frameCount;
	int frameCapacity;
	double batteryAtEnd;	// Remaining battery fraction [0, 1].
	double flightTime;		// [s]
	double distance;		// [m]
} Aircraft;

typedef struct FleetGroup {
	DroneType * type;
	int first;				// Index of the first aircraft of this group.
	int count;
} FleetGroup;

typedef struct Fleet {
	char * name;
	FleetGroup groups[MAX_FLEET_GROUPS];
	int groupCount;
	int first;				// Aircraft of a fleet are contiguous.
	int count;
} Fleet;

typedef struct Zone {
	char * name;
	bool isCircle;
	Vec2 center;			// Centroid, for polygons.
	double radius;			// Only for circles.
	Vec2 vertices[MAX_ZONE_VERTICES];	// Circles are approximated.
	int vertexCount;
	bool excluded;
	bool swept;
	double coverage;		// Fraction [0, 1], computed by the simulation.
} Zone;

typedef struct Station {
	char * name;
	Vec2 position;
	double altitude;
} Station;

typedef struct Formation {
	char * name;
	FormationShape * shape;
} Formation;

typedef struct CoverageRequirement {
	Zone * zone;
	double threshold;		// Fraction [0, 1].
} CoverageRequirement;

typedef struct PhaseRecord {
	char label[160];
	char maneuver[16];
	double start;
	double end;
} PhaseRecord;

typedef struct Model {
	DroneType droneTypes[MAX_DRONE_TYPES];
	int droneTypeCount;
	Fleet fleets[MAX_FLEETS];
	int fleetCount;
	Aircraft aircraft[MAX_AIRCRAFT];
	int aircraftCount;
	Zone zones[MAX_ZONES];
	int zoneCount;
	Station stations[MAX_STATIONS];
	int stationCount;
	Formation formations[MAX_FORMATIONS];
	int formationCount;

	MissionDef * mission;
	double windowStart;
	double windowEnd;
	bool hasMinSeparation;
	double minSeparation;
	bool hasBatteryReserve;
	double batteryReserve;	// Fraction [0, 1].
	CoverageRequirement coverage[MAX_COVERAGE_REQUIREMENTS];
	int coverageCount;

	PhaseRecord phases[MAX_PHASES];
	int phaseCount;
	double missionEnd;
} Model;

/** Allocates an empty model. */
Model * createModel(void);

/** Releases the model and every owned buffer (names are borrowed from the AST). */
void destroyModel(Model * model);

/** Lookups by name; return NULL if absent. */
DroneType * findDroneType(Model * model, const char * name);
Fleet * findFleet(Model * model, const char * name);
Zone * findZone(Model * model, const char * name);
Station * findStation(Model * model, const char * name);
Formation * findFormation(Model * model, const char * name);

/** Name of a flight phase, as emitted in the JSON log. */
const char * flightPhaseName(FlightPhase phase);

#endif
