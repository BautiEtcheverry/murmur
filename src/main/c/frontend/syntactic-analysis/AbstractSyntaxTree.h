#ifndef ABSTRACT_SYNTAX_TREE_HEADER
#define ABSTRACT_SYNTAX_TREE_HEADER

#include "../../support/logging/Logger.h"
#include "../../support/type/ModuleDestructor.h"
#include <stdbool.h>
#include <stdlib.h>

ModuleDestructor initializeAbstractSyntaxTreeModule();

typedef enum UnitKind UnitKind;
typedef enum NumericKind NumericKind;
typedef enum ZoneKind ZoneKind;
typedef enum FormationKind FormationKind;
typedef enum DroneFieldKind DroneFieldKind;
typedef enum StatementKind StatementKind;
typedef enum SweepPattern SweepPattern;
typedef enum ConstraintKind ConstraintKind;
typedef enum DeclarationKind DeclarationKind;

typedef struct Numeric Numeric;
typedef struct Quantity Quantity;
typedef struct Point Point;
typedef struct PointList PointList;
typedef struct PayloadDef PayloadDef;
typedef struct DroneField DroneField;
typedef struct DroneFieldList DroneFieldList;
typedef struct DroneDef DroneDef;
typedef struct FleetComponent FleetComponent;
typedef struct FleetComponentList FleetComponentList;
typedef struct FleetDef FleetDef;
typedef struct ZoneDef ZoneDef;
typedef struct StationDef StationDef;
typedef struct FormationShape FormationShape;
typedef struct FormationDef FormationDef;
typedef struct FleetRef FleetRef;
typedef struct Statement Statement;
typedef struct StatementList StatementList;
typedef struct Constraint Constraint;
typedef struct ConstraintList ConstraintList;
typedef struct MissionDef MissionDef;
typedef struct Declaration Declaration;
typedef struct DeclarationList DeclarationList;
typedef struct Program Program;

enum UnitKind {
	UNIT_SECONDS,
	UNIT_MINUTES,
	UNIT_METERS,
	UNIT_KILOMETERS,
	UNIT_METERS_PER_SECOND,
	UNIT_KILOMETERS_PER_HOUR,
	UNIT_METERS_PER_SECOND_SQUARED,
	UNIT_DEGREES,
	UNIT_RADIANS,
	UNIT_PIXELS,
	UNIT_PERCENT
};

enum NumericKind {
	NUMERIC_INTEGER,
	NUMERIC_DECIMAL
};

enum ZoneKind {
	ZONE_POLYGON,
	ZONE_CIRCLE
};

enum FormationKind {
	FORMATION_LINE,
	FORMATION_GRID,
	FORMATION_CIRCLE
};

enum DroneFieldKind {
	FIELD_ENDURANCE,
	FIELD_MAX_SPEED,
	FIELD_MAX_ACCEL,
	FIELD_PAYLOAD,
	FIELD_LINK_RANGE
};

enum StatementKind {
	STMT_TAKEOFF,
	STMT_LAND,
	STMT_MORPH,
	STMT_SWEEP,
	STMT_ORBIT,
	STMT_HOLD,
	STMT_RETURN,
	STMT_PARALLEL,
	STMT_SEQUENCE
};

enum SweepPattern {
	PATTERN_LAWNMOWER,
	PATTERN_SPIRAL,
	PATTERN_PERIMETER
};

enum ConstraintKind {
	CONSTRAINT_MIN_SEPARATION,
	CONSTRAINT_BATTERY_RESERVE,
	CONSTRAINT_EXCLUDE,
	CONSTRAINT_REQUIRE_COVERAGE
};

enum DeclarationKind {
	DECL_DRONE,
	DECL_FLEET,
	DECL_ZONE,
	DECL_STATION,
	DECL_FORMATION,
	DECL_MISSION
};

struct Numeric {
	NumericKind kind;
	long integerValue;
	double decimalValue;
};

struct Quantity {
	Numeric number;
	UnitKind unit;
};

struct Point {
	Numeric x;
	Numeric y;
};

struct PointList {
	Point * point;
	PointList * next;
};

struct PayloadDef {
	char * sensorType;
	Quantity resolution;
	Quantity fov;
};

struct DroneField {
	DroneFieldKind kind;
	Quantity quantity;
	PayloadDef * payload;
};

struct DroneFieldList {
	DroneField * field;
	DroneFieldList * next;
};

struct DroneDef {
	char * name;
	DroneFieldList * fields;
};

struct FleetComponent {
	long count;
	char * droneTypeName;
};

struct FleetComponentList {
	FleetComponent * component;
	FleetComponentList * next;
};

struct FleetDef {
	char * name;
	FleetComponentList * components;
};

struct ZoneDef {
	char * name;
	ZoneKind kind;
	PointList * vertices;
	Point center;
	Quantity radius;
};

struct StationDef {
	char * name;
	Point position;
	Quantity altitude;
};

struct FormationShape {
	FormationKind kind;
	Quantity spacing;
	long rows;
	long cols;
	Quantity radius;
};

struct FormationDef {
	char * name;
	FormationShape * shape;
};

struct FleetRef {
	bool isAll;
	char * fleetName;
	char * subFleetTypeName;
};

struct Statement {
	StatementKind kind;
	FleetRef * fleet;
	char * stationName;
	bool hasTimeAt;
	Quantity timeAt;
	bool hasTimeBefore;
	Quantity timeBefore;
	bool hasDuration;
	Quantity duration;
	FormationDef * formation;
	char * zoneName;
	SweepPattern pattern;
	bool hasSpacing;
	Quantity spacing;
	bool hasAltitude;
	Quantity altitude;
	bool hasRadius;
	Quantity radius;
	StatementList * body;
};

struct StatementList {
	Statement * statement;
	StatementList * next;
};

struct Constraint {
	ConstraintKind kind;
	Quantity quantity;
	char * zoneName;
};

struct ConstraintList {
	Constraint * constraint;
	ConstraintList * next;
};

struct MissionDef {
	char * name;
	Quantity windowStart;
	Quantity windowEnd;
	StatementList * body;
	ConstraintList * constraints;
};

struct Declaration {
	DeclarationKind kind;
	DroneDef * drone;
	FleetDef * fleet;
	ZoneDef * zone;
	StationDef * station;
	FormationDef * formation;
	MissionDef * mission;
};

struct DeclarationList {
	Declaration * declaration;
	DeclarationList * next;
};

struct Program {
	DeclarationList * declarations;
};

/** Destructors. */

void destroyPointList(PointList * list);
void destroyPayloadDef(PayloadDef * payload);
void destroyDroneField(DroneField * field);
void destroyDroneFieldList(DroneFieldList * list);
void destroyDroneDef(DroneDef * drone);
void destroyFleetComponent(FleetComponent * component);
void destroyFleetComponentList(FleetComponentList * list);
void destroyFleetDef(FleetDef * fleet);
void destroyZoneDef(ZoneDef * zone);
void destroyStationDef(StationDef * station);
void destroyFormationShape(FormationShape * shape);
void destroyFormationDef(FormationDef * formation);
void destroyFleetRef(FleetRef * ref);
void destroyStatement(Statement * statement);
void destroyStatementList(StatementList * list);
void destroyConstraint(Constraint * constraint);
void destroyConstraintList(ConstraintList * list);
void destroyMissionDef(MissionDef * mission);
void destroyDeclaration(Declaration * declaration);
void destroyDeclarationList(DeclarationList * list);
void destroyProgram(Program * program);

#endif
