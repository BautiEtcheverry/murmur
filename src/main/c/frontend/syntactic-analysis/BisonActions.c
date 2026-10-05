#include "BisonActions.h"
#include <string.h>

/* MODULE INTERNAL STATE */

static CompilerState * _compilerState = NULL;
static Logger * _logger = NULL;

void _shutdownBisonActionsModule() {
	if (_logger != NULL) {
		logDebugging(_logger, "Destroying module: BisonActions...");
		destroyLogger(_logger);
		_logger = NULL;
	}
	_compilerState = NULL;
}

ModuleDestructor initializeBisonActionsModule(CompilerState * compilerState) {
	_compilerState = compilerState;
	_logger = createLogger("BisonActions");
	return _shutdownBisonActionsModule;
}

/* HELPERS */

static void _log(const char * functionName) {
	logDebugging(_logger, "%s", functionName);
}

static Point * _clonePoint(Point p) {
	Point * copy = calloc(1, sizeof(Point));
	*copy = p;
	return copy;
}

/* PROGRAM / DECLARATIONS */

Program * ProgramSemanticAction(DeclarationList * declarations) {
	_log(__FUNCTION__);
	Program * program = calloc(1, sizeof(Program));
	program->declarations = declarations;
	_compilerState->abstractSyntaxtTree = program;
	return program;
}

DeclarationList * DeclarationListAppendSemanticAction(DeclarationList * list, Declaration * declaration) {
	_log(__FUNCTION__);
	DeclarationList * node = calloc(1, sizeof(DeclarationList));
	node->declaration = declaration;
	node->next = NULL;
	if (list == NULL) return node;
	DeclarationList * tail = list;
	while (tail->next != NULL) tail = tail->next;
	tail->next = node;
	return list;
}

static Declaration * _wrapDeclaration(DeclarationKind kind) {
	Declaration * declaration = calloc(1, sizeof(Declaration));
	declaration->kind = kind;
	return declaration;
}

Declaration * DroneDeclarationSemanticAction(DroneDef * drone) {
	_log(__FUNCTION__);
	Declaration * d = _wrapDeclaration(DECL_DRONE);
	d->drone = drone;
	return d;
}

Declaration * FleetDeclarationSemanticAction(FleetDef * fleet) {
	_log(__FUNCTION__);
	Declaration * d = _wrapDeclaration(DECL_FLEET);
	d->fleet = fleet;
	return d;
}

Declaration * ZoneDeclarationSemanticAction(ZoneDef * zone) {
	_log(__FUNCTION__);
	Declaration * d = _wrapDeclaration(DECL_ZONE);
	d->zone = zone;
	return d;
}

Declaration * StationDeclarationSemanticAction(StationDef * station) {
	_log(__FUNCTION__);
	Declaration * d = _wrapDeclaration(DECL_STATION);
	d->station = station;
	return d;
}

Declaration * FormationDeclarationSemanticAction(FormationDef * formation) {
	_log(__FUNCTION__);
	Declaration * d = _wrapDeclaration(DECL_FORMATION);
	d->formation = formation;
	return d;
}

Declaration * MissionDeclarationSemanticAction(MissionDef * mission) {
	_log(__FUNCTION__);
	Declaration * d = _wrapDeclaration(DECL_MISSION);
	d->mission = mission;
	return d;
}

/* DRONE */

DroneDef * DroneDefSemanticAction(char * name, DroneFieldList * fields) {
	_log(__FUNCTION__);
	DroneDef * drone = calloc(1, sizeof(DroneDef));
	drone->name = name;
	drone->fields = fields;
	return drone;
}

DroneFieldList * DroneFieldListAppendSemanticAction(DroneFieldList * list, DroneField * field) {
	_log(__FUNCTION__);
	DroneFieldList * node = calloc(1, sizeof(DroneFieldList));
	node->field = field;
	node->next = NULL;
	if (list == NULL) return node;
	DroneFieldList * tail = list;
	while (tail->next != NULL) tail = tail->next;
	tail->next = node;
	return list;
}

static DroneField * _quantityField(DroneFieldKind kind, Quantity q) {
	DroneField * field = calloc(1, sizeof(DroneField));
	field->kind = kind;
	field->quantity = q;
	return field;
}

DroneField * EnduranceFieldSemanticAction(Quantity q)  { _log(__FUNCTION__); return _quantityField(FIELD_ENDURANCE, q); }
DroneField * MaxSpeedFieldSemanticAction(Quantity q)   { _log(__FUNCTION__); return _quantityField(FIELD_MAX_SPEED, q); }
DroneField * MaxAccelFieldSemanticAction(Quantity q)   { _log(__FUNCTION__); return _quantityField(FIELD_MAX_ACCEL, q); }
DroneField * LinkRangeFieldSemanticAction(Quantity q)  { _log(__FUNCTION__); return _quantityField(FIELD_LINK_RANGE, q); }

DroneField * PayloadFieldSemanticAction(PayloadDef * payload) {
	_log(__FUNCTION__);
	DroneField * field = calloc(1, sizeof(DroneField));
	field->kind = FIELD_PAYLOAD;
	field->payload = payload;
	return field;
}

PayloadDef * PayloadDefSemanticAction(char * sensorType, Quantity resolution, Quantity fov) {
	_log(__FUNCTION__);
	PayloadDef * payload = calloc(1, sizeof(PayloadDef));
	payload->sensorType = sensorType;
	payload->resolution = resolution;
	payload->fov = fov;
	return payload;
}

/* FLEET */

FleetDef * FleetDefSemanticAction(char * name, FleetComponentList * components) {
	_log(__FUNCTION__);
	FleetDef * fleet = calloc(1, sizeof(FleetDef));
	fleet->name = name;
	fleet->components = components;
	return fleet;
}

FleetComponentList * FleetComponentListSingleSemanticAction(FleetComponent * component) {
	_log(__FUNCTION__);
	FleetComponentList * node = calloc(1, sizeof(FleetComponentList));
	node->component = component;
	return node;
}

FleetComponentList * FleetComponentListAppendSemanticAction(FleetComponentList * list, FleetComponent * component) {
	_log(__FUNCTION__);
	FleetComponentList * node = calloc(1, sizeof(FleetComponentList));
	node->component = component;
	if (list == NULL) return node;
	FleetComponentList * tail = list;
	while (tail->next != NULL) tail = tail->next;
	tail->next = node;
	return list;
}

FleetComponent * FleetComponentSemanticAction(long count, char * droneTypeName) {
	_log(__FUNCTION__);
	FleetComponent * c = calloc(1, sizeof(FleetComponent));
	c->count = count;
	c->droneTypeName = droneTypeName;
	return c;
}

/* ZONE */

ZoneDef * PolygonZoneSemanticAction(PointList * vertices) {
	_log(__FUNCTION__);
	ZoneDef * zone = calloc(1, sizeof(ZoneDef));
	zone->kind = ZONE_POLYGON;
	zone->vertices = vertices;
	return zone;
}

ZoneDef * CircleZoneSemanticAction(Point center, Quantity radius) {
	_log(__FUNCTION__);
	ZoneDef * zone = calloc(1, sizeof(ZoneDef));
	zone->kind = ZONE_CIRCLE;
	zone->center = center;
	zone->radius = radius;
	return zone;
}

ZoneDef * ZoneDefNameSemanticAction(char * name, ZoneDef * body) {
	_log(__FUNCTION__);
	body->name = name;
	return body;
}

PointList * PointListTripleSemanticAction(Point first, Point second, Point third) {
	_log(__FUNCTION__);
	PointList * node = calloc(1, sizeof(PointList));
	node->point = _clonePoint(first);
	node = PointListAppendSemanticAction(node, second);
	return PointListAppendSemanticAction(node, third);
}

PointList * PointListAppendSemanticAction(PointList * list, Point point) {
	_log(__FUNCTION__);
	PointList * node = calloc(1, sizeof(PointList));
	node->point = _clonePoint(point);
	if (list == NULL) return node;
	PointList * tail = list;
	while (tail->next != NULL) tail = tail->next;
	tail->next = node;
	return list;
}

Point PointSemanticAction(Numeric x, Numeric y) {
	_log(__FUNCTION__);
	Point p;
	p.x = x;
	p.y = y;
	return p;
}

/* STATION */

StationDef * StationDefSemanticAction(char * name, Point position, Quantity altitude) {
	_log(__FUNCTION__);
	StationDef * s = calloc(1, sizeof(StationDef));
	s->name = name;
	s->position = position;
	s->altitude = altitude;
	return s;
}

/* FORMATION */

FormationDef * NamedFormationSemanticAction(char * name, FormationShape * shape) {
	_log(__FUNCTION__);
	FormationDef * f = calloc(1, sizeof(FormationDef));
	f->name = name;
	f->shape = shape;
	return f;
}

FormationDef * AnonymousFormationSemanticAction(FormationShape * shape) {
	_log(__FUNCTION__);
	FormationDef * f = calloc(1, sizeof(FormationDef));
	f->name = NULL;
	f->shape = shape;
	return f;
}

FormationDef * FormationReferenceSemanticAction(char * name) {
	_log(__FUNCTION__);
	FormationDef * f = calloc(1, sizeof(FormationDef));
	f->name = name;
	f->shape = NULL;
	return f;
}

FormationShape * LineFormationShapeSemanticAction(Quantity spacing) {
	_log(__FUNCTION__);
	FormationShape * s = calloc(1, sizeof(FormationShape));
	s->kind = FORMATION_LINE;
	s->spacing = spacing;
	return s;
}

FormationShape * GridFormationShapeSemanticAction(long rows, long cols, Quantity spacing) {
	_log(__FUNCTION__);
	FormationShape * s = calloc(1, sizeof(FormationShape));
	s->kind = FORMATION_GRID;
	s->rows = rows;
	s->cols = cols;
	s->spacing = spacing;
	return s;
}

FormationShape * CircleFormationShapeSemanticAction(Quantity radius) {
	_log(__FUNCTION__);
	FormationShape * s = calloc(1, sizeof(FormationShape));
	s->kind = FORMATION_CIRCLE;
	s->radius = radius;
	return s;
}

/* MISSION */

MissionDef * MissionDefSemanticAction(char * name, Quantity windowStart, Quantity windowEnd, StatementList * body, ConstraintList * constraints) {
	_log(__FUNCTION__);
	MissionDef * m = calloc(1, sizeof(MissionDef));
	m->name = name;
	m->windowStart = windowStart;
	m->windowEnd = windowEnd;
	m->body = body;
	m->constraints = constraints;
	return m;
}

StatementList * StatementListAppendSemanticAction(StatementList * list, Statement * statement) {
	_log(__FUNCTION__);
	StatementList * node = calloc(1, sizeof(StatementList));
	node->statement = statement;
	if (list == NULL) return node;
	StatementList * tail = list;
	while (tail->next != NULL) tail = tail->next;
	tail->next = node;
	return list;
}

ConstraintList * ConstraintListAppendSemanticAction(ConstraintList * list, Constraint * constraint) {
	_log(__FUNCTION__);
	ConstraintList * node = calloc(1, sizeof(ConstraintList));
	node->constraint = constraint;
	if (list == NULL) return node;
	ConstraintList * tail = list;
	while (tail->next != NULL) tail = tail->next;
	tail->next = node;
	return list;
}

static Constraint * _quantityConstraint(ConstraintKind kind, Quantity q) {
	Constraint * c = calloc(1, sizeof(Constraint));
	c->kind = kind;
	c->quantity = q;
	return c;
}

Constraint * MinSeparationConstraintSemanticAction(Quantity q)  { _log(__FUNCTION__); return _quantityConstraint(CONSTRAINT_MIN_SEPARATION, q); }
Constraint * BatteryReserveConstraintSemanticAction(Quantity q) { _log(__FUNCTION__); return _quantityConstraint(CONSTRAINT_BATTERY_RESERVE, q); }

Constraint * ExcludeConstraintSemanticAction(char * zone) {
	_log(__FUNCTION__);
	Constraint * c = calloc(1, sizeof(Constraint));
	c->kind = CONSTRAINT_EXCLUDE;
	c->zoneName = zone;
	return c;
}

Constraint * CoverageConstraintSemanticAction(char * zone, Quantity threshold) {
	_log(__FUNCTION__);
	Constraint * c = calloc(1, sizeof(Constraint));
	c->kind = CONSTRAINT_REQUIRE_COVERAGE;
	c->zoneName = zone;
	c->quantity = threshold;
	return c;
}

/* FLEET REF */

FleetRef * AllFleetRefSemanticAction(void) {
	_log(__FUNCTION__);
	FleetRef * r = calloc(1, sizeof(FleetRef));
	r->isAll = true;
	return r;
}

FleetRef * NamedFleetRefSemanticAction(char * name) {
	_log(__FUNCTION__);
	FleetRef * r = calloc(1, sizeof(FleetRef));
	r->isAll = false;
	r->fleetName = name;
	return r;
}

FleetRef * SubFleetRefSemanticAction(char * fleetName, char * subTypeName) {
	_log(__FUNCTION__);
	FleetRef * r = calloc(1, sizeof(FleetRef));
	r->isAll = false;
	r->fleetName = fleetName;
	r->subFleetTypeName = subTypeName;
	return r;
}

/* STATEMENTS */

static Statement * _newStatement(StatementKind kind) {
	Statement * s = calloc(1, sizeof(Statement));
	s->kind = kind;
	return s;
}

Statement * TakeoffStatementSemanticAction(FleetRef * fleet, char * station, Quantity at) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_TAKEOFF);
	s->fleet = fleet;
	s->stationName = station;
	s->hasTimeAt = true;
	s->timeAt = at;
	return s;
}

Statement * LandStatementSemanticAction(FleetRef * fleet, char * station, Quantity at) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_LAND);
	s->fleet = fleet;
	s->stationName = station;
	s->hasTimeAt = true;
	s->timeAt = at;
	return s;
}

Statement * MorphStatementSemanticAction(FleetRef * fleet, FormationDef * formation, Quantity duration) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_MORPH);
	s->fleet = fleet;
	s->formation = formation;
	s->hasDuration = true;
	s->duration = duration;
	return s;
}

Statement * SweepStatementSemanticAction(FleetRef * fleet, char * zone, SweepPattern pattern, Quantity spacing, Quantity altitude) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_SWEEP);
	s->fleet = fleet;
	s->zoneName = zone;
	s->pattern = pattern;
	s->hasSpacing = true;
	s->spacing = spacing;
	s->hasAltitude = true;
	s->altitude = altitude;
	return s;
}

Statement * OrbitStatementSemanticAction(FleetRef * fleet, char * zone, Quantity radius, Quantity altitude, Quantity duration) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_ORBIT);
	s->fleet = fleet;
	s->zoneName = zone;
	s->hasRadius = true;
	s->radius = radius;
	s->hasAltitude = true;
	s->altitude = altitude;
	s->hasDuration = true;
	s->duration = duration;
	return s;
}

Statement * HoldStatementSemanticAction(FleetRef * fleet, Quantity duration) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_HOLD);
	s->fleet = fleet;
	s->hasDuration = true;
	s->duration = duration;
	return s;
}

Statement * ReturnStatementSemanticAction(FleetRef * fleet, char * station, Quantity before) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_RETURN);
	s->fleet = fleet;
	s->stationName = station;
	s->hasTimeBefore = true;
	s->timeBefore = before;
	return s;
}

Statement * ParallelStatementSemanticAction(StatementList * body) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_PARALLEL);
	s->body = body;
	return s;
}

Statement * SequenceStatementSemanticAction(StatementList * body) {
	_log(__FUNCTION__);
	Statement * s = _newStatement(STMT_SEQUENCE);
	s->body = body;
	return s;
}

/* NUMERICS */

Numeric IntegerNumericSemanticAction(long value) {
	Numeric n;
	n.kind = NUMERIC_INTEGER;
	n.integerValue = value;
	n.decimalValue = (double) value;
	return n;
}

Numeric DecimalNumericSemanticAction(double value) {
	Numeric n;
	n.kind = NUMERIC_DECIMAL;
	n.integerValue = (long) value;
	n.decimalValue = value;
	return n;
}

Numeric NegativeNumericSemanticAction(Numeric number) {
	number.integerValue = -number.integerValue;
	number.decimalValue = -number.decimalValue;
	return number;
}

Quantity QuantitySemanticAction(Numeric number, UnitKind unit) {
	Quantity q;
	q.number = number;
	q.unit = unit;
	return q;
}

/* ERRORS */

void SyntaxErrorAction(const YYLTYPE * location, const char * message) {
	logError(_logger, "Line %d: %s.", location->first_line, message);
}
