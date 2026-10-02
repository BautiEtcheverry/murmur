%{

#include "../../support/type/TokenLabel.h"
#include "AbstractSyntaxTree.h"
#include "BisonActions.h"

void yyerror(const YYLTYPE * location, const char * message) {}

%}

// You touch this, and you die.
%define api.pure full
%define api.push-pull push
%define api.value.union.name SemanticValue
%define parse.error detailed
%locations

%union {
	/** Lexical attributes. */
	long integer;
	double decimal;
	char * string;
	UnitKind unit;
	TokenLabel token;

	/** Value-typed non-terminals. */
	Numeric numeric;
	Quantity quantity;
	Point point;
	SweepPattern sweepPattern;

	/** Pointer-typed non-terminals. */
	PointList * pointList;
	PayloadDef * payloadDef;
	DroneField * droneField;
	DroneFieldList * droneFieldList;
	DroneDef * droneDef;
	FleetComponent * fleetComponent;
	FleetComponentList * fleetComponentList;
	FleetDef * fleetDef;
	ZoneDef * zoneDef;
	StationDef * stationDef;
	FormationShape * formationShape;
	FormationDef * formationDef;
	FleetRef * fleetRef;
	Statement * statement;
	StatementList * statementList;
	Constraint * constraint;
	ConstraintList * constraintList;
	MissionDef * missionDef;
	Declaration * declaration;
	DeclarationList * declarationList;
	Program * program;
}

/**
 * Destructors: free partial AST fragments when the parser rolls back after an
 * error. We skip <program> on purpose; the whole tree is owned by the
 * CompilerState after a successful parse.
 */
%destructor { if ($$ != NULL) free($$); } <string>
%destructor { destroyPointList($$); } <pointList>
%destructor { destroyPayloadDef($$); } <payloadDef>
%destructor { destroyDroneField($$); } <droneField>
%destructor { destroyDroneFieldList($$); } <droneFieldList>
%destructor { destroyDroneDef($$); } <droneDef>
%destructor { destroyFleetComponent($$); } <fleetComponent>
%destructor { destroyFleetComponentList($$); } <fleetComponentList>
%destructor { destroyFleetDef($$); } <fleetDef>
%destructor { destroyZoneDef($$); } <zoneDef>
%destructor { destroyStationDef($$); } <stationDef>
%destructor { destroyFormationShape($$); } <formationShape>
%destructor { destroyFormationDef($$); } <formationDef>
%destructor { destroyFleetRef($$); } <fleetRef>
%destructor { destroyStatement($$); } <statement>
%destructor { destroyStatementList($$); } <statementList>
%destructor { destroyConstraint($$); } <constraint>
%destructor { destroyConstraintList($$); } <constraintList>
%destructor { destroyMissionDef($$); } <missionDef>
%destructor { destroyDeclaration($$); } <declaration>
%destructor { destroyDeclarationList($$); } <declarationList>

/** Terminals. */
%token <integer>  INTEGER
%token <decimal>  DECIMAL
%token <string>   IDENTIFIER
%token <unit>     UNIT

%token <token> OPEN_BRACE
%token <token> CLOSE_BRACE
%token <token> OPEN_PARENTHESIS
%token <token> CLOSE_PARENTHESIS
%token <token> OPEN_BRACKET
%token <token> CLOSE_BRACKET
%token <token> COMMA
%token <token> DOT
%token <token> DOTDOT
%token <token> SEMICOLON
%token <token> EQUALS
%token <token> PLUS
%token <token> GE
%token <token> BY

%token <token> DRONE
%token <token> FLEET
%token <token> ZONE
%token <token> STATION
%token <token> FORMATION
%token <token> MISSION
%token <token> CONSTRAINTS
%token <token> PARALLEL
%token <token> SEQUENCE
%token <token> POLYGON
%token <token> CIRCLE
%token <token> POINT
%token <token> LINE
%token <token> GRID
%token <token> RADIUS
%token <token> ALTITUDE
%token <token> AT
%token <token> OF
%token <token> ENDURANCE
%token <token> MAX_SPEED
%token <token> MAX_ACCEL
%token <token> PAYLOAD
%token <token> LINK_RANGE
%token <token> RESOLUTION
%token <token> FOV
%token <token> SPACING
%token <token> WINDOW
%token <token> TAKEOFF
%token <token> LAND
%token <token> FROM
%token <token> TO
%token <token> MORPH
%token <token> SWEEP
%token <token> ORBIT
%token <token> HOLD
%token <token> RETURN
%token <token> OVER
%token <token> FOR
%token <token> BEFORE
%token <token> WITH
%token <token> LAWNMOWER
%token <token> SPIRAL
%token <token> PERIMETER
%token <token> MIN_SEPARATION
%token <token> BATTERY_RESERVE
%token <token> EXCLUDE
%token <token> REQUIRE
%token <token> COVERAGE
%token <token> ALL_KW

%token <token> IGNORED
%token <token> UNKNOWN

/** Non-terminals. */
%type <program>              program
%type <declarationList>      declarationList
%type <declaration>          declaration
%type <droneDef>             droneDeclaration
%type <droneFieldList>       droneFieldList
%type <droneField>           droneField
%type <payloadDef>           payloadDef
%type <fleetDef>             fleetDeclaration
%type <fleetComponentList>   fleetComposition
%type <fleetComponent>       fleetComponent
%type <zoneDef>              zoneDeclaration
%type <zoneDef>              zoneBody
%type <pointList>            pointList
%type <point>                point
%type <stationDef>           stationDeclaration
%type <formationDef>         formationDeclaration
%type <formationShape>       formationShape
%type <formationDef>         formationTarget
%type <missionDef>           missionDeclaration
%type <statementList>        statementList
%type <constraintList>       optionalConstraints
%type <constraintList>       constraintList
%type <constraint>           constraint
%type <statement>            statement
%type <statement>            parallelBlock
%type <statement>            sequenceBlock
%type <statement>            takeoffStmt
%type <statement>            landStmt
%type <statement>            morphStmt
%type <statement>            sweepStmt
%type <statement>            orbitStmt
%type <statement>            holdStmt
%type <statement>            returnStmt
%type <fleetRef>             fleetRef
%type <sweepPattern>         sweepPattern
%type <numeric>              number
%type <quantity>             quantity

%%

// IMPORTANT: To use λ in the following grammar, use the %empty symbol.

program:
	declarationList                                { $$ = ProgramSemanticAction($1); }
	;

declarationList:
	%empty                                          { $$ = NULL; }
	| declarationList declaration                  { $$ = DeclarationListAppendSemanticAction($1, $2); }
	;

declaration:
	droneDeclaration optSemi                      { $$ = DroneDeclarationSemanticAction($1); }
	| fleetDeclaration optSemi                    { $$ = FleetDeclarationSemanticAction($1); }
	| zoneDeclaration optSemi                     { $$ = ZoneDeclarationSemanticAction($1); }
	| stationDeclaration optSemi                  { $$ = StationDeclarationSemanticAction($1); }
	| formationDeclaration optSemi                { $$ = FormationDeclarationSemanticAction($1); }
	| missionDeclaration optSemi                  { $$ = MissionDeclarationSemanticAction($1); }
	;

optSemi:
	%empty
	| SEMICOLON
	;

/* === Drone === */

droneDeclaration:
	DRONE IDENTIFIER OPEN_BRACE droneFieldList CLOSE_BRACE
	                                                { $$ = DroneDefSemanticAction($2, $4); }
	;

droneFieldList:
	%empty                                          { $$ = NULL; }
	| droneFieldList droneField optSemi         { $$ = DroneFieldListAppendSemanticAction($1, $2); }
	;

droneField:
	ENDURANCE quantity                              { $$ = EnduranceFieldSemanticAction($2); }
	| MAX_SPEED quantity                            { $$ = MaxSpeedFieldSemanticAction($2); }
	| MAX_ACCEL quantity                            { $$ = MaxAccelFieldSemanticAction($2); }
	| PAYLOAD payloadDef                           { $$ = PayloadFieldSemanticAction($2); }
	| LINK_RANGE quantity                           { $$ = LinkRangeFieldSemanticAction($2); }
	;

payloadDef:
	IDENTIFIER RESOLUTION quantity FOV quantity     { $$ = PayloadDefSemanticAction($1, $3, $5); }
	;

/* === Fleet === */

fleetDeclaration:
	FLEET IDENTIFIER EQUALS fleetComposition       { $$ = FleetDefSemanticAction($2, $4); }
	;

fleetComposition:
	fleetComponent                                 { $$ = FleetComponentListSingleSemanticAction($1); }
	| fleetComposition PLUS fleetComponent        { $$ = FleetComponentListAppendSemanticAction($1, $3); }
	;

fleetComponent:
	INTEGER OF IDENTIFIER                           { $$ = FleetComponentSemanticAction($1, $3); }
	;

/* === Zone === */

zoneDeclaration:
	ZONE IDENTIFIER EQUALS zoneBody                { $$ = ZoneDefNameSemanticAction($2, $4); }
	;

zoneBody:
	POLYGON OPEN_BRACKET pointList CLOSE_BRACKET   { $$ = PolygonZoneSemanticAction($3); }
	| CIRCLE AT point RADIUS quantity               { $$ = CircleZoneSemanticAction($3, $5); }
	;

pointList:
	point                                           { $$ = PointListSingleSemanticAction($1); }
	| pointList COMMA point                        { $$ = PointListAppendSemanticAction($1, $3); }
	;

point:
	OPEN_PARENTHESIS number COMMA number CLOSE_PARENTHESIS
	                                                { $$ = PointSemanticAction($2, $4); }
	;

/* === Station === */

stationDeclaration:
	STATION IDENTIFIER EQUALS POINT point ALTITUDE quantity
	                                                { $$ = StationDefSemanticAction($2, $5, $7); }
	;

/* === Formation === */

formationDeclaration:
	FORMATION IDENTIFIER EQUALS formationShape     { $$ = NamedFormationSemanticAction($2, $4); }
	;

formationShape:
	LINE SPACING quantity                           { $$ = LineFormationShapeSemanticAction($3); }
	| GRID INTEGER BY INTEGER SPACING quantity      { $$ = GridFormationShapeSemanticAction($2, $4, $6); }
	| CIRCLE RADIUS quantity                        { $$ = CircleFormationShapeSemanticAction($3); }
	;

formationTarget:
	IDENTIFIER                                      { $$ = FormationReferenceSemanticAction($1); }
	| formationShape                               { $$ = AnonymousFormationSemanticAction($1); }
	;

/* === Mission === */

missionDeclaration:
	MISSION IDENTIFIER WINDOW quantity DOTDOT quantity OPEN_BRACE statementList optionalConstraints CLOSE_BRACE
	                                                { $$ = MissionDefSemanticAction($2, $4, $6, $8, $9); }
	;

optionalConstraints:
	%empty                                          { $$ = NULL; }
	| CONSTRAINTS OPEN_BRACE constraintList CLOSE_BRACE optSemi
	                                                { $$ = $3; }
	;

constraintList:
	%empty                                          { $$ = NULL; }
	| constraintList constraint optSemi           { $$ = ConstraintListAppendSemanticAction($1, $2); }
	;

constraint:
	MIN_SEPARATION quantity                         { $$ = MinSeparationConstraintSemanticAction($2); }
	| BATTERY_RESERVE quantity                      { $$ = BatteryReserveConstraintSemanticAction($2); }
	| EXCLUDE IDENTIFIER                            { $$ = ExcludeConstraintSemanticAction($2); }
	| REQUIRE COVERAGE OF IDENTIFIER GE quantity    { $$ = CoverageConstraintSemanticAction($4, $6); }
	;

statementList:
	%empty                                          { $$ = NULL; }
	| statementList statement optSemi             { $$ = StatementListAppendSemanticAction($1, $2); }
	;

statement:
	takeoffStmt                                    { $$ = $1; }
	| landStmt                                     { $$ = $1; }
	| morphStmt                                    { $$ = $1; }
	| sweepStmt                                    { $$ = $1; }
	| orbitStmt                                    { $$ = $1; }
	| holdStmt                                     { $$ = $1; }
	| returnStmt                                   { $$ = $1; }
	| parallelBlock                                { $$ = $1; }
	| sequenceBlock                                { $$ = $1; }
	;

parallelBlock:
	PARALLEL OPEN_BRACE statementList CLOSE_BRACE  { $$ = ParallelStatementSemanticAction($3); }
	;

sequenceBlock:
	SEQUENCE OPEN_BRACE statementList CLOSE_BRACE  { $$ = SequenceStatementSemanticAction($3); }
	;

fleetRef:
	ALL_KW                                          { $$ = AllFleetRefSemanticAction(); }
	| IDENTIFIER                                    { $$ = NamedFleetRefSemanticAction($1); }
	| IDENTIFIER DOT IDENTIFIER                     { $$ = SubFleetRefSemanticAction($1, $3); }
	;

takeoffStmt:
	fleetRef TAKEOFF FROM IDENTIFIER AT quantity   { $$ = TakeoffStatementSemanticAction($1, $4, $6); }
	;

landStmt:
	fleetRef LAND AT IDENTIFIER AT quantity        { $$ = LandStatementSemanticAction($1, $4, $6); }
	;

morphStmt:
	fleetRef MORPH TO formationTarget OVER quantity
	                                                { $$ = MorphStatementSemanticAction($1, $4, $6); }
	;

sweepStmt:
	fleetRef SWEEP IDENTIFIER WITH sweepPattern SPACING quantity ALTITUDE quantity
	                                                { $$ = SweepStatementSemanticAction($1, $3, $5, $7, $9); }
	;

sweepPattern:
	LAWNMOWER                                       { $$ = PATTERN_LAWNMOWER; }
	| SPIRAL                                        { $$ = PATTERN_SPIRAL; }
	| PERIMETER                                     { $$ = PATTERN_PERIMETER; }
	;

orbitStmt:
	fleetRef ORBIT IDENTIFIER RADIUS quantity ALTITUDE quantity FOR quantity
	                                                { $$ = OrbitStatementSemanticAction($1, $3, $5, $7, $9); }
	;

holdStmt:
	fleetRef HOLD FOR quantity                     { $$ = HoldStatementSemanticAction($1, $4); }
	;

returnStmt:
	fleetRef RETURN TO IDENTIFIER BEFORE quantity  { $$ = ReturnStatementSemanticAction($1, $4, $6); }
	;

/* === Numerics === */

quantity:
	number UNIT                                     { $$ = QuantitySemanticAction($1, $2); }
	;

number:
	INTEGER                                         { $$ = IntegerNumericSemanticAction($1); }
	| DECIMAL                                       { $$ = DecimalNumericSemanticAction($1); }
	;

%%
