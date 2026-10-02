#ifndef BISON_ACTIONS_HEADER
#define BISON_ACTIONS_HEADER

#include "../../support/logging/Logger.h"
#include "../../support/type/CompilerState.h"
#include "../../support/type/ModuleDestructor.h"
#include "../../support/type/TokenLabel.h"
#include "AbstractSyntaxTree.h"
#include "BisonParser.h"
#include <stdlib.h>

/** Initialize module's internal state. */
ModuleDestructor initializeBisonActionsModule(CompilerState * compilerState);

/** Program & declarations. */
Program *             ProgramSemanticAction(DeclarationList * declarations);
DeclarationList *     DeclarationListAppendSemanticAction(DeclarationList * list, Declaration * declaration);
Declaration *         DroneDeclarationSemanticAction(DroneDef * drone);
Declaration *         FleetDeclarationSemanticAction(FleetDef * fleet);
Declaration *         ZoneDeclarationSemanticAction(ZoneDef * zone);
Declaration *         StationDeclarationSemanticAction(StationDef * station);
Declaration *         FormationDeclarationSemanticAction(FormationDef * formation);
Declaration *         MissionDeclarationSemanticAction(MissionDef * mission);

/** Drone. */
DroneDef *            DroneDefSemanticAction(char * name, DroneFieldList * fields);
DroneFieldList *      DroneFieldListAppendSemanticAction(DroneFieldList * list, DroneField * field);
DroneField *          EnduranceFieldSemanticAction(Quantity q);
DroneField *          MaxSpeedFieldSemanticAction(Quantity q);
DroneField *          MaxAccelFieldSemanticAction(Quantity q);
DroneField *          PayloadFieldSemanticAction(PayloadDef * payload);
DroneField *          LinkRangeFieldSemanticAction(Quantity q);
PayloadDef *          PayloadDefSemanticAction(char * sensorType, Quantity resolution, Quantity fov);

/** Fleet. */
FleetDef *            FleetDefSemanticAction(char * name, FleetComponentList * components);
FleetComponentList *  FleetComponentListSingleSemanticAction(FleetComponent * component);
FleetComponentList *  FleetComponentListAppendSemanticAction(FleetComponentList * list, FleetComponent * component);
FleetComponent *      FleetComponentSemanticAction(long count, char * droneTypeName);

/** Zone. */
ZoneDef *             PolygonZoneSemanticAction(PointList * vertices);
ZoneDef *             CircleZoneSemanticAction(Point center, Quantity radius);
ZoneDef *             ZoneDefNameSemanticAction(char * name, ZoneDef * body);
PointList *           PointListSingleSemanticAction(Point point);
PointList *           PointListAppendSemanticAction(PointList * list, Point point);
Point                 PointSemanticAction(Numeric x, Numeric y);

/** Station. */
StationDef *          StationDefSemanticAction(char * name, Point position, Quantity altitude);

/** Formation. */
FormationDef *        NamedFormationSemanticAction(char * name, FormationShape * shape);
FormationDef *        AnonymousFormationSemanticAction(FormationShape * shape);
FormationDef *        FormationReferenceSemanticAction(char * name);
FormationShape *      LineFormationShapeSemanticAction(Quantity spacing);
FormationShape *      GridFormationShapeSemanticAction(long rows, long cols, Quantity spacing);
FormationShape *      CircleFormationShapeSemanticAction(Quantity radius);

/** Mission. */
MissionDef *          MissionDefSemanticAction(char * name, Quantity windowStart, Quantity windowEnd, StatementList * body, ConstraintList * constraints);
StatementList *       StatementListAppendSemanticAction(StatementList * list, Statement * statement);
ConstraintList *      ConstraintListAppendSemanticAction(ConstraintList * list, Constraint * constraint);
Constraint *          MinSeparationConstraintSemanticAction(Quantity q);
Constraint *          BatteryReserveConstraintSemanticAction(Quantity q);
Constraint *          ExcludeConstraintSemanticAction(char * zone);
Constraint *          CoverageConstraintSemanticAction(char * zone, Quantity threshold);

/** Fleet references. */
FleetRef *            AllFleetRefSemanticAction(void);
FleetRef *            NamedFleetRefSemanticAction(char * name);
FleetRef *            SubFleetRefSemanticAction(char * fleetName, char * subTypeName);

/** Statements. */
Statement *           TakeoffStatementSemanticAction(FleetRef * fleet, char * station, Quantity at);
Statement *           LandStatementSemanticAction(FleetRef * fleet, char * station, Quantity at);
Statement *           MorphStatementSemanticAction(FleetRef * fleet, FormationDef * formation, Quantity duration);
Statement *           SweepStatementSemanticAction(FleetRef * fleet, char * zone, SweepPattern pattern, Quantity spacing, Quantity altitude);
Statement *           OrbitStatementSemanticAction(FleetRef * fleet, char * zone, Quantity radius, Quantity altitude, Quantity duration);
Statement *           HoldStatementSemanticAction(FleetRef * fleet, Quantity duration);
Statement *           ReturnStatementSemanticAction(FleetRef * fleet, char * station, Quantity before);
Statement *           ParallelStatementSemanticAction(StatementList * body);
Statement *           SequenceStatementSemanticAction(StatementList * body);

/** Numerics. */
Numeric               IntegerNumericSemanticAction(long value);
Numeric               DecimalNumericSemanticAction(double value);
Quantity              QuantitySemanticAction(Numeric number, UnitKind unit);

#endif
