#include "AbstractSyntaxTree.h"

/* MODULE INTERNAL STATE */

static Logger * _logger = NULL;

void _shutdownAbstractSyntaxTreeModule() {
	if (_logger != NULL) {
		logDebugging(_logger, "Destroying module: AbstractSyntaxTree...");
		destroyLogger(_logger);
		_logger = NULL;
	}
}

ModuleDestructor initializeAbstractSyntaxTreeModule() {
	_logger = createLogger("AbstractSyntaxTree");
	return _shutdownAbstractSyntaxTreeModule;
}

/* PUBLIC FUNCTIONS */

void destroyPointList(PointList * list) {
	while (list != NULL) {
		PointList * next = list->next;
		if (list->point != NULL) free(list->point);
		free(list);
		list = next;
	}
}

void destroyPayloadDef(PayloadDef * payload) {
	if (payload == NULL) return;
	if (payload->sensorType != NULL) free(payload->sensorType);
	free(payload);
}

void destroyDroneField(DroneField * field) {
	if (field == NULL) return;
	if (field->kind == FIELD_PAYLOAD) destroyPayloadDef(field->payload);
	free(field);
}

void destroyDroneFieldList(DroneFieldList * list) {
	while (list != NULL) {
		DroneFieldList * next = list->next;
		destroyDroneField(list->field);
		free(list);
		list = next;
	}
}

void destroyDroneDef(DroneDef * drone) {
	if (drone == NULL) return;
	if (drone->name != NULL) free(drone->name);
	destroyDroneFieldList(drone->fields);
	free(drone);
}

void destroyFleetComponent(FleetComponent * component) {
	if (component == NULL) return;
	if (component->droneTypeName != NULL) free(component->droneTypeName);
	free(component);
}

void destroyFleetComponentList(FleetComponentList * list) {
	while (list != NULL) {
		FleetComponentList * next = list->next;
		destroyFleetComponent(list->component);
		free(list);
		list = next;
	}
}

void destroyFleetDef(FleetDef * fleet) {
	if (fleet == NULL) return;
	if (fleet->name != NULL) free(fleet->name);
	destroyFleetComponentList(fleet->components);
	free(fleet);
}

void destroyZoneDef(ZoneDef * zone) {
	if (zone == NULL) return;
	if (zone->name != NULL) free(zone->name);
	if (zone->kind == ZONE_POLYGON) destroyPointList(zone->vertices);
	free(zone);
}

void destroyStationDef(StationDef * station) {
	if (station == NULL) return;
	if (station->name != NULL) free(station->name);
	free(station);
}

void destroyFormationShape(FormationShape * shape) {
	if (shape == NULL) return;
	free(shape);
}

void destroyFormationDef(FormationDef * formation) {
	if (formation == NULL) return;
	if (formation->name != NULL) free(formation->name);
	destroyFormationShape(formation->shape);
	free(formation);
}

void destroyFleetRef(FleetRef * ref) {
	if (ref == NULL) return;
	if (ref->fleetName != NULL) free(ref->fleetName);
	if (ref->subFleetTypeName != NULL) free(ref->subFleetTypeName);
	free(ref);
}

void destroyStatement(Statement * statement) {
	if (statement == NULL) return;
	destroyFleetRef(statement->fleet);
	if (statement->stationName != NULL) free(statement->stationName);
	destroyFormationDef(statement->formation);
	if (statement->zoneName != NULL) free(statement->zoneName);
	destroyStatementList(statement->body);
	free(statement);
}

void destroyStatementList(StatementList * list) {
	while (list != NULL) {
		StatementList * next = list->next;
		destroyStatement(list->statement);
		free(list);
		list = next;
	}
}

void destroyConstraint(Constraint * constraint) {
	if (constraint == NULL) return;
	if (constraint->zoneName != NULL) free(constraint->zoneName);
	free(constraint);
}

void destroyConstraintList(ConstraintList * list) {
	while (list != NULL) {
		ConstraintList * next = list->next;
		destroyConstraint(list->constraint);
		free(list);
		list = next;
	}
}

void destroyMissionDef(MissionDef * mission) {
	if (mission == NULL) return;
	if (mission->name != NULL) free(mission->name);
	destroyStatementList(mission->body);
	destroyConstraintList(mission->constraints);
	free(mission);
}

void destroyDeclaration(Declaration * declaration) {
	if (declaration == NULL) return;
	switch (declaration->kind) {
		case DECL_DRONE:     destroyDroneDef(declaration->drone); break;
		case DECL_FLEET:     destroyFleetDef(declaration->fleet); break;
		case DECL_ZONE:      destroyZoneDef(declaration->zone); break;
		case DECL_STATION:   destroyStationDef(declaration->station); break;
		case DECL_FORMATION: destroyFormationDef(declaration->formation); break;
		case DECL_MISSION:   destroyMissionDef(declaration->mission); break;
	}
	free(declaration);
}

void destroyDeclarationList(DeclarationList * list) {
	while (list != NULL) {
		DeclarationList * next = list->next;
		destroyDeclaration(list->declaration);
		free(list);
		list = next;
	}
}

void destroyProgram(Program * program) {
	if (program == NULL) return;
	destroyDeclarationList(program->declarations);
	free(program);
}
