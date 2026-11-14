#include "Model.h"
#include <string.h>

Model * createModel(void) {
	return calloc(1, sizeof(Model));
}

void destroyModel(Model * model) {
	if (model == NULL) return;
	for (int k = 0; k < model->aircraftCount; ++k) {
		free(model->aircraft[k].frames);
	}
	free(model);
}

#define FIND_BY_NAME(array, count, name) \
	for (int k = 0; k < (count); ++k) { \
		if ((array)[k].name != NULL && strcmp((array)[k].name, (name)) == 0) return &(array)[k]; \
	} \
	return NULL;

DroneType * findDroneType(Model * model, const char * name) { FIND_BY_NAME(model->droneTypes, model->droneTypeCount, name) }
Fleet * findFleet(Model * model, const char * name) { FIND_BY_NAME(model->fleets, model->fleetCount, name) }
Zone * findZone(Model * model, const char * name) { FIND_BY_NAME(model->zones, model->zoneCount, name) }
Station * findStation(Model * model, const char * name) { FIND_BY_NAME(model->stations, model->stationCount, name) }
Formation * findFormation(Model * model, const char * name) { FIND_BY_NAME(model->formations, model->formationCount, name) }

const char * flightPhaseName(FlightPhase phase) {
	switch (phase) {
		case PHASE_GROUND: return "ground";
		case PHASE_CLIMB: return "climb";
		case PHASE_CRUISE: return "cruise";
		case PHASE_HOVER: return "hover";
		case PHASE_DESCENT: return "descent";
	}
	return "unknown";
}
