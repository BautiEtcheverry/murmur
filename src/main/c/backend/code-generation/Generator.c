#include "Generator.h"
#include "../simulation/Simulation.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

/* MODULE INTERNAL STATE */

static Logger * _logger = NULL;

void _shutdownGeneratorModule() {
	if (_logger != NULL) {
		logDebugging(_logger, "Destroying module: Generator...");
		destroyLogger(_logger);
		_logger = NULL;
	}
}

ModuleDestructor initializeGeneratorModule() {
	_logger = createLogger("Generator");
	return _shutdownGeneratorModule;
}

/* HELPERS */

/** Rounds to millimeters/milliseconds and avoids "-0.000". */
static double _r(double value) {
	const double rounded = round(value * 1000.0) / 1000.0;
	return rounded == 0 ? 0 : rounded;
}

static void _optional(FILE * out, bool present, double value) {
	if (present) fprintf(out, "%.10g", _r(value));
	else fprintf(out, "null");
}

static void _emitDroneTypes(FILE * out, Model * model) {
	fprintf(out, "  \"droneTypes\": [");
	for (int k = 0; k < model->droneTypeCount; ++k) {
		const DroneType * type = &model->droneTypes[k];
		fprintf(out, "%s\n    {\"name\": \"%s\", \"endurance\": %.10g, \"maxSpeed\": %.10g, \"maxAccel\": ",
			k == 0 ? "" : ",", type->name, _r(type->endurance), _r(type->maxSpeed));
		_optional(out, type->hasMaxAccel, type->maxAccel);
		fprintf(out, ", \"linkRange\": ");
		_optional(out, type->hasLinkRange, type->linkRange);
		if (type->hasPayload) {
			fprintf(out, ", \"payload\": {\"sensor\": \"%s\", \"resolution\": %.10g, \"fov\": %.10g}}",
				type->sensor, _r(type->resolution), _r(type->fov));
		}
		else {
			fprintf(out, ", \"payload\": null}");
		}
	}
	fprintf(out, "\n  ],\n");
}

static void _emitFleets(FILE * out, Model * model) {
	fprintf(out, "  \"fleets\": [");
	for (int f = 0; f < model->fleetCount; ++f) {
		const Fleet * fleet = &model->fleets[f];
		fprintf(out, "%s\n    {\"name\": \"%s\", \"aircraft\": [", f == 0 ? "" : ",", fleet->name);
		for (int k = 0; k < fleet->count; ++k) {
			fprintf(out, "%s\"%s\"", k == 0 ? "" : ", ", model->aircraft[fleet->first + k].id);
		}
		fprintf(out, "]}");
	}
	fprintf(out, "\n  ],\n");
}

static void _emitZones(FILE * out, Model * model) {
	fprintf(out, "  \"zones\": [");
	for (int z = 0; z < model->zoneCount; ++z) {
		const Zone * zone = &model->zones[z];
		fprintf(out, "%s\n    {\"name\": \"%s\", \"kind\": \"%s\", \"center\": [%.10g, %.10g], \"radius\": ",
			z == 0 ? "" : ",", zone->name, zone->isCircle ? "circle" : "polygon", _r(zone->center.x), _r(zone->center.y));
		_optional(out, zone->isCircle, zone->radius);
		fprintf(out, ", \"vertices\": [");
		for (int k = 0; k < zone->vertexCount; ++k) {
			fprintf(out, "%s[%.10g, %.10g]", k == 0 ? "" : ", ", _r(zone->vertices[k].x), _r(zone->vertices[k].y));
		}
		fprintf(out, "], \"excluded\": %s, \"swept\": %s, \"coverage\": ", zone->excluded ? "true" : "false", zone->swept ? "true" : "false");
		_optional(out, zone->swept, zone->coverage);
		fprintf(out, "}");
	}
	fprintf(out, "\n  ],\n");
}

static void _emitStations(FILE * out, Model * model) {
	fprintf(out, "  \"stations\": [");
	for (int s = 0; s < model->stationCount; ++s) {
		const Station * station = &model->stations[s];
		fprintf(out, "%s\n    {\"name\": \"%s\", \"position\": [%.10g, %.10g], \"altitude\": %.10g}",
			s == 0 ? "" : ",", station->name, _r(station->position.x), _r(station->position.y), _r(station->altitude));
	}
	fprintf(out, "\n  ],\n");
}

static void _emitMission(FILE * out, Model * model) {
	if (model->mission == NULL) {
		fprintf(out, "  \"mission\": null,\n  \"constraints\": null,\n  \"phases\": [],\n");
		return;
	}
	fprintf(out, "  \"mission\": {\"name\": \"%s\", \"window\": [%.10g, %.10g], \"end\": %.10g},\n",
		model->mission->name, _r(model->windowStart), _r(model->windowEnd), _r(model->missionEnd));
	fprintf(out, "  \"constraints\": {\"minSeparation\": ");
	_optional(out, model->hasMinSeparation, model->minSeparation);
	fprintf(out, ", \"batteryReserve\": ");
	_optional(out, model->hasBatteryReserve, model->batteryReserve);
	fprintf(out, ", \"coverage\": [");
	for (int k = 0; k < model->coverageCount; ++k) {
		fprintf(out, "%s{\"zone\": \"%s\", \"required\": %.10g, \"achieved\": %.10g}", k == 0 ? "" : ", ",
			model->coverage[k].zone->name, _r(model->coverage[k].threshold), _r(model->coverage[k].zone->coverage));
	}
	fprintf(out, "]},\n");
	fprintf(out, "  \"phases\": [");
	for (int k = 0; k < model->phaseCount; ++k) {
		const PhaseRecord * phase = &model->phases[k];
		fprintf(out, "%s\n    {\"label\": \"%s\", \"maneuver\": \"%s\", \"start\": %.10g, \"end\": %.10g}",
			k == 0 ? "" : ",", phase->label, phase->maneuver, _r(phase->start), _r(phase->end));
	}
	fprintf(out, "\n  ],\n");
}

static void _emitAircraft(FILE * out, Model * model) {
	fprintf(out, "  \"aircraft\": [");
	for (int k = 0; k < model->aircraftCount; ++k) {
		const Aircraft * aircraft = &model->aircraft[k];
		fprintf(out, "%s\n    {\"id\": \"%s\", \"type\": \"%s\", \"fleet\": \"%s\", \"deployed\": %s, \"flightTime\": %.10g, \"distance\": %.10g, \"batteryAtEnd\": %.10g}",
			k == 0 ? "" : ",", aircraft->id, aircraft->type->name, model->fleets[aircraft->fleet].name,
			aircraft->deployed ? "true" : "false", _r(aircraft->flightTime), _r(aircraft->distance), _r(aircraft->batteryAtEnd));
	}
	fprintf(out, "\n  ],\n");
}

static void _emitFrame(FILE * out, Model * model, double t, bool first) {
	fprintf(out, "%s\n      {\"t\": %.10g, \"drones\": [", first ? "" : ",", _r(t));
	bool firstDrone = true;
	for (int k = 0; k < model->aircraftCount; ++k) {
		Vec3 p;
		bool airborne;
		FlightPhase phase;
		double battery;
		if (!sampleAircraft(&model->aircraft[k], t, &p, &airborne, &phase, &battery)) continue;
		fprintf(out, "%s{\"id\": \"%s\", \"x\": %.10g, \"y\": %.10g, \"z\": %.10g, \"phase\": \"%s\", \"battery\": %.10g}",
			firstDrone ? "" : ", ", model->aircraft[k].id, _r(p.x), _r(p.y), _r(p.z), flightPhaseName(phase), _r(battery));
		firstDrone = false;
	}
	fprintf(out, "]}");
}

static void _emitTimeline(FILE * out, Model * model) {
	fprintf(out, "  \"timeline\": {\"dt\": %.10g, \"frames\": [", GENERATOR_FRAME_DT);
	if (model->mission != NULL) {
		bool first = true;
		double t = model->windowStart;
		for (; t < model->missionEnd; t += GENERATOR_FRAME_DT) {
			_emitFrame(out, model, t, first);
			first = false;
		}
		_emitFrame(out, model, model->missionEnd, first);
	}
	fprintf(out, "\n  ]}\n");
}

/* PUBLIC FUNCTIONS */

CompilationStatus executeCodeGeneration(Model * model, const char * outputPath) {
	char defaultPath[512];
	if (outputPath == NULL) {
		if (mkdir("out", 0755) != 0 && errno != EEXIST) {
			logError(_logger, "No se pudo crear el directorio de salida 'out': %s.", strerror(errno));
			return FAILED;
		}
		snprintf(defaultPath, sizeof(defaultPath), "out/%s.json", model->mission != NULL ? model->mission->name : "program");
		outputPath = defaultPath;
	}
	FILE * out = fopen(outputPath, "w");
	if (out == NULL) {
		logError(_logger, "No se pudo abrir el archivo de salida '%s': %s.", outputPath, strerror(errno));
		return FAILED;
	}
	fprintf(out, "{\n  \"format\": \"murmur-flight-log\",\n  \"version\": 1,\n");
	_emitMission(out, model);
	_emitDroneTypes(out, model);
	_emitFleets(out, model);
	_emitZones(out, model);
	_emitStations(out, model);
	_emitAircraft(out, model);
	_emitTimeline(out, model);
	fprintf(out, "}\n");
	fclose(out);
	logInformation(_logger, "Plan de vuelo generado en '%s'.", outputPath);
	return SUCCEEDED;
}
