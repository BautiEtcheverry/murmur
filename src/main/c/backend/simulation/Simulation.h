#ifndef SIMULATION_HEADER
#define SIMULATION_HEADER

#include "../../frontend/syntactic-analysis/AbstractSyntaxTree.h"
#include "../../support/logging/Logger.h"
#include "../../support/type/CompilationStatus.h"
#include "../../support/type/ModuleDestructor.h"
#include "../domain/Model.h"

/** Kinematic and energy parameters of the simulation. */
#define SIM_CRUISE_ALTITUDE 30.0		// [m] above the station after takeoff.
#define SIM_CLIMB_RATE 3.0				// [m/s]
#define SIM_DESCENT_RATE 2.0			// [m/s]
#define SIM_CRUISE_FACTOR 0.8			// Fraction of max_speed used to transit and sweep.
#define SIM_ORBIT_FACTOR 0.5			// Fraction of max_speed used to orbit.
#define SIM_GROUND_SLOT 10.0			// [m] minimum distance between parked aircraft.
#define SIM_DETOUR_MARGIN 5.0			// [m] clearance kept around excluded zones.
#define SIM_SAMPLE_DT 0.5				// [s] sampling period for trajectory checks.
#define SIM_ENERGY_CLIMB 1.6			// Consumption factors per flight phase,
#define SIM_ENERGY_DESCENT 0.8			// relative to the nominal consumption
#define SIM_ENERGY_CRUISE 1.0			// (1 / endurance per second).
#define SIM_ENERGY_HOVER 1.2

/** Initialize module's internal state. */
ModuleDestructor initializeSimulationModule();

/**
 * Second, third and fourth verification levels: reconstructs the trajectory
 * of every aircraft (discrete temporal sampling), and checks the mission
 * window, kinematic limits (max_speed, max_accel), minimum separation,
 * excluded zones, link range, energy (endurance and battery reserve), and the
 * coverage requirements.
 */
CompilationStatus executeSimulation(Model * model);

/**
 * Samples the state of an aircraft at instant t. Returns false if the
 * aircraft was not deployed yet (before its first takeoff).
 */
bool sampleAircraft(const Aircraft * aircraft, double t, Vec3 * position, bool * airborne, FlightPhase * phase, double * battery);

#endif
