#ifndef GENERATOR_HEADER
#define GENERATOR_HEADER

#include "../../support/logging/Logger.h"
#include "../../support/type/CompilationStatus.h"
#include "../../support/type/ModuleDestructor.h"
#include "../domain/Model.h"

/** Sampling period of the emitted flight log [s]. */
#define GENERATOR_FRAME_DT 1.0

/** Initialize module's internal state. */
ModuleDestructor initializeGeneratorModule();

/**
 * Emits the flight log (JSON) of a verified mission: the declarations
 * resolved to SI units, the phases of the mission, the energy and coverage
 * report, and the position of every aircraft at each sampled instant. The
 * visualizer (src/main/visualizer/index.html) consumes this file.
 *
 * If outputPath is NULL, the log is written to "out/<mission>.json".
 */
CompilationStatus executeCodeGeneration(Model * model, const char * outputPath);

#endif
