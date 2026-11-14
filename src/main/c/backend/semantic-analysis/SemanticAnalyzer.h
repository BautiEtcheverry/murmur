#ifndef SEMANTIC_ANALYZER_HEADER
#define SEMANTIC_ANALYZER_HEADER

#include "../../frontend/syntactic-analysis/AbstractSyntaxTree.h"
#include "../../support/logging/Logger.h"
#include "../../support/type/CompilationStatus.h"
#include "../../support/type/ModuleDestructor.h"
#include "../domain/Model.h"

/**
 * A set of aircraft, produced by resolving a fleet reference (`all`,
 * `Bravo`, or `Bravo.Scout`).
 */
typedef struct Selection {
	int indexes[MAX_AIRCRAFT];
	int count;
} Selection;

/** Initialize module's internal state. */
ModuleDestructor initializeSemanticAnalyzerModule();

/**
 * First verification level ("over the AST"): builds the symbol tables of the
 * model, checks that every identifier is declared before its use and that it
 * denotes the right kind of entity, checks the dimensional correctness of
 * every magnitude, the geometric validity of zones, formation cardinality,
 * the mission window and the absence of simultaneous maneuvers over the same
 * aircraft inside a parallel block.
 */
CompilationStatus executeSemanticAnalysis(Program * program, Model * model);

/** Converts a (previously validated) quantity to SI units. */
double quantityToSI(Quantity quantity);

/** Resolves a (previously validated) fleet reference. */
bool resolveFleetRef(Model * model, const FleetRef * ref, Selection * selection);

/** Writes a printable version of a fleet reference into the buffer. */
const char * fleetRefText(const FleetRef * ref, char * buffer, size_t size);

#endif
