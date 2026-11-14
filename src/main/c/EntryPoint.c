#include "backend/code-generation/Generator.h"
#include "backend/domain/Model.h"
#include "backend/semantic-analysis/SemanticAnalyzer.h"
#include "backend/simulation/Simulation.h"
#include "frontend/Frontend.h"
#include "frontend/lexical-analysis/FlexActions.h"
#include "frontend/syntactic-analysis/BisonActions.h"
#include "support/logging/Logger.h"
#include "support/type/CompilationStatus.h"
#include "support/type/CompilerState.h"
#include "support/type/ModuleDestructor.h"

/**
 * The main entry-point of the entire application. If you use "strtok" to
 * parse anything inside this project instead of using Flex and Bison, I will
 * find you, and I will kill you (Bryan Mills; "Taken", 2008).
 *
 * Pipeline: lexical + syntactic analysis (AST) -> semantic analysis (symbol
 * tables, units, geometry, cardinality, parallelism) -> simulation and
 * trajectory verification (speed, separation, exclusion, link, energy,
 * coverage) -> code generation (JSON flight log).
 *
 * Usage: Murmur-Compiler [output.json] < program
 */
const int main(const int length, const char ** arguments) {
	LexicalAnalyzer * lexicalAnalyzer = createLexicalAnalyzer();
	Logger * logger = createLogger("EntryPoint");
	for (int k = 0; k < length; ++k) {
		logDebugging(logger, "Argument %d: \"%s\"", k, arguments[k]);
	}
	const char * outputPath = 1 < length ? arguments[1] : NULL;
	CompilerState compilerState = {
		.abstractSyntaxtTree = NULL,
		.value = 0
	};
	ModuleDestructor moduleDestructors[] = {
		initializeAbstractSyntaxTreeModule(),
		initializeFlexActionsModule(lexicalAnalyzer),
		initializeBisonActionsModule(&compilerState),
		initializeFrontendModule(lexicalAnalyzer),
		initializeSemanticAnalyzerModule(),
		initializeSimulationModule(),
		initializeGeneratorModule()
	};
	Model * model = createModel();
	CompilationStatus compilationStatus = executeSyntacticAnalysis();
	Program * program = compilerState.abstractSyntaxtTree;
	if (compilationStatus != SUCCEEDED) {
		logError(logger, "The syntactic-analysis phase rejects the input program.");
		compilationStatus = FAILED;
	}
	else if (executeSemanticAnalysis(program, model) != SUCCEEDED) {
		logError(logger, "The semantic-analysis phase rejects the input program.");
		compilationStatus = FAILED;
	}
	else if (executeSimulation(model) != SUCCEEDED) {
		logError(logger, "The mission verification phase rejects the input program.");
		compilationStatus = FAILED;
	}
	else if (executeCodeGeneration(model, outputPath) != SUCCEEDED) {
		logError(logger, "The code-generation phase failed.");
		compilationStatus = FAILED;
	}
	logDebugging(logger, "Releasing model and AST resources...");
	destroyModel(model);
	destroyProgram(program);
	for (int k = (sizeof(moduleDestructors)/sizeof(ModuleDestructor)) - 1; 0 <= k; --k) {
		moduleDestructors[k]();
	}
	logDebugging(logger, "Compilation is done.");
	destroyLogger(logger);
	destroyLexicalAnalyzer(lexicalAnalyzer);
	return compilationStatus;
}
