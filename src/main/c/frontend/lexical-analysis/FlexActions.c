#include "FlexActions.h"
#include <stdlib.h>
#include <string.h>

/* MODULE INTERNAL STATE */

static bool _logIgnoredLexemes = true;
static LexicalAnalyzer * _lexicalAnalyzer = NULL;
static Logger * _logger = NULL;

void _shutdownFlexActionsModule() {
	if (_logger != NULL) {
		logDebugging(_logger, "Destroying module: FlexActions...");
		destroyLogger(_logger);
		_logger = NULL;
	}
	_lexicalAnalyzer = NULL;
}

ModuleDestructor initializeFlexActionsModule(LexicalAnalyzer * lexicalAnalyzer) {
	_lexicalAnalyzer = lexicalAnalyzer;
	_logger = createLogger("FlexActions");
	_logIgnoredLexemes = getBooleanOrDefault("LOG_IGNORED_LEXEMES", _logIgnoredLexemes);
	return _shutdownFlexActionsModule;
}

/* PRIVATE HELPERS */

static void _logTokenAction(const char * actionName, Token * token) {
	char * _lexeme = escape(token->lexeme);
	logDebugging(_logger, WARNING_COLOR "%s" DEFAULT_COLOR ": Token(context=%d, label=%d, length=%d, lexeme=%s\"%s\"%s, line=%d, semanticValue=%p)",
		actionName,
		token->context,
		token->label,
		token->length,
		INFORMATION_COLOR, _lexeme, DEFAULT_COLOR,
		token->line,
		token->semanticValue);
	free(_lexeme);
}

static CompilationStatus _pushAndDestroy(Token * token) {
	CompilationStatus status = pushToken(_lexicalAnalyzer, token);
	destroyToken(token);
	return status;
}

/* PUBLIC FUNCTIONS */

CompilationStatus EOFLexemeAction(void) {
	CompilationStatus status = IN_PROGRESS;
	Token * token = createToken(_lexicalAnalyzer, 0);
	_logTokenAction(__FUNCTION__, token);
	if (!popInputBuffer(_lexicalAnalyzer)) {
		status = pushToken(_lexicalAnalyzer, token);
		FlexContext context = currentLexicalAnalyzerContext(_lexicalAnalyzer);
		if (0 < context) {
			logError(_logger, "The final context is not closed (context=%d).", context);
			status = FAILED;
		}
	}
	destroyToken(token);
	return status;
}

CompilationStatus IgnoredLexemeAction(void) {
	if (_logIgnoredLexemes) {
		Token * token = createToken(_lexicalAnalyzer, IGNORED);
		_logTokenAction(__FUNCTION__, token);
		destroyToken(token);
	}
	return IN_PROGRESS;
}

CompilationStatus EnterMultilineCommentLexemeAction(FlexContext context) {
	if (_logIgnoredLexemes) {
		Token * token = createToken(_lexicalAnalyzer, IGNORED);
		_logTokenAction(__FUNCTION__, token);
		destroyToken(token);
	}
	enterLexicalAnalyzerContext(_lexicalAnalyzer, context);
	return IN_PROGRESS;
}

CompilationStatus LeaveMultilineCommentLexemeAction(void) {
	leaveLexicalAnalyzerContext(_lexicalAnalyzer);
	if (_logIgnoredLexemes) {
		Token * token = createToken(_lexicalAnalyzer, IGNORED);
		_logTokenAction(__FUNCTION__, token);
		destroyToken(token);
	}
	return IN_PROGRESS;
}

CompilationStatus KeywordLexemeAction(TokenLabel label) {
	Token * token = createToken(_lexicalAnalyzer, label);
	_logTokenAction(__FUNCTION__, token);
	return _pushAndDestroy(token);
}

CompilationStatus PunctuationLexemeAction(TokenLabel label) {
	Token * token = createToken(_lexicalAnalyzer, label);
	_logTokenAction(__FUNCTION__, token);
	return _pushAndDestroy(token);
}

CompilationStatus IntegerLexemeAction(void) {
	Token * token = createToken(_lexicalAnalyzer, INTEGER);
	token->semanticValue->integer = atol(token->lexeme);
	_logTokenAction(__FUNCTION__, token);
	return _pushAndDestroy(token);
}

CompilationStatus DecimalLexemeAction(void) {
	Token * token = createToken(_lexicalAnalyzer, DECIMAL);
	token->semanticValue->decimal = atof(token->lexeme);
	_logTokenAction(__FUNCTION__, token);
	return _pushAndDestroy(token);
}

CompilationStatus IdentifierLexemeAction(void) {
	Token * token = createToken(_lexicalAnalyzer, IDENTIFIER);
	char * copy = (char *) calloc(token->length + 1, sizeof(char));
	memcpy(copy, token->lexeme, token->length);
	token->semanticValue->string = copy;
	_logTokenAction(__FUNCTION__, token);
	return _pushAndDestroy(token);
}

CompilationStatus UnitLexemeAction(TokenLabel label, UnitKind unit) {
	Token * token = createToken(_lexicalAnalyzer, label);
	token->semanticValue->unit = unit;
	_logTokenAction(__FUNCTION__, token);
	return _pushAndDestroy(token);
}

CompilationStatus UnknownLexemeAction(void) {
	Token * token = createToken(_lexicalAnalyzer, UNKNOWN);
	_logTokenAction(__FUNCTION__, token);
	char * _lexeme = escape(token->lexeme);
	logError(_logger, "Line %d: unknown character \"%s\".", token->line, _lexeme);
	free(_lexeme);
	/**
	 * No rule expects an UNKNOWN token, so pushing it aborts the parser,
	 * which releases the AST fragments that are still on its stack.
	 */
	_pushAndDestroy(token);
	return FAILED;
}
