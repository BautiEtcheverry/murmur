%top{

#include "../../support/type/CompilationStatus.h"
#include "../../support/type/FlexContext.h"
#include "../../support/type/TokenLabel.h"
#include "../syntactic-analysis/AbstractSyntaxTree.h"
#include "../syntactic-analysis/BisonParser.h"

}

%{

/**
 * @see https://westes.github.io/flex/manual/Definitions-Section.html
 */

extern CompilationStatus EOFLexemeAction(void);
extern CompilationStatus IgnoredLexemeAction(void);
extern CompilationStatus EnterMultilineCommentLexemeAction(FlexContext context);
extern CompilationStatus LeaveMultilineCommentLexemeAction(void);

extern CompilationStatus KeywordLexemeAction(TokenLabel label);
extern CompilationStatus PunctuationLexemeAction(TokenLabel label);
extern CompilationStatus IntegerLexemeAction(void);
extern CompilationStatus DecimalLexemeAction(void);
extern CompilationStatus IdentifierLexemeAction(void);
extern CompilationStatus UnitLexemeAction(UnitKind unit);
extern CompilationStatus UnknownLexemeAction(void);

%}

%x MULTILINE_COMMENT

whitespace      [[:space:]]+

%%

"/*"                                { return EnterMultilineCommentLexemeAction(MULTILINE_COMMENT); }
<MULTILINE_COMMENT>"*/"             { return LeaveMultilineCommentLexemeAction(); }
<MULTILINE_COMMENT>[^*]+            { return IgnoredLexemeAction(); }
<MULTILINE_COMMENT>.                { return IgnoredLexemeAction(); }

"//"[^\n]*                          { return IgnoredLexemeAction(); }

"[s]"                               { return UnitLexemeAction(UNIT_SECONDS); }
"[min]"                             { return UnitLexemeAction(UNIT_MINUTES); }
"[m]"                               { return UnitLexemeAction(UNIT_METERS); }
"[km]"                              { return UnitLexemeAction(UNIT_KILOMETERS); }
"[m/s]"                             { return UnitLexemeAction(UNIT_METERS_PER_SECOND); }
"[km/h]"                            { return UnitLexemeAction(UNIT_KILOMETERS_PER_HOUR); }
"[m/s^2]"                           { return UnitLexemeAction(UNIT_METERS_PER_SECOND_SQUARED); }
"[deg]"                             { return UnitLexemeAction(UNIT_DEGREES); }
"[rad]"                             { return UnitLexemeAction(UNIT_RADIANS); }
"[px]"                              { return UnitLexemeAction(UNIT_PIXELS); }
"[%]"                               { return UnitLexemeAction(UNIT_PERCENT); }

"drone"                             { return KeywordLexemeAction(DRONE); }
"fleet"                             { return KeywordLexemeAction(FLEET); }
"zone"                              { return KeywordLexemeAction(ZONE); }
"station"                           { return KeywordLexemeAction(STATION); }
"formation"                         { return KeywordLexemeAction(FORMATION); }
"mission"                           { return KeywordLexemeAction(MISSION); }
"constraints"                       { return KeywordLexemeAction(CONSTRAINTS); }
"parallel"                          { return KeywordLexemeAction(PARALLEL); }
"sequence"                          { return KeywordLexemeAction(SEQUENCE); }
"polygon"                           { return KeywordLexemeAction(POLYGON); }
"circle"                            { return KeywordLexemeAction(CIRCLE); }
"point"                             { return KeywordLexemeAction(POINT); }
"line"                              { return KeywordLexemeAction(LINE); }
"grid"                              { return KeywordLexemeAction(GRID); }
"radius"                            { return KeywordLexemeAction(RADIUS); }
"altitude"                          { return KeywordLexemeAction(ALTITUDE); }
"at"                                { return KeywordLexemeAction(AT); }
"of"                                { return KeywordLexemeAction(OF); }
"endurance"                         { return KeywordLexemeAction(ENDURANCE); }
"max_speed"                         { return KeywordLexemeAction(MAX_SPEED); }
"max_accel"                         { return KeywordLexemeAction(MAX_ACCEL); }
"payload"                           { return KeywordLexemeAction(PAYLOAD); }
"link_range"                        { return KeywordLexemeAction(LINK_RANGE); }
"resolution"                        { return KeywordLexemeAction(RESOLUTION); }
"fov"                               { return KeywordLexemeAction(FOV); }
"spacing"                           { return KeywordLexemeAction(SPACING); }
"window"                            { return KeywordLexemeAction(WINDOW); }
"takeoff"                           { return KeywordLexemeAction(TAKEOFF); }
"land"                              { return KeywordLexemeAction(LAND); }
"from"                              { return KeywordLexemeAction(FROM); }
"to"                                { return KeywordLexemeAction(TO); }
"morph"                             { return KeywordLexemeAction(MORPH); }
"sweep"                             { return KeywordLexemeAction(SWEEP); }
"orbit"                             { return KeywordLexemeAction(ORBIT); }
"hold"                              { return KeywordLexemeAction(HOLD); }
"return"                            { return KeywordLexemeAction(RETURN); }
"over"                              { return KeywordLexemeAction(OVER); }
"for"                               { return KeywordLexemeAction(FOR); }
"before"                            { return KeywordLexemeAction(BEFORE); }
"with"                              { return KeywordLexemeAction(WITH); }
"lawnmower"                         { return KeywordLexemeAction(LAWNMOWER); }
"spiral"                            { return KeywordLexemeAction(SPIRAL); }
"perimeter"                         { return KeywordLexemeAction(PERIMETER); }
"min_separation"                    { return KeywordLexemeAction(MIN_SEPARATION); }
"battery_reserve"                   { return KeywordLexemeAction(BATTERY_RESERVE); }
"exclude"                           { return KeywordLexemeAction(EXCLUDE); }
"require"                           { return KeywordLexemeAction(REQUIRE); }
"coverage"                          { return KeywordLexemeAction(COVERAGE); }
"all"                               { return KeywordLexemeAction(ALL_KW); }
"x"                                 { return KeywordLexemeAction(BY); }

".."                                { return PunctuationLexemeAction(DOTDOT); }
">="                                { return PunctuationLexemeAction(GE); }
"{"                                 { return PunctuationLexemeAction(OPEN_BRACE); }
"}"                                 { return PunctuationLexemeAction(CLOSE_BRACE); }
"("                                 { return PunctuationLexemeAction(OPEN_PARENTHESIS); }
")"                                 { return PunctuationLexemeAction(CLOSE_PARENTHESIS); }
"["                                 { return PunctuationLexemeAction(OPEN_BRACKET); }
"]"                                 { return PunctuationLexemeAction(CLOSE_BRACKET); }
","                                 { return PunctuationLexemeAction(COMMA); }
"."                                 { return PunctuationLexemeAction(DOT); }
";"                                 { return PunctuationLexemeAction(SEMICOLON); }
"="                                 { return PunctuationLexemeAction(EQUALS); }
"+"                                 { return PunctuationLexemeAction(PLUS); }

[0-9]+"."[0-9]+                     { return DecimalLexemeAction(); }
[0-9]+                              { return IntegerLexemeAction(); }

[a-zA-Z][a-zA-Z0-9_]*               { return IdentifierLexemeAction(); }

{whitespace}                        { return IgnoredLexemeAction(); }
.                                   { return UnknownLexemeAction(); }

<<EOF>>                             { return EOFLexemeAction(); }

%%

#include "FlexActions.h"
#include "FlexExport.h"
