/* msg_filter: keep tool-call JSON off the speaker (E2 rule, SPEC §5/§6).
 * Server LLM may return a bare block {"tool": "...", "arguments": {...}};
 * reading that aloud is a shipped bug. Filter before TTS queue.
 * All entry points soft-fail on bad input (NULL/bad JSON -> safe default).
 */
#pragma once

#include <stdbool.h>

typedef enum {
    ACT_TTS_QUEUE, /* normal speech text -> TTS queue */
    ACT_DROP_TOOL, /* tool-call JSON -> never TTS */
    ACT_EMOTION,   /* llm emotion -> face/servo only */
    ACT_ABORT,     /* barge-in abort */
    ACT_SYSTEM,    /* hello/system/unknown -> system handler */
} msg_action_t;

/* True if text matches ^\s*\{"tool" and carries an "arguments" key. */
bool msg_filter_is_tool_json(const char *text);

/* Minimal dispatcher skeleton over (type, state, text). */
msg_action_t msg_filter_route(const char *type, const char *state, const char *text);

/* Built-in vectors incl. whitespace tool-JSON variants, normal VI text,
 * emotion, empty. Returns fail count (0 = pass). */
int msg_filter_selftest(void);
