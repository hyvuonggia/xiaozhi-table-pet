/* msg_filter implementation: E2 tool-JSON rule, SPEC §5/§6 anti-patterns.
 * No locks, no blocking, no abort-on-error — pure classification.
 */
#include <string.h>
#include <ctype.h>

#include "msg_filter.h"
#include "cJSON.h"

bool msg_filter_is_tool_json(const char *text)
{
    if (!text) {
        return false;
    }
    const char *p = text;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    static const char kToolOpen[] = "{\"tool\"";
    if (strncmp(p, kToolOpen, sizeof(kToolOpen) - 1) != 0) {
        return false;
    }
    /* E2 shape is {"tool": ..., "arguments": {...}} — require arguments. */
    return strstr(p, "\"arguments\"") != NULL;
}

msg_action_t msg_filter_route(const char *type, const char *state, const char *text)
{
    (void)state;
    if (!type) {
        return ACT_SYSTEM; /* soft-fail: unknown -> system */
    }
    if (text && msg_filter_is_tool_json(text)) {
        return ACT_DROP_TOOL; /* never TTS tool JSON, whatever the type */
    }
    if (strcmp(type, "llm") == 0 || strcmp(type, "emotion") == 0) {
        return ACT_EMOTION;
    }
    if (strcmp(type, "abort") == 0) {
        return ACT_ABORT;
    }
    if (strcmp(type, "tts") == 0 || strcmp(type, "stt") == 0) {
        return ACT_TTS_QUEUE;
    }
    return ACT_SYSTEM;
}

int msg_filter_selftest(void)
{
    int fails = 0;
#define VEC(cond) do { if (!(cond)) { fails++; } } while (0)
    /* 1: exact tool JSON -> drop */
    VEC(msg_filter_is_tool_json("{\"tool\": \"get_now\", \"arguments\": {}}"));
    /* 2: leading whitespace/newlines -> still drop */
    VEC(msg_filter_is_tool_json("  \n\t {\"tool\":\"get_weather_vn\", \"arguments\": {\"city\": \"Ha Noi\"}}"));
    /* 3: tool JSON via route() -> ACT_DROP_TOOL even with tts type */
    VEC(msg_filter_route("tts", "sentence_start",
                         " {\"tool\": \"web_search_vn\", \"arguments\": {\"q\": \"gia vang\"}}")
        == ACT_DROP_TOOL);
    /* 4: normal VI text -> not tool, routes to TTS */
    VEC(!msg_filter_is_tool_json("Xin chào, hôm nay trời đẹp."));
    VEC(msg_filter_route("tts", "sentence_start", "Xin chào.") == ACT_TTS_QUEUE);
    /* 5: emotion/llm -> ACT_EMOTION */
    VEC(msg_filter_route("llm", NULL, "{\"emotion\": \"happy\"}") == ACT_EMOTION);
    /* 6: abort -> ACT_ABORT */
    VEC(msg_filter_route("abort", "stop", NULL) == ACT_ABORT);
    /* 7: empty text -> not tool */
    VEC(!msg_filter_is_tool_json(""));
    /* 8: NULL-safe soft-fail */
    VEC(!msg_filter_is_tool_json(NULL));
    VEC(msg_filter_route(NULL, NULL, NULL) == ACT_SYSTEM);
    /* 9: {"tool" without arguments -> not a tool call, keep for TTS path */
    VEC(!msg_filter_is_tool_json("{\"tool\": \"get_now\"}"));
#undef VEC
    return fails;
}
