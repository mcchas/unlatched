/*
 * Building JSON documents, for the two things that emit them: the HTTP API and
 * the MQTT bridge.
 *
 * This is deliberately only the WRITING half. Nothing here parses -- the web API
 * takes form-encoded parameters, which WebServer decodes for us, and the MQTT
 * bridge hands inbound payloads to ArduinoJson. A hand-rolled parser for input
 * arriving from a broker is a class of bug worth paying a library to avoid;
 * writing is the other way round, being a few hundred bytes of appends against
 * a buffer we control.
 *
 * Extracted from web_ui.cpp when the bridge needed the same thing. The escaping
 * in particular is why: it existed once, correctly, and a second copy would have
 * been the one that forgot a case the first had learned -- room names are
 * user-typed, and "Kids' room" or a name with a stray quote in it turns an
 * unescaped document into one the far end cannot parse. What that looks like
 * from a phone is the rig having gone offline.
 *
 * Documents are assembled with += and these appenders rather than with
 * expressions like "key" + String(n): Arduino's String only defines operator+
 * with a String on the left, so that idiom does not compile here at all.
 */

#ifndef JSON_OUT_H
#define JSON_OUT_H

#include <Arduino.h>
#include <cstdint>
#include <cstdio>

/// JSON string escaping, including the control characters below 0x20 that a
/// bare append would emit raw and make the document invalid.
inline void jsonEscaped(String &out, const char *s) {
    for (const char *p = s; p && *p; p++) {
        switch (*p) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<uint8_t>(*p) < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "\\u%04X", static_cast<unsigned>(*p) & 0xFF);
                    out += buf;
                } else {
                    out += *p;
                }
        }
    }
}

/// `"key":"value"`, escaped.
inline void kvStr(String &out, const char *key, const char *value) {
    out += '"';
    out += key;
    out += "\":\"";
    jsonEscaped(out, value);
    out += '"';
}

/// `"key":123`.
inline void kvNum(String &out, const char *key, const long value) {
    out += '"';
    out += key;
    out += "\":";
    out += value;
}

/// `"key":true`.
inline void kvBool(String &out, const char *key, const bool value) {
    out += '"';
    out += key;
    out += "\":";
    out += value ? "true" : "false";
}

/// `"key":"6537B4"` -- a node address, the form every other interface prints.
inline void kvNode(String &out, const char *key, const uint8_t n[3]) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%02X%02X%02X", n[0], n[1], n[2]);
    kvStr(out, key, buf);
}

/// `"6537B4"` as a bare array element.
inline void jsonNode(String &out, const uint8_t n[3]) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%02X%02X%02X", n[0], n[1], n[2]);
    out += '"';
    out += buf;
    out += '"';
}

#endif  // JSON_OUT_H
