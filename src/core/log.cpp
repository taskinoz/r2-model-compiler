#include "core/log.h"

#include <cstdio>
#include <vector>

namespace r2c {

static LogLevel g_level = LogLevel::Normal;
static int g_warnings = 0;

void SetLogLevel(LogLevel l) { g_level = l; }
LogLevel GetLogLevel() { return g_level; }

std::string VFormat(const char* fmt, va_list ap) {
    va_list cp;
    va_copy(cp, ap);
    int n = std::vsnprintf(nullptr, 0, fmt, cp);
    va_end(cp);
    if (n < 0) return fmt;
    std::vector<char> buf(static_cast<size_t>(n) + 1);
    std::vsnprintf(buf.data(), buf.size(), fmt, ap);
    return std::string(buf.data(), static_cast<size_t>(n));
}

std::string Format(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::string s = VFormat(fmt, ap);
    va_end(ap);
    return s;
}

void Info(const char* fmt, ...) {
    if (g_level == LogLevel::Quiet) return;
    va_list ap;
    va_start(ap, fmt);
    std::string s = VFormat(fmt, ap);
    va_end(ap);
    std::fputs(s.c_str(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
}

void Verbose(const char* fmt, ...) {
    if (g_level != LogLevel::Verbose) return;
    va_list ap;
    va_start(ap, fmt);
    std::string s = VFormat(fmt, ap);
    va_end(ap);
    std::fputs(s.c_str(), stdout);
    std::fputc('\n', stdout);
}

void Warn(const char* fmt, ...) {
    g_warnings++;
    va_list ap;
    va_start(ap, fmt);
    std::string s = VFormat(fmt, ap);
    va_end(ap);
    std::fprintf(stdout, "WARNING: %s\n", s.c_str());
    std::fflush(stdout);
}

int WarningCount() { return g_warnings; }

void Fatal(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::string s = VFormat(fmt, ap);
    va_end(ap);
    throw CompileError(s);
}

}  // namespace r2c
