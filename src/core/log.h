// Console logging in studiomdl's style so Crowbar's compile log reads naturally.
#pragma once

#include <cstdarg>
#include <stdexcept>
#include <string>

namespace r2c {

enum class LogLevel { Quiet, Normal, Verbose };

void SetLogLevel(LogLevel l);
LogLevel GetLogLevel();
void Info(const char* fmt, ...);
void Verbose(const char* fmt, ...);
void Warn(const char* fmt, ...);
int WarningCount();

// Thrown for any compile error; main() prints it as "ERROR: ..." and exits non-zero.
class CompileError : public std::runtime_error {
public:
    explicit CompileError(const std::string& m) : std::runtime_error(m) {}
};
[[noreturn]] void Fatal(const char* fmt, ...);

std::string Format(const char* fmt, ...);
std::string VFormat(const char* fmt, va_list ap);

}  // namespace r2c
