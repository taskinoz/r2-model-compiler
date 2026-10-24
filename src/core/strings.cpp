#include "core/strings.h"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace r2c {

std::string ToLower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool IEquals(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++)
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) return false;
    return true;
}

std::string Trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) a++;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) b--;
    return s.substr(a, b - a);
}

std::string StripExtension(const std::string& s) {
    size_t dot = s.find_last_of('.');
    size_t sep = s.find_last_of("/\\");
    if (dot == std::string::npos || (sep != std::string::npos && dot < sep)) return s;
    return s.substr(0, dot);
}

std::string GetExtension(const std::string& s) {
    size_t dot = s.find_last_of('.');
    size_t sep = s.find_last_of("/\\");
    if (dot == std::string::npos || (sep != std::string::npos && dot < sep)) return "";
    return s.substr(dot + 1);
}

std::string FileBase(const std::string& s) {
    size_t sep = s.find_last_of("/\\");
    std::string f = sep == std::string::npos ? s : s.substr(sep + 1);
    return StripExtension(f);
}

std::string DirName(const std::string& s) {
    size_t sep = s.find_last_of("/\\");
    if (sep == std::string::npos) return "";
    return s.substr(0, sep + 1);
}

std::string JoinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (b.empty()) return a;
    // absolute b wins
    if (b[0] == '/' || b[0] == '\\' || (b.size() > 1 && b[1] == ':')) return b;
    char last = a.back();
    if (last == '/' || last == '\\') return a + b;
    return a + "/" + b;
}

std::string FixSlashes(std::string s, char sep) {
    for (auto& c : s)
        if (c == '/' || c == '\\') c = sep;
    return s;
}

bool FileExists(const std::string& p) {
    std::error_code ec;
    return std::filesystem::is_regular_file(std::filesystem::u8path(p), ec);
}

bool StartsWithI(const std::string& s, const std::string& prefix) {
    return s.size() >= prefix.size() && IEquals(s.substr(0, prefix.size()), prefix);
}

bool EndsWithI(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && IEquals(s.substr(s.size() - suffix.size()), suffix);
}

std::vector<std::string> Split(const std::string& s, char c) {
    std::vector<std::string> out;
    size_t start = 0;
    for (size_t i = 0; i <= s.size(); i++) {
        if (i == s.size() || s[i] == c) {
            out.push_back(s.substr(start, i - start));
            start = i + 1;
        }
    }
    return out;
}

}  // namespace r2c
