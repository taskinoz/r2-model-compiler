// Small string and path helpers.
#pragma once

#include <string>
#include <vector>

namespace r2c {

std::string ToLower(std::string s);
bool IEquals(const std::string& a, const std::string& b);
std::string Trim(const std::string& s);
std::string StripExtension(const std::string& s);
std::string GetExtension(const std::string& s);  // without the dot
std::string FileBase(const std::string& s);      // file name without directory or extension
std::string DirName(const std::string& s);       // directory with trailing separator, or ""
std::string JoinPath(const std::string& a, const std::string& b);
std::string FixSlashes(std::string s, char sep = '/');
bool FileExists(const std::string& p);
bool StartsWithI(const std::string& s, const std::string& prefix);
bool EndsWithI(const std::string& s, const std::string& suffix);
std::vector<std::string> Split(const std::string& s, char c);

}  // namespace r2c
