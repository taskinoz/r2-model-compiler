#include "qc/tokenizer.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "core/log.h"
#include "core/strings.h"

namespace r2c {

std::vector<Token> Tokenizer::TokenizeText(const std::string& text, const std::string& file) {
    std::vector<Token> out;
    int line = 1;
    bool newline = true;
    size_t i = 0, n = text.size();
    while (i < n) {
        char c = text[i];
        if (c == '\n') {
            line++;
            newline = true;
            i++;
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c))) {
            i++;
            continue;
        }
        // comments
        if (c == '/' && i + 1 < n && text[i + 1] == '/') {
            while (i < n && text[i] != '\n') i++;
            continue;
        }
        if (c == '#' || c == ';') {
            // studiomdl treats lines starting with these as comments
            while (i < n && text[i] != '\n') i++;
            continue;
        }
        if (c == '/' && i + 1 < n && text[i + 1] == '*') {
            i += 2;
            while (i + 1 < n && !(text[i] == '*' && text[i + 1] == '/')) {
                if (text[i] == '\n') {
                    line++;
                    newline = true;
                }
                i++;
            }
            i += 2;
            continue;
        }
        Token t;
        t.file = file;
        t.line = line;
        t.newlineBefore = newline;
        newline = false;
        if (c == '"') {
            // QC paths use literal backslashes, including a trailing backslash
            // before the closing quote. Decode escapes only in event options,
            // where retail strings can contain literal quote characters.
            bool eventOptions = out.size() >= 3 && IEquals(out[out.size() - 3].text, "event");
            i++;
            while (i < n && text[i] != '"') {
                if (eventOptions && text[i] == '\\' && i + 1 < n && (text[i + 1] == '"' || text[i + 1] == '\\')) {
                    t.text += text[i + 1];
                    i += 2;
                    continue;
                }
                if (text[i] == '\n') line++;
                t.text += text[i++];
            }
            if (i == n) Fatal("%s(%d): unterminated quoted string", file.c_str(), t.line);
            t.quoted = true;
            i++;
        } else if (c == '{' || c == '}') {
            t.text = std::string(1, c);
            i++;
        } else {
            size_t s = i;
            while (i < n && !std::isspace(static_cast<unsigned char>(text[i])) && text[i] != '"' && text[i] != '{' &&
                   text[i] != '}') {
                if (text[i] == '/' && i + 1 < n && text[i + 1] == '/') break;
                i++;
            }
            t.text = text.substr(s, i - s);
        }
        out.push_back(t);
    }
    return out;
}

static std::string ReadAll(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) Fatal("could not open \"%s\"", path.c_str());
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

void Tokenizer::LoadFile(const std::string& path) {
    toks_ = TokenizeText(ReadAll(path), path);
    pos_ = 0;
}

void Tokenizer::IncludeFile(const std::string& path) { InsertTokens(TokenizeText(ReadAll(path), path)); }

void Tokenizer::InsertTokens(const std::vector<Token>& toks) {
    if (toks.empty()) return;
    std::vector<Token> copy = toks;
    copy[0].newlineBefore = true;
    toks_.insert(toks_.begin() + static_cast<std::ptrdiff_t>(pos_), copy.begin(), copy.end());
    // the token after the inserted run starts a new command line
    size_t after = pos_ + copy.size();
    if (after < toks_.size()) toks_[after].newlineBefore = true;
}

std::string Tokenizer::Substitute(const std::string& s) const {
    if (s.find('$') == std::string::npos || vars_.empty()) return s;
    std::string out;
    size_t i = 0;
    while (i < s.size()) {
        if (s[i] == '$') {
            size_t e = s.find('$', i + 1);
            if (e != std::string::npos && e > i + 1) {
                std::string name = s.substr(i + 1, e - i - 1);
                auto it = vars_.find(name);
                if (it != vars_.end()) {
                    out += it->second;
                    i = e + 1;
                    continue;
                }
            }
        }
        out += s[i++];
    }
    return out;
}

bool Tokenizer::Get(bool crossLine) {
    if (pos_ >= toks_.size()) return false;
    if (!crossLine && toks_[pos_].newlineBefore) {
        Fatal("%s: line is incomplete", Where().c_str());
    }
    cur_ = toks_[pos_++];
    cur_.text = Substitute(cur_.text);
    return true;
}

bool Tokenizer::Available() const { return pos_ < toks_.size() && !toks_[pos_].newlineBefore; }

void Tokenizer::Unget() {
    if (pos_ > 0) pos_--;
}

std::string Tokenizer::Where() const { return Format("%s(%d)", cur_.file.c_str(), cur_.line); }

std::string Tokenizer::Need(const char* what) {
    if (!Available()) Fatal("%s: expected %s", Where().c_str(), what);
    Get(false);
    return cur_.text;
}

std::string Tokenizer::NeedAny(const char* what) {
    if (!Get(true)) Fatal("%s: unexpected end of file, expected %s", Where().c_str(), what);
    return cur_.text;
}

float Tokenizer::NeedFloat(const char* what) {
    std::string t = Need(what);
    char* end = nullptr;
    float v = std::strtof(t.c_str(), &end);
    if (end == t.c_str()) Fatal("%s: expected a number for %s, got \"%s\"", Where().c_str(), what, t.c_str());
    return v;
}

int Tokenizer::NeedInt(const char* what) {
    std::string t = Need(what);
    char* end = nullptr;
    long v = std::strtol(t.c_str(), &end, 10);
    if (end == t.c_str()) {
        float f = std::strtof(t.c_str(), &end);
        if (end == t.c_str()) Fatal("%s: expected an integer for %s, got \"%s\"", Where().c_str(), what, t.c_str());
        return static_cast<int>(f);
    }
    return static_cast<int>(v);
}

void Tokenizer::SkipRestOfCommand() {
    int depth = 0;
    while (true) {
        if (depth == 0) {
            if (!Available()) {
                // a block may open on the next line
                if (pos_ < toks_.size() && toks_[pos_].text == "{" && !toks_[pos_].quoted) {
                    Get(true);
                    depth++;
                    continue;
                }
                return;
            }
            Get(false);
        } else {
            if (!Get(true)) return;
        }
        if (cur_.quoted) continue;
        if (cur_.text == "{") depth++;
        if (cur_.text == "}") {
            depth--;
            if (depth <= 0) return;
        }
    }
}

std::vector<Token> Tokenizer::ReadBlock() {
    std::vector<Token> out;
    if (!Get(true) || cur_.text != "{") Fatal("%s: expected '{'", Where().c_str());
    int depth = 1;
    while (Get(true)) {
        if (!cur_.quoted) {
            if (cur_.text == "{") depth++;
            if (cur_.text == "}") {
                depth--;
                if (depth == 0) return out;
            }
        }
        out.push_back(cur_);
    }
    Fatal("%s: missing '}'", Where().c_str());
}

}  // namespace r2c
