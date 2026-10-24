// QC script tokenizer with studiomdl's line semantics.
//
// Get(false) only returns a token on the current line; Get(true) may cross lines.
// Available() tells whether another token exists on the current line. Included
// files are spliced in place. $definevariable values are substituted into tokens
// written as $name$.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace r2c {

struct Token {
    std::string text;
    std::string file;
    int line = 0;
    bool quoted = false;
    bool newlineBefore = false;  // first token on its line
};

class Tokenizer {
public:
    void LoadFile(const std::string& path);
    // Insert another file's tokens at the current position (for $include).
    void IncludeFile(const std::string& path);
    void InsertTokens(const std::vector<Token>& toks);

    bool Get(bool crossLine);
    bool Available() const;  // another token on the current line?
    void Unget();
    bool AtEnd() const { return pos_ >= toks_.size(); }

    const std::string& Text() const { return cur_.text; }
    const Token& Cur() const { return cur_; }
    std::string Where() const;

    // Helpers that raise a CompileError when the token is missing.
    std::string Need(const char* what);           // same line
    std::string NeedAny(const char* what);        // any line
    float NeedFloat(const char* what);
    int NeedInt(const char* what);

    // Skips tokens until the end of the current line (or past a balanced {} block
    // when one starts on this line).
    void SkipRestOfCommand();
    // Reads a balanced { ... } block and returns its raw tokens (without the outer braces).
    std::vector<Token> ReadBlock();

    std::map<std::string, std::string>& Variables() { return vars_; }
    struct Macro {
        std::vector<std::string> params;
        std::vector<Token> body;
    };
    std::map<std::string, Macro>& Macros() { return macros_; }

    static std::vector<Token> TokenizeText(const std::string& text, const std::string& file);

private:
    std::string Substitute(const std::string& s) const;
    std::vector<Token> toks_;
    size_t pos_ = 0;
    Token cur_;
    std::map<std::string, std::string> vars_;
    std::map<std::string, Macro> macros_;
};

}  // namespace r2c
