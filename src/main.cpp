// r2studiomdl: Titanfall 2 (MDL v53) model compiler.
//
// Command line mirrors Valve's studiomdl so Crowbar can drive it:
//   r2studiomdl [-game <dir>] [-nop4] [-verbose] [-quiet] [-o <out.mdl>] model.qc
// Output goes to <game>/models/<$modelname>, or next to the QC when -game is absent.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "build/context.h"
#include "core/log.h"
#include "core/strings.h"
#include "qc/qc.h"
#include "write/writers.h"

using namespace r2c;

static void Usage() {
    std::printf(
        "r2studiomdl - Titanfall 2 model compiler (MDL v53)\n"
        "usage: r2studiomdl [options] <file.qc>\n"
        "  -game <dir>     game/mod directory; output is written to <dir>/models/<$modelname>\n"
        "  -o <file>       write the .mdl to this exact path\n"
        "  -outdir <dir>   write to <dir>/<$modelname>\n"
        "  -path <dir>     extra directory to search for SMD/DMX files (repeatable)\n"
        "  -verbose        more output\n"
        "  -quiet          errors and warnings only\n"
        "  -nop4, -nox360, -fullcollide and other studiomdl flags are accepted and ignored\n");
}

int main(int argc, char** argv) {
    std::string qc, game, outFile, outDir;
    QcOptions opt;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "ERROR: %s needs a value\n", a.c_str());
                std::exit(1);
            }
            return argv[++i];
        };
        if (a.size() > 1 && a[0] == '-') {
            std::string l = ToLower(a);
            if (l == "-game") game = next();
            else if (l == "-o" || l == "-out") outFile = next();
            else if (l == "-outdir") outDir = next();
            else if (l == "-path") opt.searchPaths.push_back(next());
            else if (l == "-verbose" || l == "-v") SetLogLevel(LogLevel::Verbose);
            else if (l == "-quiet") SetLogLevel(LogLevel::Quiet);
            else if (l == "-h" || l == "-help" || l == "--help") {
                Usage();
                return 0;
            } else if (l == "-t" || l == "-a" || l == "-minlod") {
                next();
            } else {
                // -nop4, -nox360, -fullcollide, -allowdebug, ... : not applicable
            }
        } else {
            qc = a;
        }
    }
    if (qc.empty()) {
        Usage();
        return 1;
    }
    if (GetExtension(qc).empty()) qc += ".qc";
    auto t0 = std::chrono::steady_clock::now();
    try {
        Info("qdir:    \"%s\"", DirName(std::filesystem::absolute(std::filesystem::u8path(qc)).u8string()).c_str());
        if (!game.empty()) Info("gamedir: \"%s\"", game.c_str());
        Info("Working on \"%s\"", qc.c_str());
        auto def = ParseQC(qc, opt);
        CompiledModel model = Build(*def);
        std::vector<uint8_t> mdl = WriteMDL(model);

        std::filesystem::path out;
        if (!outFile.empty())
            out = std::filesystem::u8path(outFile);
        else if (!outDir.empty())
            out = std::filesystem::u8path(outDir) / std::filesystem::u8path(model.name);
        else if (!game.empty())
            out = std::filesystem::u8path(game) / "models" / std::filesystem::u8path(model.name);
        else
            out = std::filesystem::u8path(def->qcDir) / std::filesystem::u8path(FileBase(model.name) + ".mdl");
        std::error_code ec;
        if (out.has_parent_path()) std::filesystem::create_directories(out.parent_path(), ec);
        Info("---------------------");
        Info("writing %s:", out.u8string().c_str());
        std::ofstream f(out, std::ios::binary);
        if (!f) Fatal("could not write %s", out.u8string().c_str());
        f.write(reinterpret_cast<const char*>(mdl.data()), static_cast<std::streamsize>(mdl.size()));
        f.close();
        int verts = 0, tris = 0;
        for (auto& bp : model.bodyParts)
            for (auto& md : bp.models) {
                verts += md.NumVerts();
                for (auto& me : md.meshes)
                    if (!me.lods.empty()) tris += static_cast<int>(me.lods[0].indices.size() / 3);
            }
        Info("bones       %7d", static_cast<int>(model.bones.size()));
        Info("vertices    %7d", verts);
        Info("triangles   %7d", tris);
        Info("sequences   %7d", static_cast<int>(model.sequences.size()));
        Info("animations  %7d", static_cast<int>(model.animations.size()));
        Info("total       %7d", static_cast<int>(mdl.size()));
        double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        Info("Completed \"%s\" in %.2f seconds (%d warnings)", qc.c_str(), secs, WarningCount());
    } catch (const CompileError& e) {
        std::printf("ERROR: %s\n", e.what());
        return 1;
    } catch (const std::exception& e) {
        std::printf("ERROR: internal: %s\n", e.what());
        return 2;
    }
    return 0;
}
