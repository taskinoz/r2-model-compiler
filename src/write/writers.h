// Serialisers for the v53 model container and its embedded files.
#pragma once

#include <cstdint>
#include <vector>

#include "build/compiled.h"

namespace r2c {

std::vector<uint8_t> WriteVVD(const CompiledModel& m, int checksum);
std::vector<uint8_t> WriteVVC(const CompiledModel& m, int checksum);
std::vector<uint8_t> WriteVTX(const CompiledModel& m, int checksum);
// Full v53 .mdl with VTX/VVD/VVC/PHY embedded.
std::vector<uint8_t> WriteMDL(const CompiledModel& m);

}  // namespace r2c
