// DMX reader (placeholder; implemented in a later step).
#include "core/log.h"
#include "input/source.h"

namespace r2c {

std::shared_ptr<Source> LoadDMX(const std::string& path) { Fatal("DMX input is not supported yet: %s", path.c_str()); }

}  // namespace r2c
