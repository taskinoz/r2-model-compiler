// Collision model ($collisionmodel / $collisionjoints) -> PHY. Placeholder until the
// IVP compact surface writer lands.
#include "build/context.h"
#include "core/log.h"

namespace r2c {

void BuildCollisionModel(BuildContext& ctx) {
    if (ctx.qc.collision.present) Warn("collision models are not compiled yet; the model will have no PHY");
}

void BuildPerTriAABB(BuildContext& ctx) {
    ctx.out.perTri.version = 0;
}

}  // namespace r2c
