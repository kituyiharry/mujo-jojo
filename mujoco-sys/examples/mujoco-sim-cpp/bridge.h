#include <memory>
#include <mujoco/mujoco.h>
#include "simulate.h"

using namespace mujoco;

std::unique_ptr<Simulate> createSim(mjvCamera *cam, mjvOption *opt, mjvPerturb *pert, bool is_passive);
void runPhysicsLoop(std::unique_ptr<Simulate> sim);
