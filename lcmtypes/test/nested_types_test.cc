// This test is simply to confirm that nested LCM-generated C++ messages are
// code-gen'd correctly, and can be included at will.  If something isn't
// working, it will fail to compile.

#include "drake/common/unused.h"
#include "drake/lcmt_robot_plan.hpp"  // XXX

int main() {
  drake::lcmt_robot_plan plan{};
  unused(plan);

  drake::lcmt_robot_state state{};
  unused(state);

  return 0;
}
