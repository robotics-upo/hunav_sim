#ifndef HUNAV_BEHAVIORS__SFM_AGENTS_HPP_
#define HUNAV_BEHAVIORS__SFM_AGENTS_HPP_

#include <math.h> /* fabs */
#include <string>
#include <vector>

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace hunav_agent_manager {

class SFMAgents {
public:
  SFMAgents();
  ~SFMAgents();

protected:
  void computeForces();
  std::vector<sfm::Agent> sfm_agents_;
  sfm::Agent sfm_robot_;
};
} // namespace hunav_agent_manager
#endif // HUNAV_BEHAVIORS__SFM_AGENTS_HPP_
