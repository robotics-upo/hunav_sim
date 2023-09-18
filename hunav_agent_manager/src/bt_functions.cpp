
#include "hunav_agent_manager/bt_functions.hpp"

namespace hunav
{

  BTfunctions::BTfunctions()
  {
    // int Box::objectCount = 0;
    // AgentManager::agents_.clear();
    // AgentManager::sfm_agents_.clear();
    init();
    printf("[BTfunctions.Constructor] initialized!\n");
  }

  BTfunctions::~BTfunctions() {}

  void BTfunctions::init()
  {
    printf("[BTfunctions.init] initialized!\n");
    // agent_manager_.init();
  }

  BT::NodeStatus BTfunctions::robotVisible(BT::TreeNode &self)
  {
    auto idmsg = self.getInput<int>("agent_id");
    if (!idmsg)
    {
      throw BT::RuntimeError("RobotVisible. missing required input [agent_id]: ",
                             idmsg.error());
    }
    auto dmsg = self.getInput<double>("distance");
    if (!dmsg)
    {
      throw BT::RuntimeError("RobotVisible. missing required input [distance]: ",
                             dmsg.error());
    }

    int id = idmsg.value();
    double dist = dmsg.value();
    // std::cout << "BTfunctions.robotVisible. Ticking agent: " << id <<
    // std::endl;
    if (agent_manager_.isRobotVisible(id, dist))
    {
      // std::cout << "BTfunctions.robotVisible. Returning success" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      // std::cout << "BTfunctions.robotVisible. Returning failure" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus BTfunctions::goalReached(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    if (!msg)
    {
      throw BT::RuntimeError("GoalReached. missing required input [agent_id]: ",
                             msg.error());
    }
    int id = msg.value();
    if (agent_manager_.goalReached(id))
    {
      // std::cout << "BTfunctions.GoalReached. agent: " << id << " Goal Reached!"
      //          << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      // std::cout << "BTfunctions.GoalReached. agent: " << id << " Goal not
      // reached"
      //           << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus BTfunctions::updateGoal(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    if (!msg)
    {
      throw BT::RuntimeError("UpdateGoal. missing required input [agent_id]: ",
                             msg.error());
    }
    int id = msg.value();
    if (agent_manager_.updateGoal(id))
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }

  // BT::NodeStatus BTfunctions::impassiveNav(BT::TreeNode &self) {
  //  return regularNav(self);
  //}

  BT::NodeStatus BTfunctions::regularNav(BT::TreeNode &self)
  {

    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError("RegularNav. missing required input [agent_id]: ",
                             msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError("RegularNav. missing required input [time_step]: ",
                             msg2.error());
    }

    int id = msg.value();
    double dt = msg2.value();
    // double dt = (_info.simTime - this->lastUpdate).Double();
    // std::cout << "[BTfunctions.RegularNav] Ticking agent: " << id << " dt: "
    // << dt
    //           << std::endl;
    // Update SFM model position
    agent_manager_.updatePosition(id, dt);
    // sfm::SFM.updatePosition(sfm_agents_[id], time_step_secs_);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus BTfunctions::surprisedNav(BT::TreeNode &self)
  {
    // auto msg = self.getInput<std::string>("agent_id");
    auto msg = self.getInput<int>("agent_id");
    if (!msg)
    {
      throw BT::RuntimeError("SurprisedNav. missing required input [agent_id]: ",
                             msg.error());
    }
    // int id = std::stoi(msg.value()); // BT::convertFromString<int>(msg);
    int id = msg.value();
    // std::cout << "BTfunctions.SurprisedNav. Ticking agent: " << id <<
    // std::endl;
    // stop the agent and just look at the robot (change the agent orientation)
    agent_manager_.lookAtTheRobot(id);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus BTfunctions::curiousNav(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError("CuriousNav. missing required input [agent_id]: ",
                             msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError("CuriousNav. missing required input [time_step]: ",
                             msg2.error());
    }
    int id = msg.value();
    double dt = msg2.value();

    agent_manager_.approximateRobot(id, dt);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus BTfunctions::scaredNav(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError("ScaredNav. missing required input [agent_id]: ",
                             msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError("ScaredNav. missing required input [time_step]: ",
                             msg2.error());
    }
    int id = msg.value();
    // printf("[BTfunctions.ScareNav] After getting id: %i\n", id);
    double dt = msg2.value();

    agent_manager_.avoidRobot(id, dt);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus BTfunctions::threateningNav(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError(
          "threateningNav. missing required input [agent_id]: ", msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError(
          "threateningNav. missing required input [time_step]: ", msg2.error());
    }
    int id = msg.value();
    double dt = msg2.value();

    agent_manager_.blockRobot(id, dt);
    return BT::NodeStatus::SUCCESS;
  }

} // namespace hunav
