from .BasePrompt import BasePrompt
import copy
class NodeQuery(BasePrompt):
    def __init__(self) -> None:
        super().__init__()
        self.background = """
Act as an expert software developer.
You are diligent and tireless!
You NEVER leave comments describing code without implementing it!
You always COMPLETELY IMPLEMENT the needed code!
Always use best practices when coding.
Respect and use existing conventions, libraries, etc that are already present in the code base.

You are assisting a behavior-tree designer, who designs behaviors in the hunav_agent_manager repository using the BehaviorTree.cpp, The map of the BehaviorTree.CPP repository is given below:

The map of the hunav_agent_manager repository is given below:

hunav_agent_manager/src/agent_manager.cpp:
⋮...
│namespace hunav
│{
│
⋮...
│  AgentManager::AgentManager()
⋮...
│  void AgentManager::init()
⋮...
│  bool AgentManager::isRobotVisible(int id, double dist)
│  {
│    std::lock_guardstd::mutex guard(mutex_);
⋮...
│  void AgentManager::lookAtTheRobot(int id)
│  {
│
│    std::lock_guardstd::mutex guard(mutex_);
⋮...
│  void AgentManager::approximateRobot(int id, double dt)
│  {
│
│    std::lock_guardstd::mutex guard(mutex_);
│
⋮...
│  void AgentManager::blockRobot(int id, double dt)
│  {
│
│    std::lock_guardstd::mutex guard(mutex_);
│
⋮...
│  void AgentManager::avoidRobot(int id, double dt)
│  {
│    std::lock_guardstd::mutex guard(mutex_);
│
⋮...
│  bool AgentManager::goalReached(int id)
│  {
│    std::lock_guardstd::mutex guard(mutex_);
⋮...
│  bool AgentManager::updateGoal(int id)
│  {
│    std::lock_guardstd::mutex guard(mutex_);
│
⋮...
│  hunav_msgs::msg::Agents AgentManager::getUpdatedAgentsMsg()
│  {
⋮...
│    std::lock_guardstd::mutex guard(mutex_);
⋮...
│  void AgentManager::computeForces(int id)
⋮...
│  void AgentManager::computeForces()
⋮...
│  void AgentManager::updateAllAgents(
│      const hunav_msgs::msg::Agent::SharedPtr robot_msg,
│      const hunav_msgs::msg::Agents::SharedPtr agents_msg)
│  {
│
│    std::lock_guardstd::mutex guard(mutex_);
│
⋮...
│  void AgentManager::updateAgentsAndRobot(
│      const hunav_msgs::msg::Agents::SharedPtr agents_msg)
│  {
│
│    std::lock_guardstd::mutex guard(mutex_);
│
⋮...
│  void AgentManager::updatePosition(int id, double dt)
│  {
│
│    std::lock_guardstd::mutex guard(mutex_);
│
⋮...

hunav_agent_manager/src/bt_functions.cpp:
⋮...
│#include "hunav_agent_manager/bt_functions.hpp"
│
│namespace hunav
│{
│
│  BTfunctions::BTfunctions()
⋮...
│  void BTfunctions::init()
⋮...

hunav_agent_manager/src/time_expired_condition.cpp:
⋮...
│namespace hunav {
│
│TimeExpiredCondition::TimeExpiredCondition(const std::string &condition_name,
│                                           const BT::NodeConfiguration &conf)
│    : BT::ConditionNode(condition_name, conf), period_(1.0) {
│  getInput("seconds", period_);
│  // node_ = config().blackboard->getrclcpp::Node::SharedPtr("node");
│  // start_ = node_->now();
│  getInput("ts", dt_);
│  getInput("only_once", only_once_);
⋮...

The designer will write Behavior Trees in XML for the humans in the task and might define custom actions, conditions, decorators or sub-trees. You will be given the [DESCRIPTION] of what these custom nodes must do and your job is to write corresponding functions using the BehaviorTree.cpp package. 
Code for each custom node consists of 3 components:

1. NODE NAME: The name of the node, which will be used to call the node in the BehaviorTree.cpp file.
2. NODE TYPE: The type of the node, which can be either: ACTION or CONDITION
3. NODE DEFINITION: A definition of the logic of the behavior of the node, that is, when the node returns SUCCESS or FAILURE when the node is ticked. THIS FUNCTION WILL BE ADDED TO hunav_agent_manager/src/bt_functions.cpp
4. NODE HEADER: The header for each custom node to be added to the hunav_agent_manager/include/bt_functions.hpp file
5. PORTS USED: The input ports used by the node. These can be either: ((agent_id),(agent_id,distance),(agent_id,time_step))
6. AUX FUNCTIONS: Any auxiliary functions needed for running the logic of the node. THESE FUNCTIONS WILL BE ADDED TO hunav_agent_manager/src/agent_manager.cpp
7. AUX FUNCTION HEADERS: The header for each auxillary function to be added to hunav_agent_manager/include/agent_manager.hpp

Examples of NODE DEFINITION are given below:
//check if the robot is visible to an agent
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
if (agent_manager_.isRobotVisible(id, dist))
{
return BT::NodeStatus::SUCCESS;
}
else
{
return BT::NodeStatus::FAILURE;
}
}

//check if an agent has reached their goal
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
return BT::NodeStatus::SUCCESS;
}
else
{
return BT::NodeStatus::FAILURE;
}
}

//update the goal of an agent by popping the top goal
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
}}

//Regular Navigation
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
agent_manager_.updatePosition(id, dt); //update the position of an agent in the simulator
return BT::NodeStatus::SUCCESS;

}
//Surprised Navigation
BT::NodeStatus BTfunctions::surprisedNav(BT::TreeNode &self)
{
auto msg = self.getInput<int>("agent_id");
if (!msg)
{
throw BT::RuntimeError("SurprisedNav. missing required input [agent_id]: ",
msg.error());
}
int id = msg.value();
agent_manager_.lookAtTheRobot(id);
return BT::NodeStatus::SUCCESS;
}
The AUX FUNCTIONS currently available are given below:
float AgentManager::robotSquaredDistance(int id)
{
float xa = agents_[id].sfmAgent.position.getX(); // position.position.x;
float ya = agents_[id].sfmAgent.position.getY(); // position.y;
float xr = robot_.sfmAgent.position.getX();      // position.x;
float yr = robot_.sfmAgent.position.getY();      // position.y;
return (xr - xa) * (xr - xa) + (yr - ya) * (yr - ya);
}

bool AgentManager::lineOfSight(int id)
{
float ax = agents_[id].sfmAgent.position.getX();
float ay = agents_[id].sfmAgent.position.getY();
float rx = robot_.sfmAgent.position.getX();
float ry = robot_.sfmAgent.position.getY();
double yaw = agents_[id].sfmAgent.yaw.toRadian();
float nrx = (rx - ax) * cos(yaw) + (ry - ay) * sin(yaw);
float nry = -(rx - ax) * sin(yaw) + (ry - ay) * cos(yaw);
float rangle = atan2(nry, nrx);

if (abs(rangle) > (M_PI / 2.0 + 0.17))
{
  return false;
}
else
{
  return true;
}

}

bool AgentManager::isRobotVisible(int id, double dist)
{
std::lock_guardstd::mutex guard(mutex_);
float squared_dist = robotSquaredDistance(id);
if (squared_dist <= (dist * dist))
{
return lineOfSight(id);
}
else
{
return false;
}
}

void AgentManager::lookAtTheRobot(int id)
{

std::lock_guard<std::mutex> guard(mutex_);
// Robot position
float rx = robot_.sfmAgent.position.getX();
float ry = robot_.sfmAgent.position.getY();
// Agent position
float ax = agents_[id].sfmAgent.position.getX();
float ay = agents_[id].sfmAgent.position.getY();
float ah = agents_[id].sfmAgent.yaw.toRadian();
// Transform robot position to agent coords system
float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
utils::Angle robotYaw;
robotYaw.setRadian(atan2(nry, nrx));
float max_ang_vel = M_PI; // rad/secs
utils::Angle max_angle =
    utils::Angle::fromRadian(max_ang_vel * 0.01);
if (robotYaw.sign() < 0)
  max_angle.setRadian(max_angle.toRadian() * (-1));

// Update the agent angle
if (fabs(robotYaw.toRadian()) > max_angle.toRadian())
{
  agents_[id].sfmAgent.yaw = (agents_[id].sfmAgent.yaw + max_angle);
}
else
{
  agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
}
agents_[id].behavior_state = 1;

}
void AgentManager::blockRobot(int id, double dt)
{

std::lock_guard<std::mutex> guard(mutex_);

agents_[id].behavior_state = 1;

// Robot position
float rx = robot_.sfmAgent.position.getX();
float ry = robot_.sfmAgent.position.getY();

float h = robot_.sfmAgent.yaw.toRadian();

// Store the initial set o goals
std::list<sfm::Goal> gls = agents_[id].sfmAgent.goals;

// Change the agent goal.
// We should compute a goal in front of the robot heading.
float newgx = rx + 1.1 * sin(h);
float newgy = ry + 1.1 * cos(h);
sfm::Goal g;
g.center.set(newgx, newgy);
g.radius = 0.05; // robot_.sfmAgent.radius;
agents_[id].sfmAgent.goals.push_front(g);
// change agent vel according to the proximity of the robot
// move slowly when close
float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
agents_[id].sfmAgent.desiredVelocity = 2.0;
// recompute forces
computeForces(id);
// update position
sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
// if the agent is close to the robot,
// look at the robot
float dist = sqrt(robotSquaredDistance(id));
if (dist <= 1.2)
{
  // Agent position
  float ax = agents_[id].sfmAgent.position.getX();
  float ay = agents_[id].sfmAgent.position.getY();
  float ah = agents_[id].sfmAgent.yaw.toRadian();
  // Transform robot position to agent coords system
  float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
  float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
  utils::Angle robotYaw; // = utils::Angle::fromRadian(atan2(nry, nrx));
  robotYaw.setRadian(atan2(nry, nrx));
  agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
}

// restore the goals and the velocity
agents_[id].sfmAgent.goals = gls;
agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;

}

void AgentManager::avoidRobot(int id, double dt)
{
std::lock_guardstd::mutex guard(mutex_);

agents_[id].behavior_state = 1;

// we decrease the maximum velocity
double init_vel = agents_[id].sfmAgent.desiredVelocity;
agents_[id].sfmAgent.desiredVelocity = 0.6;
computeForces(id);

// We add an extra repulsive force from the robot
utils::Vector2d minDiff =
    agents_[id].sfmAgent.position - robot_.sfmAgent.position;
double distance = minDiff.norm() - agents_[id].sfmAgent.radius;

utils::Vector2d Scaryforce =
    20.0 * (agents_[id].sfmAgent.params.forceSigmaObstacle / distance) *
    minDiff.normalized();
agents_[id].sfmAgent.forces.globalForce += Scaryforce;

// update position
sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

// restore desired vel
agents_[id].sfmAgent.desiredVelocity = init_vel;

}

bool AgentManager::goalReached(int id)
{
std::lock_guardstd::mutex guard(mutex_);
if (!agents_[id].sfmAgent.goals.empty() &&
(agents_[id].sfmAgent.goals.front().center -
agents_[id].sfmAgent.position)
.norm() <= (agents_[id].sfmAgent.goals.front().radius + 0.1))
{
return true;
}
else
return false;
}

bool AgentManager::updateGoal(int id)
{
std::lock_guardstd::mutex guard(mutex_);
sfm::Goal g = agents_[id].sfmAgent.goals.front();
agents_[id].sfmAgent.goals.pop_front();
if (agents_[id].sfmAgent.cyclicGoals)
{
agents_[id].sfmAgent.goals.push_back(g);
}
return true;
}

void AgentManager::updatePosition(int id, double dt)
{
std::lock_guardstd::mutex guard(mutex_);
agents_[id].behavior_state = 0;
sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
}
###
Output the custom nodes and auxillary functions in the JSON format shown in the example below:
Note that the function name is in camel case while the node name is in pascal case.
###
USER: [DESCRIPTION]: Write a BT node called curiousNav that performs the following logic: Agent approaches the robot slowly and when the agent is closer than 1.5 units, the agent stops and looks at the robot.
ASSISTANT:
{ 
NODE_NAME: 'curiousNav',
NODE_TYPE:'ACTION',
NODE_DEFINITION:
  ""
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
  "",
  NODE_HEADER:
    "BT::NodeStatus curiousNav(BT::TreeNode &self);",
  ,
  PORTS_USED:[
    "agent_id",
    "time_step",
  ],
  AUX_FUNCTIONS:[
""void AgentManager::approximateRobot(int id, double dt)
  {

    std::lock_guard<std::mutex> guard(mutex_);

    agents_[id].behavior_state = 1;

    // Robot position
    float rx = robot_.sfmAgent.position.getX();
    float ry = robot_.sfmAgent.position.getY();
    float dist = sqrt(robotSquaredDistance(id));

    // if the agent is close to the robot,
    // stop and look at the robot
    if (dist <= 1.5)
    {
      // Agent position
      float ax = agents_[id].sfmAgent.position.getX();
      float ay = agents_[id].sfmAgent.position.getY();
      float ah = agents_[id].sfmAgent.yaw.toRadian();
      // Transform robot position to agent coords system
      float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
      float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
      utils::Angle robotYaw; // = utils::Angle::fromRadian(atan2(nry, nrx));
      robotYaw.setRadian(atan2(nry, nrx));

      agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
    }
    else
    {

      // Change the agent goal
      sfm::Goal g;
      g.center.set(rx, ry);
      g.radius = robot_.sfmAgent.radius;
      agents_[id].sfmAgent.goals.push_front(g);

      // change agent vel according to the proximity of the robot
      // move slowly when close
      float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
      agents_[id].sfmAgent.desiredVelocity = 1.8 * (dist / max_dist_view_);
      // recompute forces
      computeForces(id);
      // update position
      sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
      // restore values just in case the approximation
      // ends in the next iteration
      agents_[id].sfmAgent.goals.pop_front();
      agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
    }
  }
  ""
  ],
  AUX_FUNCTION_HEADERS: [
    "void approximateRobot(int id, double dt);"
  ]
  }
###
USER: <NODE_QUERY>
ASSISTANT:"""

    def get_full_prompt(self,**kwargs):
        full_prompt = copy.deepcopy(self.background)
        full_prompt = full_prompt.replace('<NODE_QUERY>',kwargs['description'])
        return dict(
             system = self.system_prompt,
             user = [
                 {
                     'type': 'text',
                     'content': full_prompt
                 }
             ]
         )