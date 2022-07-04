# types
uint8 PERSON=1
uint8 ROBOT=2
uint8 OTHER=3

# behaviors
uint8 BEH_REGULAR=1
uint8 BEH_IMPASSIVE=2
uint8 BEH_SURPRISED=3
uint8 BEH_SCARED=4
uint8 BEH_CURIOUS=5
uint8 BEH_THREATENING=6

# behavior states
uint8 BEH_NO_ACTIVE=0
uint8 BEH_ACTIVE_1=1
uint8 BEH_ACTIVE_2=2

int32 id
uint8 type
uint8 behavior
string name
uint8 behavior_state
int8 group_id
geometry_msgs/Pose position
float32 yaw
geometry_msgs/Twist velocity
float32 desired_velocity
float32 radius
float32 linear_vel
float32 angular_vel

geometry_msgs/Pose[] goals
bool cyclic_goals
float32 goal_radius

geometry_msgs/Point[] closest_obs

