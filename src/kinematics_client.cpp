#include "ros/ros.h"
#include "agv/kinematics.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_client");
  if (argc != 2)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<agv::kinematics>("kinematics");
  agv::kinematics srv;
  srv.request.angle = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Right Wheel: %f \nLeft Wheel: %f", srv.response.wheel[0], srv.response.wheel[1]);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
