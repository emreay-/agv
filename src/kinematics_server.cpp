#include "ros/ros.h"
#include "agv/kinematics.h"

bool findRef(agv::kinematics::Request  &req,
             agv::kinematics::Response &res)
{
  res.wheel[0] = (req.angle)*2.64;
  res.wheel[1] = (req.angle)*-2.64;
  ROS_INFO("request (angle): %f", req.angle);
  ROS_INFO("sending back response (wheel references): \nRight Wheel = %f \nLeft Wheel = %f", res.wheel[0], res.wheel[1]);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("kinematics", findRef);
  ROS_INFO("Ready to go!");
  ros::spin();

  return 0;
}
