#include "ros/ros.h"
#include "std_msgs/String.h"


void orderCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "order_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("order", 1000, orderCallback);
  ros::spin();

  return 0;
}
