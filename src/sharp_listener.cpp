#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Bool.h"

float range = -1;

void rangeCallback(const sensor_msgs::Range::ConstPtr& range_data)
{
  range = range_data->range;
  ROS_INFO("I heard: [%f]", range);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sharp_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("range_data", 1000, rangeCallback);
  ros::Publisher pub = n.advertise<std_msgs::Bool>("order", 1000);
  std_msgs::Bool order;

  while (ros::ok())
  {
   while(range >= 0)
   { 
    if(range>4 && range<15)
    {
      order.data = 0;
      pub.publish(order);
    }
    else if(range>15.001)
    {
      order.data = 1;
      pub.publish(order);
    }
    else
    {
      order.data = 0;
      pub.publish(order);
    }
    range = -1;
   }
   ros::spinOnce();
  }
  return 0;
}
