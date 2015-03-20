#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"

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
  ros::Publisher pub = n.advertise<std_msgs::String>("order", 1000);
  std_msgs::String go;
  std_msgs::String stop;
  std_msgs::String error;
  go.data = "Go!";
  stop.data = "Stop!";
  error.data = "ERROR..";

  while (ros::ok())
  {
   while(range >= 0)
   { 
    if(range>4 && range<15)
    {
      pub.publish(stop);
    }
    else if(range>15.001)
    {
      pub.publish(go);
    }
    else
    {
      pub.publish(error);
    }
    range = -1;
   }
   ros::spinOnce();
  }
  return 0;
}
