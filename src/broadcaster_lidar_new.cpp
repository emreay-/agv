#include<string>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Joy.h>
#include<tf/transform_broadcaster.h>
#include<boost/bind.hpp>
#include<boost/ref.hpp>


//broadcastTF function definition
void broadcastTF(tf::TransformBroadcaster *tf_broadcaster, std::string parent_frame,
                 std::string child_frame,
                 double x, double y, double z,
                 double theta)
{
	// broadcast Transform from vehicle to device
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = parent_frame;
	odom_trans.child_frame_id = child_frame;
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = z;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
	//broadcasting the transform
	tf_broadcaster->sendTransform(odom_trans);
}

//main function
int main(int argc, char** argv) 
{
	ros::init(argc, argv, "odom_transform"); //node initialize
	ros::NodeHandle nh_; //node handle object

	tf::TransformBroadcaster broadcaster; //tf broadcaster object

	ros::Rate loop_rate(30); //set loop rate to 30 Hz
	while (ros::ok())
	{
		ros::spinOnce(); //check new messages
		
	        //broadcast transforms
        	broadcastTF(&broadcaster,"odom","base_link",0,0,0,0);
       	 	broadcastTF(&broadcaster,"base_link","wheel_left",0,0,0,0);
	        broadcastTF(&broadcaster,"base_link","wheel_right",0,0,0,0);
		broadcastTF(&broadcaster,"base_link","laser",0.25,0,0.1,180);

        	//rest until loop rate is done
        	loop_rate.sleep();

	}//end of while
	return 0;
}//end of main

