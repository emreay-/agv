#include<string>
#include<ros/ros.h>
#include"agv/int16Array.h"
#include<tf/transform_broadcaster.h>
#include<boost/bind.hpp>
#include<boost/ref.hpp>
#include<nav_msgs/Odometry.h>
#include<cmath>

//physical dimensions
const float wheelRadius = 0.1;
const float length = 0.49;
const float vy = 0.0;

//callback function for subscribed joint states
void encoderCallback(ros::NodeHandle &node_handle, const agv::int16Array::ConstPtr& encoderArray)
{
	double encL, encR;
	encL = encoderArray->data[1];
	encR = encoderArray->data[2];
	node_handle.setParam("encL",encL);
	node_handle.setParam("encR",encR);
}

//broadcastTF function definition
void broadcastTF(tf::TransformBroadcaster *tf_broadcaster, ros::Time& current, std::string parent_frame,
                 std::string child_frame,
                 double x, double y, double z,
                 double theta)
{
	// broadcast Transform from vehicle to device
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current;
	odom_trans.header.frame_id = parent_frame;
	odom_trans.child_frame_id = child_frame;
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = z;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
	//broadcasting the transform
	tf_broadcaster->sendTransform(odom_trans);
}

//publishOdometry function definition
void publishOdometry(ros::Publisher *odom_pub, ros::Time& current, std::string parent_frame,
                     std::string child_frame,
                     double x, double y, double z,
                     double theta, double vx, double vy, double wth)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = current;
	odom.header.frame_id = parent_frame;
	odom.child_frame_id = child_frame;
	//setting the position values
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = z;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
	//setting the velocity values
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = wth;
	odom_pub->publish(odom);
}

//main function
int main(int argc, char** argv) 
{
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	double wLeft, wRight, vLeft, vRight, vx, wth, dt, delta_x, delta_y, delta_th;
	double encL, encR, encL_prev, encR_prev, delta_encL, delta_encR, enc_difference, enc_sum;
	double icr, central_icr, dist;
	ros::init(argc, argv, "odom_transform"); //node initialize
	ros::NodeHandle nh_; //node handle object

	tf::TransformBroadcaster broadcaster; //tf broadcaster object
	ros::Subscriber encoder_sub_; //subscriber object
	ros::Publisher odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
	//subscribe to joint_state_pub topic
	encoder_sub_ = nh_.subscribe<agv::int16Array>("encoder_values",
	100, boost::bind(encoderCallback,boost::ref(nh_), _1));
	//time variables
	ros::Time current_time, prev_time;
	prev_time = ros::Time::now();
	encL = 0;
	encR = 0;
	encL_prev = 0;
	encR_prev = 0;

	ros::Rate loop_rate(30); //set loop rate to 30 Hz

	while (ros::ok())
	{
		ros::spinOnce(); //check new messages
		current_time = ros::Time::now(); //set current time
		
		nh_.getParam("encL",encL); //get left wheel encoder value from parameter server
		nh_.getParam("encR",encR); //get right wheel encoder value from parameter server

		delta_encL = (encL-encL_prev)*2*M_PI*wheelRadius/500;
		delta_encR = (encR-encR_prev)*2*M_PI*wheelRadius/500;

		//delta encoder differences divided by 2 and converted to 
		//radians, the divider 2 is cancelled by the 2 of 2*M_PI
		enc_difference = (delta_encR-delta_encL); 
		delta_th = (enc_difference)/length;
		th += delta_th;

		//the average of the delta encoder values converted to meters
		enc_sum = (delta_encL+delta_encR)/2; 
		delta_x = enc_sum*cos((M_PI/2)-(delta_th/2));
		delta_y = enc_sum*sin((M_PI/2)-(delta_th/2));

		x += delta_x;
		y += delta_y;		
		
		dt = (current_time - prev_time).toSec(); //time differential
		vx = delta_x/(dt); //linear velocity in the x axis of ROBOT frame
		wth = delta_th/dt;

	        //broadcast transforms
        	broadcastTF(&broadcaster,current_time,"odom","base_link",x,y,0,th);
       	 	broadcastTF(&broadcaster,current_time,"base_link","wheel_left",0,0,0,0);
	        broadcastTF(&broadcaster,current_time,"base_link","wheel_right",0,0,0,0);
		broadcastTF(&broadcaster,current_time,"base_link","laser",0.25,0,0.1,0);
		//publish odometry
		publishOdometry(&odom_pub_,current_time,"odom","base_link",x,y,0,th,vx,vy,wth);
		//set previouse encoder values
		encL_prev = encL;
		encR_prev = encR;
        	//set previous time
		prev_time = current_time;
        	//rest until loop rate is done
        	loop_rate.sleep();
	}//end of while
	return 0;
}//end of main

