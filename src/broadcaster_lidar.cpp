#include<string>
#include<ros/ros.h>
#include"agv/int16Array.h"
#include"agv/float32Array.h"
#include<tf/transform_broadcaster.h>
#include<boost/bind.hpp>
#include<boost/ref.hpp>
#include<boost/assign.hpp>
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
	odom.pose.covariance = boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (1e3);
	odom.twist.covariance = boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (1e3);
	odom_pub->publish(odom);
}

//main function
int main(int argc, char** argv) 
{
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	double delta_x = 0.0;
	double delta_y = 0.0;
	double delta_th = 0.0;
	double vx, wth, dt;
	double encL, encR, encL_prev, encR_prev, delta_encL, delta_encR, enc_difference, enc_avg, dist;

	ros::init(argc, argv, "odom_transform"); //node initialize
	ros::NodeHandle nh_; //node handle object

	tf::TransformBroadcaster broadcaster; //tf broadcaster object
	ros::Subscriber encoder_sub_; //subscriber object
	ros::Publisher odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Publisher debug_pub_ = nh_.advertise<agv::float32Array>("debug", 50);
	//subscribe to joint_state_pub topic
	encoder_sub_ = nh_.subscribe<agv::int16Array>("encoder_values",
	100, boost::bind(encoderCallback,boost::ref(nh_), _1));
	//time variables
	ros::Time current_time, prev_time;
	prev_time = ros::Time::now();

	agv::float32Array debug;

	encL = 0;
	encR = 0;
	encL_prev = 0;
	encR_prev = 0;

	ros::Rate loop_rate(5); //set loop rate to 30 Hz

	while (ros::ok())
	{
		ros::spinOnce(); //check new messages
		current_time = ros::Time::now(); //set current time
		
		nh_.getParam("encL",encL); //get left wheel encoder value from parameter server
		nh_.getParam("encR",encR); //get right wheel encoder value from parameter server

		//calculating the changes in encoders and converting them to meters
		delta_encL = (encL-encL_prev)*2*M_PI*wheelRadius/500;
		delta_encR = (encR-encR_prev)*2*M_PI*wheelRadius/500;
		//the average of the delta encoder values
		enc_avg = (delta_encL+delta_encR)/2; 
		dist = enc_avg;
		//the difference between the delta encoder values
		enc_difference = (delta_encR-delta_encL);
		//calculation of the change in the orientation 
		delta_th = (enc_difference)/length;
		debug.data[0]=delta_th;
		//calculation of the changes in the position in world map frames
		delta_x = dist*sin((M_PI/2)-(delta_th/2))*cos(th);
		delta_y = dist*sin((M_PI/2)-(delta_th/2))*sin(th);
		//calculation of the current position and orientation
		x += delta_x;
		y += delta_y;		
		th = fmod((th+delta_th),2*M_PI);
		debug.data[1]=th;
		dt = (current_time - prev_time).toSec(); //time differential
		vx = delta_x/(cos(th)*dt); //linear velocity in the x axis of ROBOT frame
		debug.data[2]=vx;
		wth = delta_th/dt;
		debug.data[3]=wth;
	        //broadcast transforms
        	broadcastTF(&broadcaster,current_time,"odom","base_link",x,y,0,th);
       	 	broadcastTF(&broadcaster,current_time,"base_link","wheel_left",0,0,0,0);
	        broadcastTF(&broadcaster,current_time,"base_link","wheel_right",0,0,0,0);
		broadcastTF(&broadcaster,current_time,"base_link","laser",0.25,0,0.1,0);
		//publish odometry
		publishOdometry(&odom_pub_,current_time,"odom","base_link",x,y,0,th,vx,vy,wth);
		debug_pub_.publish(debug);
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

