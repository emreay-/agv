#include<string>
#include<ros/ros.h>
#include"agv/int16Array.h"
#include<tf/transform_broadcaster.h>
#include<boost/bind.hpp>
#include<boost/ref.hpp>
#include<boost/assign.hpp>
#include<nav_msgs/Odometry.h>
#include<math.h>

//physical dimensions
const float wheelRadius = 0.1;
const float length = 0.49;
const float vy = 0.0;

//callback function for subscribed joint states
void encoderCallback(ros::NodeHandle &node_handle, const agv::int16Array::ConstPtr& encoderArray)
{
	double wL, wR;
	//converting rpm to m/s and than calculating angular velocities
	wL = (encoderArray->data[1])*M_PI/30;
	wR = (encoderArray->data[2])*M_PI/30;
	node_handle.setParam("wLeft",wL);
	node_handle.setParam("wRight",wR);
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
	double wLeft, wRight, vLeft, vRight, vx, wth, dt, delta_x, delta_y, delta_th;

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
	ros::Rate loop_rate(30); //set loop rate to 30 Hz
	nh_.setParam("wLeft", 0);
	nh_.setParam("wRight", 0);

	while (ros::ok())
	{
		ros::spinOnce(); //check new messages
		current_time = ros::Time::now(); //set current time
		
		nh_.getParam("wLeft",wLeft); //get left wheel angular velocity value from parameter server
		nh_.getParam("wRight",wRight); //get right wheel angular velocity value from parameter server

		vLeft = wheelRadius*wLeft; //linear velocity of left wheel
		vRight = wheelRadius*wRight; //linear velocity of rigth wheel
		vx = (vRight+vLeft)/2; //linear velocity in ROBOT frame
		wth = (vRight-vLeft)/length; //angular velocity of the ROBOT
		dt = (current_time - prev_time).toSec(); //time differential
		delta_x = vx*cos(th)*dt; //change in x axis in world frame
		delta_y = vx*sin(th)*dt; //change in y axis in world frame
		delta_th = wth*dt; //change in the orientation

		x += delta_x;
		y += delta_y;
		th += delta_th;

	        //broadcast transforms
        	broadcastTF(&broadcaster,current_time,"odom","base_link",x,y,0,th);
       	 	broadcastTF(&broadcaster,current_time,"base_link","wheel_left",0,0,0,0);
	        broadcastTF(&broadcaster,current_time,"base_link","wheel_right",0,0,0,0);
		broadcastTF(&broadcaster,current_time,"base_link","laser",0.25,0,0.1,0);
		//publish odometry
		publishOdometry(&odom_pub_,current_time,"odom","base_link",x,y,0,th,vx,vy,wth);
        	//set previous time
		prev_time = current_time;
        	//rest until loop rate is done
        	loop_rate.sleep();
	}//end of while
	return 0;
}//end of main

