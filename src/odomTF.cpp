#include<string>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Joy.h>
#include<tf/transform_broadcaster.h>
#include<boost/bind.hpp>
#include<boost/ref.hpp>

//physical dimensions
const float wheelRadius = 0.1;
const float length = 0.8;

//callback function for subscribed joint states
void jsCallback(ros::NodeHandle &node_handle, const sensor_msgs::JointState::ConstPtr& jointState)
{
	double wL, wR;
	wL = jointState->velocity[1];
	wR = jointState->velocity[2];
	node_handle.setParam("wLeft",wL);
	node_handle.setParam("wRight",wR);
}

//broadcastTF function definition
void broadcastTF(tf::TransformBroadcaster *tf_broadcaster,
                 std::string device_frame,
                 double x, double y, double z,
                 double theta)
{
	// broadcast Transform from vehicle to device
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = device_frame;
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
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	double wLeft, wRight, vLeft, vRight, vx, wth, dt, delta_x, delta_y, delta_th;

	ros::init(argc, argv, "odom_transform"); //node initialize
	ros::NodeHandle nh_; //node handle object

	tf::TransformBroadcaster broadcaster; //tf broadcaster object
	ros::Subscriber js_sub_; //subscriber object
	//subscribe to joint_state_pub topic
	js_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_state_pub",
	100, boost::bind(jsCallback,boost::ref(nh_), _1));
	//time variables
	ros::Time current_time, prev_time;
	prev_time = ros::Time::now();
	ros::Rate loop_rate(30); //set loop rate to 30 Hz
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
        	broadcastTF(&broadcaster,"base_link",x,y,0,th);
       	 	broadcastTF(&broadcaster,"wheel_left",x,y,0,th);
	        broadcastTF(&broadcaster,"wheel_right",x,y,0,th);
        	//set previous time
		prev_time = current_time;
        	//rest until loop rate is done
        	loop_rate.sleep();
	}//end of while
	return 0;
}//end of main

