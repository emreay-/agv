#include<string>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Joy.h>
#include<tf/transform_broadcaster.h>

	//definition of the class TeleopAgv
	class TeleopAgv
	{
	public:
	TeleopAgv(); //constructor
	~TeleopAgv(); //deconstructor

	private:
	//callback function for joystick
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	//node handle object	
	ros::NodeHandle nh_;
	
	int linearL_, linearR_;
	double l_scale_L, l_scale_R;	
	ros::Publisher js_pub_; //publisher object
	ros::Subscriber joy_sub_; //subscriber object
	}; //end of class definition
	
	//class constructor definition
	TeleopAgv::TeleopAgv():
	  linearL_(1),
	  linearR_(2)
	{
		ROS_INFO("Calling the constructor of class TeleopAgv");
		//getting necessary parameters from parameter server
		nh_.getParam("axis_linear_L", linearL_);
		nh_.getParam("axis_linear_R", linearR_);
		nh_.getParam("scale_linear_L", l_scale_L);
		nh_.getParam("scale_linear_R", l_scale_R);
		//advertising to the topic joint_state_pub
		js_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state_pub", 1);
		//subscribing to the topic joy
		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",1000, &TeleopAgv::joyCallback, this);
	}
	
	//class deconstructor definition
	TeleopAgv::~TeleopAgv()
	{
		ROS_INFO("Calling the destructor of class TeleopAgv");
	}

	//joystick callback function definition
	void TeleopAgv::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
	        sensor_msgs::JointState joint_state; //joint state object
          	joint_state.header.stamp = ros::Time::now(); //set header time stamp
		//resize the number of names that would be given to the joint states
       		joint_state.name.resize(3); 
		//resize the number of (angular) velocity values of the joint states
	        joint_state.velocity.resize(3);
		//set the name of the first joint
        	joint_state.name[0] ="base";
		//set the name of the second joint as left
	        joint_state.name[1] ="left_wheel";
		//set the name of the third joint as right
        	joint_state.name[2] ="right_wheel";
		//set the velocity of left wheel from the left analog joystick value
	        joint_state.velocity[1] = l_scale_L*joy->axes[linearL_];
		//set the velocity of left wheel from the left analog joystick value
		joint_state.velocity[2] = l_scale_R*joy->axes[linearR_];
		//publish joint states
		js_pub_.publish(joint_state);
	}

	//main function
	int main(int argc, char** argv) 
	{
		//initialize the node
		ros::init(argc, argv, "jsPublisher");
		//construct a TeleopAgv object
		TeleopAgv teleop_agv;
		//spin
		ros::spin();
	}//end of main

