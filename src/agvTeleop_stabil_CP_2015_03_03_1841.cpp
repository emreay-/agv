#include<string>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Joy.h>
#include<tf/transform_broadcaster.h>
#include<agv/uint8Array.h>

	//definition of the class TeleopAgv
	class TeleopAgv
	{
	public:
	TeleopAgv(); //constructor
	~TeleopAgv(); //deconstructor
	ros::Publisher wheel_pub_; //publisher object
	agv::uint8Array wheelArr;

	private:
	//callback function for joystick
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	//node handle object	
	ros::NodeHandle nh_;
	int linearL_, linearR_;
	double l_scale_L, l_scale_R;
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
		wheel_pub_ = nh_.advertise<agv::uint8Array>("wheel_values", 1);
		//subscribing to the topic joy
		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",10, &TeleopAgv::joyCallback, this);
		wheelArr.data[0] = 35;
		wheelArr.data[1] = 0;
		wheelArr.data[2] = 0;
		wheelArr.data[3] = 0;
		wheelArr.data[4] = 0;
		wheelArr.data[5] = 0;
		wheelArr.data[6] = 0;
		wheelArr.data[7] = 0;
		wheelArr.data[8] = 0;
		wheelArr.data[9] = 33;
	}
	
	//class deconstructor definition
	TeleopAgv::~TeleopAgv()
	{
		ROS_INFO("Calling the destructor of class TeleopAgv");
	}

	//joystick callback function definition
	void TeleopAgv::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{   
		double left_wheel_raw, right_wheel_raw;
		short left_wheel_dir, right_wheel_dir, left_wheel_vel, right_wheel_vel;
	        left_wheel_raw = l_scale_L*joy->axes[linearL_];
		right_wheel_raw = l_scale_R*joy->axes[linearR_];

		if(left_wheel_raw<0)
		{
			left_wheel_dir = 250;
			left_wheel_vel = -28 * left_wheel_raw;
		}
		else
		{
			left_wheel_dir = 0;
			left_wheel_vel = 28 * left_wheel_raw;
		}
		if(right_wheel_raw<0)
		{
			right_wheel_dir = 250;
			right_wheel_vel = -28 * right_wheel_raw;
		}
		else
		{
			right_wheel_dir = 0;
			right_wheel_vel = 28 * right_wheel_raw;
		}
		
		wheelArr.data[0] = 35;
		wheelArr.data[1] = left_wheel_dir;
		wheelArr.data[2] = 0;
		wheelArr.data[3] = left_wheel_vel;
		wheelArr.data[4] = 0;
		wheelArr.data[5] = right_wheel_dir;
		wheelArr.data[6] = 0;
		wheelArr.data[7] = right_wheel_vel;
		wheelArr.data[8] = 0;
		wheelArr.data[9] = 33;
	}

	//main function
	int main(int argc, char** argv) 
	{
		//initialize the node
		ros::init(argc, argv, "agvTeleop");
		//construct a TeleopAgv object
		TeleopAgv teleop_agv;
		ros::Rate loop_rate(5);
		while(ros::ok())
		{
			ros::spinOnce();
			teleop_agv.wheel_pub_.publish(teleop_agv.wheelArr);
			loop_rate.sleep();
		}
	}//end of main

