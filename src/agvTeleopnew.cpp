#include<string>
#include<ros/ros.h>
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
	int analogL_, analogR_, right2_;
	int select_button, cross_up, cross_right, cross_down, cross_left;
	double l_scale_L, l_scale_R;
	ros::Subscriber joy_sub_; //subscriber object
	}; //end of class definition
	
	//class constructor definition
	TeleopAgv::TeleopAgv():
	  analogL_(1),
	  analogR_(2)
	{
		ROS_INFO("Calling the constructor of class TeleopAgv");
		//getting necessary parameters from parameter server
		nh_.getParam("analog_left_upwards", analogL_);
		nh_.getParam("analog_right_upwards", analogR_);
		nh_.getParam("right2_", right2_);
		nh_.getParam("cross_up", cross_up);
		nh_.getParam("cross_right", cross_right);
		nh_.getParam("cross_down", cross_down);
		nh_.getParam("cross_left", cross_left);
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
		short crossUpState, crossRightState, crossDownState, crossLeftState, r2State;
	        r2State = joy->buttons[right2_];				
		
		if(r2State == 0)
		{
			left_wheel_raw = l_scale_L*joy->axes[analogL_];
			right_wheel_raw = l_scale_R*joy->axes[analogR_];

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
		}

		else if(r2State == 1)
		{
			crossUpState = joy->buttons[cross_up];
			crossRightState = joy->buttons[cross_right];
			crossDownState = joy->buttons[cross_down];
			crossLeftState = joy->buttons[cross_left];
			
			if(crossUpState == 1 && (crossRightState + crossDownState + crossLeftState) == 0)
			{
				left_wheel_dir = 0;
				right_wheel_dir = 0;
				left_wheel_vel = 250;
				right_wheel_vel = 250;
			}

			else if(crossRightState == 1 && (crossDownState + crossLeftState + crossUpState) == 0)
			{
				left_wheel_dir = 0;
				right_wheel_dir = 250;
				left_wheel_vel = 125;
				right_wheel_vel = 125;
			}

			else if(crossDownState == 1 && (crossLeftState + crossUpState + crossRightState) == 0)
			{
				left_wheel_dir = 250;
				right_wheel_dir = 250;
				left_wheel_vel = 250;
				right_wheel_vel = 250;
			}

			else if(crossLeftState == 1 && (crossUpState + crossRightState + crossDownState) == 0)
			{
				left_wheel_dir = 250;
				right_wheel_dir = 0;
				left_wheel_vel = 125;
				right_wheel_vel = 125;
			}

			else 
			{
				left_wheel_vel = 0;
				right_wheel_vel = 0;
			}			
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

