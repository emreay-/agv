#include<string>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<math.h>

class rectangularTrajectory
{
	public:
	rectangularTrajectory();
	~rectangularTrajectory();
	void moveAgv(double vx, double vy, double wth);
	void setPrevTime(ros::Time now);
	int getState();

	private:
	void timerCallback(const ros::TimerEvent&);
	void broadcastTF(tf::TransformBroadcaster *tf_broadcaster,
                 	std::string device_frame,
                 	double x, double y, double z,
                 	double theta);
	ros::NodeHandle nh_;
	tf::TransformBroadcaster broadcaster;
	double x, y, th, vx, vy, wth, dt, delta_x, delta_y, delta_th;
	ros::Time current_time, prev_time, duration;
	ros::Timer timer;
};

rectangularTrajectory::rectangularTrajectory()
{
	ROS_INFO("Calling the constructor of the class rectangularTrajectory");
	timer = nh_.createTimer(ros::Duration(1.0), &rectangularTrajectory::timerCallback, this);
	x = 0.0;
	y = 0.0;
	th = M_PI/2;
}

rectangularTrajectory::~rectangularTrajectory()
{
	ROS_INFO("Calling the deconstructor of the class rectangularTrajectory");
}

void rectangularTrajectory::broadcastTF(tf::TransformBroadcaster *tf_broadcaster,
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

void rectangularTrajectory::moveAgv(double vx, double vy, double wth)
{
	current_time = ros::Time::now(); //set current time
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
	ros::spinOnce();
}

void rectangularTrajectory::setPrevTime(ros::Time now)
{
	prev_time = now.now();
}

int rectangularTrajectory::getState()
{
	int state;	
	nh_.getParam("state",state);
	return state;
}

void rectangularTrajectory::timerCallback(const ros::TimerEvent&)
{
	int new_state;
	new_state = getState()+1;
	nh_.setParam("state",new_state);
	ROS_INFO("timer triggered, new state: %d",new_state);
}

//main function
int main(int argc, char** argv) 
{
	ros::init(argc, argv, "rectangular_tf"); //node initialize
	rectangularTrajectory rectangularTrajectory_;
	rectangularTrajectory_.setPrevTime(ros::Time::now());
	while (ros::ok())
	{
		ros::spinOnce();

		while((rectangularTrajectory_.getState())%2 == 0)
		{
			rectangularTrajectory_.moveAgv(4,0,0);
		}
		
		while((rectangularTrajectory_.getState())%2 == 1)
		{
			rectangularTrajectory_.moveAgv(0,0,M_PI/2);
		}

	}//end of while
	return 0;
}//end of main
