#include<string>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<math.h>

class rectangularTrajectory
{
	public:
	rectangularTrajectory();
	~rectangularTrajectory();
	void moveAgv(double ref_x, double ref_y, double ref_th);
	
	private:
	void broadcastTF(tf::TransformBroadcaster *tf_broadcaster,
                 	std::string device_frame,
                 	double x, double y, double z,
                 	double theta);
	short directionStatus(double direction);
	short angleDirStatus(double direction);
	ros::NodeHandle nh_;
	tf::TransformBroadcaster broadcaster;
	double x, y, th, inc_x, inc_y, inc_th, dist_x, dist_y, dist_th, dir_x, dir_y, dir_th;
};

rectangularTrajectory::rectangularTrajectory()
{
	ROS_INFO("Calling the constructor of the class rectangularTrajectory");
	x = 0.0;
	y = 0.0;
	th = M_PI/2;
	dist_x = 0.0;
	dist_y = 0.0;
	dist_th = 0.0;
	dir_x = 0.0;
	dir_y = 0.0;	
	dir_th = 0.0;
	inc_x = 0.01;
	inc_y = 0.01;
	inc_th = M_PI/200;
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
/*
double rectangularTrajectory::distanceStatus(double distance)
{
	if(distance>0.01)	return 1;
	else			return 0;
}*/

short rectangularTrajectory::directionStatus(double distance)
{
	if(distance>0)			return 1;
	else if(distance<0)		return -1;
	else				return 0;
}

short rectangularTrajectory::angleDirStatus(double distance)
{
	if(distance<M_PI && distance>0)	return 1;
	else if(distance>M_PI)		return -1;
	else if(distance<-M_PI)		return 1;
	else 				return 0;
}


void rectangularTrajectory::moveAgv(double ref_x, double ref_y, double ref_th)
{
	broadcastTF(&broadcaster,"base_link",x,y,0,th);
	broadcastTF(&broadcaster,"wheel_left",x,y,0,th);
	broadcastTF(&broadcaster,"wheel_right",x,y,0,th);
	
	ref_th = fmod(ref_th,(2*M_PI));
	dist_x = ref_x - x;
	dist_y = ref_y - y;
	dist_th = ref_th - th;
	
	while(abs(dist_x)>0.01 || abs(dist_y)>0.01 || abs(dist_th)>M_PI/200)
	{
		dir_x = directionStatus(dist_x);
		dir_y = directionStatus(dist_y);
		dir_th = angleDirStatus(dist_th);
		x += (dir_x*inc_x);
		y += (dir_y*inc_y);
		th += (dir_th*inc_th);
		th = fmod(th,(2*M_PI));
	   	broadcastTF(&broadcaster,"base_link",x,y,0,th);
		broadcastTF(&broadcaster,"wheel_left",x,y,0,th);
		broadcastTF(&broadcaster,"wheel_right",x,y,0,th);
		dist_x = ref_x - x;
		dist_y = ref_y - y;
		dist_th = ref_th - th;
		nh_.setParam("x",x);
		nh_.setParam("y",y);
		nh_.setParam("th",th);
	}
}


//main function
int main(int argc, char** argv) 
{
	ros::init(argc, argv, "rectangular_tf"); //node initialize
	rectangularTrajectory rectangularTrajectory_;
	while (ros::ok())
	{
		ros::spinOnce();
		rectangularTrajectory_.moveAgv(0,4,M_PI/2); //forward
		rectangularTrajectory_.moveAgv(0,4,M_PI);   //turn left
		rectangularTrajectory_.moveAgv(-4,4,M_PI);  //forward
		rectangularTrajectory_.moveAgv(-4,4,1.5*M_PI); //turn left
		rectangularTrajectory_.moveAgv(-4,0,1.5*M_PI); //forward
		rectangularTrajectory_.moveAgv(-4,0,2*M_PI);
		rectangularTrajectory_.moveAgv(0,0,2*M_PI);
		rectangularTrajectory_.moveAgv(0,0,M_PI/2);
		
	}//end of while
	return 0;
}//end of main
