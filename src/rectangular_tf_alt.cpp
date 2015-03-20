#include<string>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Joy.h>
#include<tf/transform_broadcaster.h>
#include<boost/bind.hpp>
#include<boost/ref.hpp>
#include<math.h>

void timerCallback(ros::NodeHandle &node_handle, const ros::TimerEvent& e)
{
	int state;	
	node_handle.getParam("state",state);
	state = (state++)%2;
	node_handle.setParam("state",state);
	ROS_INFO("timer triggered");
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
	double th = M_PI/2;
	double vx, vy, wth, dt, delta_x, delta_y, delta_th;
	int state;

	ros::init(argc, argv, "rectangular_tf"); //node initialize
	ros::NodeHandle nh_; //node handle object
	ros::Timer timer = nh_.createTimer(ros::Duration(1.0), boost::bind(timerCallback,boost::ref(nh_),_1));
	tf::TransformBroadcaster broadcaster; //tf broadcaer stobject
	//time variables
	ros::Time current_time, prev_time, duration;
	prev_time = ros::Time::now();
	//ros::Rate loop_rate(50);
	
	while (ros::ok())
	{
		ros::spinOnce();
/*		broadcastTF(&broadcaster,"base_link",x,y,0,th);
       	 	broadcastTF(&broadcaster,"wheel_left",x,y,0,th);
	        broadcastTF(&broadcaster,"wheel_right",x,y,0,th);
*/		nh_.getParam("state",state);

		while(state == 0)
		{
			current_time = ros::Time::now(); //set current time
			vx = 4; //linear velocity in ROBOT frame
			vy = 0;
			wth = 0; //angular velocity of the ROBOT
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
		

		while(state == 1)
		{
			current_time = ros::Time::now(); //set current time
			vx = 0;
			vy = 0;
			wth = 1.57079632679; //angular velocity of the ROBOT
			dt = (current_time - prev_time).toSec(); //time differential
			delta_x = 0; //change in x axis in world frame
			delta_y = 0; //change in y axis in world frame
			delta_th = wth*dt; //change in the orientation
			//if(delta_th>M_PI/2 || delta_th >= (M_PI/2)-(M_PI/10)) delta_th=M_PI/2;
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
	}//end of while
	return 0;
}//end of main
