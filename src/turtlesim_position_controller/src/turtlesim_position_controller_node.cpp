// Includes
#include <ros/ros.h>
#include <math.h>						// To calculate errors
#include <turtlesim/Pose.h>				// To read current position
#include <geometry_msgs/Twist.h>		// To send velocity command
#include <geometry_msgs/Pose2D.h>		// To get desired position command


// Function declarations
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg);
float GetErrorLin(turtlesim::Pose curpose, geometry_msgs::Pose2D despose);
float GetErrorAng(turtlesim::Pose curpose, geometry_msgs::Pose2D despose);

// Global variables
bool STOP = true;				// to hold stop flag, wait till first command given
turtlesim::Pose CurPose;		// current position holder
geometry_msgs::Pose2D DesPose;	// desired position holder

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TurtlesimPositionController_pubsub");	// connect to roscore
    ros::NodeHandle n;												// node object

    // register sub to get desired position/pose commands
    ros::Subscriber ComPose_sub = n.subscribe("/turtle1/PositionCommand", 5, ComPoseCallback);
    // register sub to get current position/pose
    ros::Subscriber CurPose_sub = n.subscribe("/turtle1/pose", 5, CurPoseCallback);
    // register pub to send twist velocity (cmd_vel)
    ros::Publisher Twist_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

    ros::Rate loop_rate(10);							// freq to run loops in
    float ErrorLin = 0;
    float ErrorAng = 0;
    geometry_msgs::Twist CmdVel;

    ROS_INFO("Ready to send position commands");		// let user know we are ready and set
    while(ros::ok() && n.ok()) {						// while ros and node are okay
    	ros::spinOnce();
    	if(STOP == false) {								// and no stop command
    		ErrorLin = GetErrorLin(CurPose, DesPose);
    		ErrorAng = GetErrorAng(CurPose, DesPose);
    		printf("Error linear: %f, Error angular: %f\n", ErrorLin, ErrorAng);

    		if (ErrorLin > 0) {							// check to see if error is positive
    			CmdVel.linear.x =  0.2 * ErrorLin;		// multiply by linear P  for control signal	
    		} else {
    			CmdVel.linear.x = 0;
    		}
    		
    		CmdVel.angular.z = 0.5 * ErrorAng;			// multiply by linear P for control signal
    		Twist_pub.publish(CmdVel);
    	} else {
    		printf("Waiting...\n");
    	}
    	loop_rate.sleep();								// sleep to maintain loop rate
    }
    ros::spin();
}

// Callback to send new described Pose msgs
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
	STOP = false;					// start loop
	DesPose.x = msg -> x;			// copy msg to variable to use elsewhere
	DesPose.y = msg -> y;
	return;
}

// Callback to send new current Pose msgs
void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg) {
	CurPose.x = msg -> x;
	CurPose.y = msg -> y;
	ROS_INFO("X position: %f, Y position: %f\n", CurPose.x, CurPose.y);
	CurPose.theta = msg -> theta;	// copy msg to variable to use elsewhere
	return;
}

// Function to get angular error between  
//facing direction  of the turtle and the direction to desired pose
float GetErrorAng(turtlesim::Pose curpose, geometry_msgs::Pose2D despose) {
	// create error vector
	float Ex = despose.x - curpose.x;			// Error X component
	float Ey = despose.y - curpose.y;			// Error Y component

	// get desired angle
	float dest =  atan2f(Ey, Ex); 				// use float version to get arc tangent

	// get angle error
	float Et = dest - curpose.theta;

	//~ ROS_INFO("Ex: %f, Ey: %f, Et: %f", Ex, Ey, Et);
    return Et;
}	

// Function to get linear error from the turtles perspective
// Error only along turtle X axis
float GetErrorLin(turtlesim::Pose curpose, geometry_msgs::Pose2D despose) {
	// create error vector
	float Ex = despose.x - curpose.x;			// Error X component
	float Ey = despose.y - curpose.y; 			// Error Y component
	float Et = GetErrorAng(curpose, despose);	// get angle between vectors

	// project error onto turtle x axis
	float Etx =  hypotf(Ex, Ey)*cos(Et);

	return Etx;
}