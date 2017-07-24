// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>

#include "../include/vel_control.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>

#define FORWARD 0
#define REVERSE 1
#define RIGHT 2
#define LEFT 3
#define UP 4
#define DOWN 5
#define PITCH_UP 6
#define YAW_UP 7
#define ROLL_UP 8
#define PITCH_DOWN 9
#define YAW_DOWN 10
#define ROLL_DOWN 11
#define ARM 1
#define HUSKY 2
#define SPEED 10

namespace control_panel{
namespace husky_control {


QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ROS_INFO( "Velocity control initializing");
	int fargc = 0;
	char** fargv = (char**)malloc(sizeof(char*)*(fargc+1));
	ros::init(fargc,fargv,"husky_drive_node");
	free(fargv);
	moving_arm = false;
	moving_base = false;
	ros::start();
	start();
}

void QNode::run() {
	std::cout << "Husky Control Running" << std::endl;

	while ( ros::ok() ) {
		if(moving_base){
			ROS_INFO("Moving base %d at %f",direction,base_speed);
			ros::Duration(.5).sleep();
		}
		if(moving_arm){
			ROS_INFO("Moving arm %d at %f",direction,arm_speed);
			ros::Duration(.5).sleep();
		}
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;


	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
void QNode::move(int dir, float speed, int dev) {

	if(dev == ARM){
		moving_arm = true;
		arm_speed = speed;
		direction = dir;
	}else if(dev == HUSKY){
		moving_base = true;
		base_speed = speed;
		direction = dir;
	}
}
void QNode::stop() {

	moving_arm = false;
	moving_base = false;
	ROS_INFO("Stopped");

}



}
};


