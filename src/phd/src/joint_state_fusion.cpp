#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>

// ROS includes
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>



//Global Variables
sensor_msgs::JointState joint_state;  

//Callback for /cube_joint_states topic
void cubeCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//See main function for names of joints	
	joint_state.position[4] = msg->position[0];
}
//Callback for /arm_joint_states topic
void armCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//See main function for names of joints
	joint_state.position[5] = msg->position[0];
	joint_state.position[6] = msg->position[1];
	joint_state.position[7] = msg->position[2];
	joint_state.position[8] = msg->position[3];
	joint_state.position[9] = msg->position[4];
}
//Callback for /joint_states topic (husky)
void huskyCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//See main function for names of joints	
	joint_state.position[0] = msg->position[0];
	joint_state.position[1] = msg->position[1];
	joint_state.position[2] = msg->position[2];
	joint_state.position[3] = msg->position[3];
}

int main(int argc, char** argv)
{
	//Ros initialization
	ros::init(argc, argv, "joint_fusion_node");
	ros::NodeHandle n;
	//Subscribers for the other joint_state publishers
	ros::Subscriber cube_sub = n.subscribe("cube_joint_states", 1, cubeCallback);
	ros::Subscriber arm_sub = n.subscribe("arm_joint_states", 1, armCallback);
	ros::Subscriber husky_sub = n.subscribe("joint_states", 1, huskyCallback);
	//Publisher for joint_states
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("fused_joint_states", 10, true);
	//Publish initial values
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(13);
	joint_state.position.resize(13);
	joint_state.name[0] ="joint_back_left_wheel";
	joint_state.position[0] = 0;
	joint_state.name[1] ="joint_back_right_wheel";
	joint_state.position[1] = 0;
	joint_state.name[2] ="joint_front_left_wheel";
	joint_state.position[2] = 0;
	joint_state.name[3] ="joint_front_right_wheel";
	joint_state.position[3] = 0;
	joint_state.name[4] ="cube_joint";
	joint_state.position[4] = -1.57;
	joint_state.name[5] ="j1";
	joint_state.position[5] = 0;
	joint_state.name[6] ="j2";
	joint_state.position[6] = 0;
	joint_state.name[7] ="j3";
	joint_state.position[7] = 0;
	joint_state.name[8] ="j4";
	joint_state.position[8] = 0;
	joint_state.name[9] ="j5";
	joint_state.position[9] = 0;
	joint_pub.publish(joint_state);
	//Spin to publish initial values and check if new ones have already come in
	ros::spinOnce();
	while(ros::ok()){
		//Publish joint values
		joint_pub.publish(joint_state);
		//Spin to publish and execute callbacks
		ros::spinOnce();
	}
	return 0;
}




