#include <string>
#include <tf/transform_broadcaster.h>


// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>


// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

//Function Declarations
int safe(void);
void joints(void);

//Global Variables
float omega1, omega2, omega3;
float j1pos, j2pos, j3pos;
float theta1, theta2, theta3;
float armX, armY, armZ;
float armVx, armVy, armVz;
float a = 0;
float b = 0.228;
float d = 0.3125;
int modJ1 = 22;
int modJ2 = 22;
int modJ3 = 22;
int vel = 1.5;
int acc = 2; 
int ret = 0;

float Xlimit_max = 0.45;
float Xlimit_min = 0.2;
float Ylimit_max = 0.30;
float Ylimit_min = -0.30;
int dev = 0;

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    //Comment this when using husky
	double blw=0, brw=0, flw=0, frw=0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";




/////////////////////////////////////

    while (ros::ok()) {
        //update joint_state
	joints();
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(13);
        joint_state.position.resize(13);
        joint_state.name[0] ="joint_back_left_wheel";
        joint_state.position[0] = blw;
        joint_state.name[1] ="joint_back_right_wheel";
        joint_state.position[1] = brw;
        joint_state.name[2] ="joint_front_left_wheel";
        joint_state.position[2] = flw;
        joint_state.name[3] ="joint_front_right_wheel";
        joint_state.position[3] = frw;
        joint_state.name[4] ="cube_5_joint";
        joint_state.position[4] = j1pos;
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
        joint_state.name[10] ="flange";
        joint_state.position[10] = 0;


        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}


int safe(void){

	
}
void joints(void){


}



