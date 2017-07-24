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

// own includes
#include <schunk_powercube_chain/PowerCubeCtrl.h>
#include <schunk_powercube_chain/PowerCubeCtrlParams.h>
//#include <schunk_powercube_chain/Diagnostics.h>

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

////////////PCUBE INITILIZATION


 int count = 0;

int ctr;
	int i = 0;
	unsigned long serNo = 0;
	char strInitString[] = "RS232:1,9600";
	printf( "Console Test Program for PowerCube\n" );
	printf( "Version 1.0, (c) Amtec GmbH 2005\n" );
	printf( "----------------------------------\n\n ");

	ret = PCube_openDevice( &dev, strInitString );
	printf( "PCube_openDevice() returned: %d\n", ret);
	if( ret == 0 )
	{
		for( i = 1; i < MAX_MODULES; i++ )
		{
			ret = PCube_getModuleSerialNo( dev, i, &serNo );
			if( ret == 0 )
				printf( "Found module %d with SerialNo %d\n", i, serNo );
			//else printf("Mod %d Ret %d\n",i,ret);
	
		}
	}
	else
	{	
		printf( "Unable to open Device! error %d\n", ret );
		return 0;
	}
	
ret = PCube_resetModule( dev, 22 );
printf("Initilization Successful\n" );


/////////////////////////////////////

    while (ros::ok()) {
        //update joint_state
	joints();
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);
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


ret = PCube_closeDevice( dev );
    return 0;
}


int safe(void){

	float tartheta1, tartheta2, tartheta3;
	float j1temp, j2temp, j3temp, j3pos2;

	joints();

	j1temp = j1pos + omega1/20;
			
	j2temp = j2pos + omega2/20;
			
	j3pos2 = j3pos + 1.57079632679489661923;
		
	j3temp = j3pos2 + omega3/20;



	if(j1pos > 2.5 && omega1 > 0){

		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint limit error\n");

		return 0;
	}else if(j1pos < -2.5 && omega1 < 0){

		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint limit error\n");

		return 0;
	}else if(j2pos > 2.5 && omega2 > 0){

		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint limit error\n");

		return 0;
	}else if(j2pos < -2.5 && omega2 < 0){

		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint limit error\n");

		return 0;
	}else if(j3pos > 2.5 && omega3 > 0){

		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint limit error\n");

		return 0;
	}else if(j3pos < -2.35 && omega3 < 0){

		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint limit error\n");

		return 0;
	}else if(omega1 > 2 || omega1 < -2 || omega2 > 2 || omega2 < -2 || omega3 > 2 || omega3 < -2){

		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint velocity error\n");

		return 0;
	}else if(j3pos > 1.15 && omega3 < 0 && j3pos2 > 0){
	
		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint singularity error\n");

		return 0;
	}else if(j3pos2 > -0.10 && j3pos2 < 0 && omega3 > 0){
	
		ret = PCube_moveVel(dev, modJ1, 0);
		ret = PCube_moveVel(dev, modJ2, 0);
		ret = PCube_moveVel(dev, modJ3, 0);
		printf("joint singularity error\n");

		return 0;
	}else if(armX>(Xlimit_max) && armVx<0){ 	
		printf("not within xmax workspace\n");
		return 0;
	
	}else if(armX<(Xlimit_min) && armVx>0){ 	
		printf("not within xmin workspace\n");
		return 0;
	
	}else if(armY>(Ylimit_max) && armVy<0){ 	
		printf("not within ymax workspace\n");
		return 0;
	
	}else if(armY<(Ylimit_min) && armVy>0){ 	
		printf("not within ymin workspace\n");
		return 0;
	
	}else{
	
		return 1;
	}

}
void joints(void){

	//Retrieve joint positions

	ret = PCube_getPos( dev, modJ1, &j1pos);
	//ret = PCube_getPos( dev, modJ2, &j2pos);
	//ret = PCube_getPos( dev, modJ3, &j3pos);

	//printf("j1 %f   j2 %f   j3 %f\n", j1pos, j2pos, j3pos);


	//jacobian uses a slightly different zero displacement
	theta1 = j1pos;
	theta2 = j2pos - 1.57079632679489661923;
	theta3 = -j3pos;

	//Compensate for zero-displacement configuration
	j2pos = -j2pos;
	j3pos = -j3pos;

	//rollover joint values
	if(j3pos > 3.14159265){
		j3pos = j3pos - 3.14159265;
	}
	if(j3pos < -3.14159265){
		j3pos = j3pos + 3.14159265;
	}		
	if(j2pos > 3.14159265){
		j2pos = j2pos - 3.14159265;
	}
	if(j2pos < -3.14159265){
		j2pos = j2pos + 3.14159265;
	}
	if(theta2 > 3.14159265){
		theta2  = theta2  - 3.14159265;
	}
	if(theta2 < -3.14159265){
		theta2  = theta2  + 3.14159265;
	}

	//calculate forward displacement
	armX = cos(j1pos)*(b*sin(j2pos)+d*cos(j3pos-j2pos));
	armY = sin(j1pos)*(b*sin(j2pos)+d*cos(j3pos-j2pos));
	armZ = b*cos(j2pos)+d*sin(j3pos-j2pos);
//	printf("armX: %f\narmY: %f\narmZ: %f\n",armX,armY,armZ);

}



