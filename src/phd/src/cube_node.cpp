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


// own includes
#include <schunk_powercube_chain/PowerCubeCtrl.h>
#include <schunk_powercube_chain/PowerCubeCtrlParams.h>
#include <phd/cube_msg.h>


//Global Variables
float j1pos;
int modJ1 = 22;
int vel = 1.5;
int acc = 2; 
int ret = 0;
int dev = 0;

void cmdCallback(const phd::cube_msg::ConstPtr& msg)
{
  //If pose is set, move to j1 position, otherwise move at j1 velocity
  ROS_INFO("I heard: [%f %f %f]", msg->j1, msg->vel, msg->acc);
	if(msg->pose){
		ROS_INFO("Moving");
		ret = PCube_moveRamp(dev, modJ1, msg->j1 , msg->vel, msg->acc);
  }
	else{
		//this doesn't have a use at the moment, so it's commented out		
		//ret = PCube_moveVel(dev, modJ1, msg->j1;);
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube_move");

    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("cube_joint_states", 10, true);
    ros::Subscriber joint_cmd_sub = n.subscribe("joint_cmd", 1000, cmdCallback);
    tf::TransformBroadcaster broadcaster;
    sensor_msgs::JointState joint_state;  

    ////////////PCUBE INITILIZATION///////////////////////////////////////
    int count = 0;
    int ctr;
    int i = 0;
    unsigned long serNo = 0;
    //usb # is n+1 where dev is ttyusbn
    char strInitString[] = "RS232:0,9600";
    printf( "Booting up PowerCube\n" );
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
    //////////////////////////////////////////////////////

    //Home the modules
    ROS_INFO("Homing Modules");
    ret = PCube_homeModule( dev, 22 ); 
    unsigned long modState;
    int intState =0;
    //while loop to wait until cube is homed
    while(intState != 2){
	    //printf("Homing module 22: %d\n", ret);
	    ret = PCube_getModuleState( dev, 22, &modState);
	    intState = modState & 15; //bit mask for homed status
    }
    printf("Cubes Ready\n");
    ret = PCube_getPos( dev, modJ1, &j1pos);

        //initial joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] ="cube_5_joint";
        joint_state.position[0] = j1pos;

//move to initial pose
ret = PCube_moveRamp(dev, modJ1, -.785, vel, acc);
        
while(ros::ok()){  
  //Get the cube position
  ret = PCube_getPos( dev, modJ1, &j1pos);
  // update transform
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = j1pos;
  //send the joint state 
  joint_pub.publish(joint_state);
  //check for callbacks
  ros::spinOnce();

}

//close powercube connection
ret = PCube_closeDevice( dev );
return 0;
}




