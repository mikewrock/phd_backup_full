// ROS includes
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
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
#define DEBUG 1
//Global Variables
float j1pos;
int modJ1 = 22;
int vel = 1.5;
int acc = 2; 
int ret = 0;
int dev = 0;
//The callback function to listen for cube commands and send them to the motor driver
void cmdCallback(const phd::cube_msg::ConstPtr& msg)
{
  //If pose is set, move to j1 position, otherwise move at j1 velocity
  if(DEBUG) ROS_INFO("I heard: [%f %f %f]", msg->j1, msg->vel, msg->acc);
	if(msg->pose){
		//Pass the device, module, position, velocity and acceleration for the ramp movement profile
		ret = PCube_moveRamp(dev, modJ1, msg->j1 , msg->vel, msg->acc);
  }
	else{
		//this doesn't have a use at the moment, so it's commented out		
		//ret = PCube_moveVel(dev, modJ1, msg->j1;);
	}
}

//Main function, some of this code may appear odd, as the original node was designed for multiple power cubes. Some of that code has been kept to allow for adding more cubes to the robot
int main(int argc, char** argv)
{

	//Ros initilization
	ros::init(argc, argv, "cube_move");
	ros::NodeHandle n;
	//Publish joint positions to /cube_joint_states
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("cube_joint_states", 10, true);
	//Listen for commanda on /joint_cmd
	ros::Subscriber joint_cmd_sub = n.subscribe("joint_cmd", 1000, cmdCallback);
	//Variable to publish joint states
	sensor_msgs::JointState joint_state;  

	//Powercube Initialization
	int i = 0; //counter for searching for powercubes
	unsigned long serNo = 0; //holder for powercube serial number
	//A custom version of m5wapi is used to allow for USB-Serial adapters, it has overwritten the serial part of the init string
	//When /dev contains ttyusb1, the init string should be RS232:0 (use n-1 for usb #). 9600 is the baud rate
	char strInitString[] = "RS232:0,9600";
	if(DEBUG) ROS_INFO( "Booting up PowerCube" );
	if(DEBUG) ROS_INFO( "----------------------------------\n");
	//Connect to device
	ret = PCube_openDevice( &dev, strInitString );
	if(DEBUG) ROS_INFO( "PCube_openDevice() returned: %d", ret);
	if( ret == 0 ) //If the connection was made
	{	//Loop through all possible module positions and display their serial number
		for( i = 1; i < MAX_MODULES; i++ )
		{	
			//Check to see if the module exists (ret == 0)
			ret = PCube_getModuleSerialNo( dev, i, &serNo );
			if( ret == 0 )
				if(DEBUG) ROS_INFO( "Found module %d with SerialNo %lu", i, serNo );
			}
		}
	else
	{	
		//If unable to connect, display the error and end the node
		if(DEBUG) ROS_INFO( "Unable to open Device! error %d", ret );
		return 0;
	}
	//The only module we use is 22, so it's hard-coded
	//Reset the module (clears errors and e-stops
	ret = PCube_resetModule( dev, 22 );
	if(DEBUG) ROS_INFO("Initilization Successful\n" );

	//Home the modules
	if(DEBUG) ROS_INFO("Homing Modules");
	ret = PCube_homeModule( dev, 22 ); 
	//Variables for homing the module
	unsigned long modState;
	int intState =0;
	//while loop to wait until cube is homed
	while(intState != 2){
		//Read the module state to check if its homed
		ret = PCube_getModuleState( dev, 22, &modState);
		intState = modState & 15; //bit mask for homed status
	}
	if(DEBUG) ROS_INFO("Cubes Ready");
	//Though the cube should be at 0, its safest to re-measure the position before publishing it
	ret = PCube_getPos( dev, modJ1, &j1pos);

	//initial joint_state
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(1);
	joint_state.position.resize(1);
	joint_state.name[0] ="cube_joint";
	joint_state.position[0] = j1pos;

	//move to initial pose
	ret = PCube_moveRamp(dev, modJ1, -1.57, vel, acc);

	while(ros::ok()){  
		//Get the cube position
		ret = PCube_getPos( dev, modJ1, &j1pos);
		// update transform
		joint_state.header.stamp = ros::Time::now();
		//If the joint has moved, update and publish the values
		if(joint_state.position[0] != j1pos){  
			joint_state.position[0] = j1pos;
			//send the joint state 
			joint_pub.publish(joint_state);
			}
		//check for callbacks and publish if necessary
		ros::spinOnce();
	}

	//close powercube connection
	ret = PCube_closeDevice( dev );
	return 0;
}




