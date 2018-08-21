// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>
#include "bcap/stdint.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>  
#include <phd/arm_msg.h>

// bCAP (Always last)
#include "bcap/bcap_client.h"
//Dont think we need this ->#define E_BUF_FULL  (0x83201483)
//Defines for the Denso Controller
#define SERVER_IP_ADDRESS "tcp:192.168.0.1"
#define SERVER_PORT_NUM 5007
#define PERIOD 100
#define AMPLITUDE 15

#define DEBUG 1
#define PTP 1
#define CP 2
#define T 6
#define SPEED 7
#define JOINT 3
#define STRING 4
#define SHUTDOWN 5
phd::arm_msg arm_cmd; //An arm message datatype
bool move_flag = false; //A flag to denote whether a new pose has been sent
char buffer[180]; //Character buffer for sprintf-ing from an arm_msg to a denso variant
int fd; //File descriptor
float dJnt[8]; //Array for joint angles
float dPos[8]; //Array for arm pose
VARIANT vntErr; //Variant to hold error codes
uint32_t hCtrl; //Controller Handle
uint32_t hRobot; //Robot Handle
uint32_t hTask; //Task Handle
uint32_t hErr; //Error Handle
uint32_t jHandle; //Joint angle handle
uint32_t pHandle; //Arm Pose handle
uint32_t hSpd; //Speed Handle
bool cp_flag = false;
bool shutdown = false;

//Callback for the arm commands on topic /arm_cmd
void arm_cb (const phd::arm_msg::ConstPtr& msg)
{
int n;
	//Parse the message to denso format. set the move flag
	//fig represents the configuration (see denso manual setting-e pg 288), 5 is an elbow up pose
//arm_cmd = *msg;
//int n=sprintf(buffer,"@P P(%f,%f,%f,%f,%f,%f,%d)",arm_cmd.x,arm_cmd.y,arm_cmd.z,arm_cmd.rx,arm_cmd.ry,arm_cmd.rz,arm_cmd.fig); 	
int fig;
	switch(msg->motion_type){
		case PTP: 
			if(msg->x < 0 && msg->y > 0) fig = 9;
			else fig = 1;
			n=sprintf(buffer,"@P P(%f,%f,%f,%f,%f,%f,%d)",msg->x,msg->y,msg->z,msg->rx,msg->ry,msg->rz,fig);
			cp_flag = false;
			if(DEBUG) ROS_INFO("Sending PTP %s",buffer);
			break;
		case CP: 
			if(msg->x < 0 && msg->y > 0) fig = 9;
			else fig = 1;
			n=sprintf(buffer,"@P P(%f,%f,%f,%f,%f,%f,%d)",msg->x,msg->y,msg->z,msg->rx,msg->ry,msg->rz,fig);
			cp_flag = true;
			if(DEBUG) ROS_INFO("Sending CP %s",buffer);
			break;
		case T: 
			n=sprintf(buffer,"@P T(%f,%f,%f,%f,%f,%f,0,1,0,%d)",msg->x,msg->y,msg->z,msg->rx,msg->ry,msg->rz,msg->fig);
			cp_flag = false;
			if(DEBUG) ROS_INFO("Sending T %s",buffer);
			break;
		case JOINT:
			n=sprintf(buffer,"@P J(%f,%f,%f,%f,%f,%f,0)",msg->j1,msg->j2,msg->j3,msg->j4,msg->j5,msg->j6);
			cp_flag = false;
			if(DEBUG) ROS_INFO("Sending %s",buffer);
			break;
		case STRING:
			n=sprintf(buffer,"%s",msg->user_string.c_str());
			cp_flag = false;
			if(DEBUG) ROS_INFO("Sending %s",buffer);
			break;
		case SHUTDOWN:
			shutdown = true;
			break;
		case SPEED:
			break;
		}
			
	
	if(msg->motion_type == SPEED){
		ROS_INFO("Setting speed to %f",msg->vel);
		HRESULT hr; //Result
		BSTR bstrCommand;  //Command
		VARIANT vSpd; //speed variants
		double *pdArray; //Array for reading joint angles
		//Set speed vlaue
		vSpd.fltVal = 100;
		bstrCommand = SysAllocString(L"EXTSPEED");
		float dnt[3];
		dnt[0] = msg->vel; //Velocity
		dnt[1] = 100.0; //Acceleration
		dnt[2] = 100.0; //Decceleration
		VARIANT vntS; //Holds the desired speed values
		vntS.vt = (VT_R4 | VT_ARRAY);
		vntS.parray = SafeArrayCreateVector(VT_R4, 0, 1);
		SafeArrayAccessData(vntS.parray, (void**)&pdArray);
		memcpy(pdArray, dnt, sizeof(dnt));
		SafeArrayUnaccessData(vntS.parray);
		hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntS, &vSpd);
		SysFreeString(bstrCommand);
		VariantClear(&vntS);
		if FAILED(hr){
		ROS_INFO("Ext Speed not set %x\n", hr);
		}
		else if(DEBUG) ROS_INFO("Ext Speed Set");

	}else move_flag = true; //Tell the loop in main a new command has come in
}

//Initialization function
HRESULT initArm(void);

int main(int argc, char **argv)
{

	//Ros initialization
	ros::init (argc, argv, "arm_node");
	ros::NodeHandle nh;
	// Create a ROS subscriber for arm commands
	ros::Subscriber sub = nh.subscribe ("arm_cmd", 1, arm_cb);

	///////////Transform Publisher
	ros::Rate loop_rate(30); //Set the rate to 30Hz
	//Joint states published on /arm_joint_states to be fused by joint_state_fusion
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("arm_joint_states", 1);
	//pose publisher
	ros::Publisher pose_pub = nh.advertise<phd::arm_msg>("arm_pose", 1);
	sensor_msgs::JointState joint_state;
	//Publish initial values
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(5);
	joint_state.position.resize(5);
	joint_state.name[0] ="j1";
	joint_state.position[0] = 0;
	joint_state.name[1] ="j2";
	joint_state.position[1] = 0;
	joint_state.name[2] ="j3";
	joint_state.position[2] = 0;
	joint_state.name[3] ="j4";
	joint_state.position[3] = 0;
	joint_state.name[4] ="j5";
	joint_state.position[4] = 0;
	joint_pub.publish(joint_state);


	//Declare Denso variables
	HRESULT hr; //Result
	BSTR bstrCommand;  //Command
	VARIANT vntParam;  //Variant for Command Parameters
	VARIANT vntResult;  //Variant to hold return values
	VARIANT vJnt, vPos; //Joint angle and Positon variants
	double *pdArray; //Array for reading joint angles
	wchar_t wstr[180]; //Wide character array for arm motion commands
	int n; //Int for swprintf function
	//Initialize Denso Arm
	hr = initArm();
	if (FAILED(hr)) ROS_ERROR("Init failed %x",hr);
	else if(DEBUG)ROS_INFO("Arm Initializaed");

	//Main Loop
	while(ros::ok() && shutdown == false){
		//Read joint angles and copy them in to dJnt
		hr = bCap_VariableGetValue(fd, jHandle, &vJnt);
		SafeArrayAccessData(vJnt.parray, (void**)&pdArray);
		memcpy(dJnt, pdArray, sizeof(dJnt));
		SafeArrayUnaccessData(vJnt.parray);
		VariantClear(&vJnt);

		//Publish joint angles converted to radians (and flipped for urdf if necessary)
		joint_state.header.stamp = ros::Time::now();
		joint_state.position[0] = 3.14/180*dJnt[0];
		joint_state.position[1] = -3.14/180*dJnt[1];
		joint_state.position[2] = -3.14/180*(dJnt[2]-90);
		joint_state.position[3] = -3.14/180*dJnt[3];
		joint_state.position[4] = -3.14/180*dJnt[4];
		joint_pub.publish(joint_state);

		//Read pose, copy to dPos, and publish
		hr = bCap_VariableGetValue(fd, pHandle, &vPos);
		SafeArrayAccessData(vPos.parray, (void**)&pdArray);
		memcpy(dPos, pdArray, sizeof(dPos));
		SafeArrayUnaccessData(vPos.parray);
		VariantClear(&vPos);
		//Create a message for publishing pose
		phd::arm_msg pose_msg;
		pose_msg.x = dPos[0];
		pose_msg.y = dPos[1];
		pose_msg.z = dPos[2];
		pose_msg.rx = dPos[3];
		pose_msg.ry = dPos[4];
		pose_msg.rz = dPos[5];
		pose_msg.fig = dPos[6];
		pose_pub.publish(pose_msg);

		//Check for arm command and publish joint states
		ros::spinOnce();
		//If a command is received
		if(move_flag){      

			move_flag = false; //Reset move flag
			//if(DEBUG) ROS_INFO("Moving to: %s",buffer);
			//Copy contents of buffer to our wide char array for the variant to hold
			n=swprintf(wstr,180, L"%s",buffer);
			vntParam.vt = VT_BSTR; //set the parameter type to b-string
			vntParam.bstrVal = SysAllocString(wstr); //set the value of the variant by pointing it to the wide string wstr
			//Create a blank b-string for the funcntion to send in the spot thats supposed to be blank
			bstrCommand = SysAllocString(L"");
			if(cp_flag){
				hr = bCap_RobotMove(fd, hRobot, 2L, vntParam, bstrCommand); //Send the command to the controller
			}else{
				hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand); //Send the command to the controller
			}
			SysFreeString(bstrCommand); //free the bstring from memory
			VariantClear(&vntParam); //clear the variant
			if (FAILED(hr)){
				ROS_ERROR("Move command failed %x",hr);
				  	hr = bCap_VariableGetValue(fd, hErr, &vntErr);
				  	if(DEBUG) ROS_INFO("Errors %x", vntErr.lVal);
				  	int eStatus = vntErr.lVal;
				  	VariantClear(&vntErr);
				/*//Sometimes it takes multiple calls to clear all the errors so its in a while loop
				int eStatus = 1;
  				while(ros::ok() && eStatus !=  0){
				  	if(DEBUG) ROS_INFO("Clearing errors");
				  	// Clear previous errors
				  	bstrCommand = SysAllocString(L"ClearError");
				  	vntParam.lVal = 0;
				  	vntParam.vt = VT_I4;
				  	hr = bCap_ControllerExecute(fd, hCtrl, bstrCommand, vntParam, &vntErr);
				  	if (FAILED(hr))
				  	  ROS_ERROR("[ClearError] command failed %x",hr);
				 	 else if(DEBUG) ROS_INFO("Errors Cleared");
				  	SysFreeString(bstrCommand);
				  	VariantClear(&vntParam);
				  	VariantClear(&vntErr);
				  	//Check for error codes
				  	hr = bCap_VariableGetValue(fd, hErr, &vntErr);
				  	if(DEBUG) ROS_INFO("Errors %x", vntErr.lVal);
				  	eStatus = vntErr.lVal;
				  	VariantClear(&vntErr);
 				}  
				hr = bCap_TaskStart(fd, hTask, 1, NULL);
				if FAILED(hr) {
  				ROS_INFO("Could not restart task: %x",hr);
				}else if(DEBUG) ROS_INFO("Restarted Task"); 
				eStatus = 1;
  				while(ros::ok() && eStatus !=  0){
				  	if(DEBUG) ROS_INFO("Clearing errors");
				  	// Clear previous errors
				  	bstrCommand = SysAllocString(L"ClearError");
				  	vntParam.lVal = 0;
				  	vntParam.vt = VT_I4;
				  	hr = bCap_ControllerExecute(fd, hCtrl, bstrCommand, vntParam, &vntErr);
				  	if (FAILED(hr))
				  	  ROS_ERROR("[ClearError] command failed %x",hr);
				 	 else if(DEBUG) ROS_INFO("Errors Cleared");
				  	SysFreeString(bstrCommand);
				  	VariantClear(&vntParam);
				  	VariantClear(&vntErr);
				  	//Check for error codes
				  	hr = bCap_VariableGetValue(fd, hErr, &vntErr);
				  	if(DEBUG) ROS_INFO("Errors %x", vntErr.lVal);
				  	eStatus = vntErr.lVal;
				  	VariantClear(&vntErr);
 				}   
				bstrCommand = SysAllocString(L"Motor");
				vntParam.vt = VT_I2;
				vntParam.iVal = 1;
				hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
				SysFreeString(bstrCommand);
				VariantClear(&vntParam);
				if FAILED(hr){
				ROS_ERROR("Motor Restart %x\n", hr);
				return (hr);
				}
				else if(DEBUG) ROS_INFO("Motor On");*/
			}
			else if(DEBUG) ROS_INFO("Moved");
		} 
	}

	//Shut down sequence
	if(DEBUG) ROS_INFO("Shutting Down");
	
	//Turn off motor
	bstrCommand = SysAllocString(L"Motor");
	vntParam.iVal = 0;
	vntParam.vt = VT_I2;
	hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
	SysFreeString(bstrCommand);
	VariantClear(&vntParam);
	if FAILED(hr) {
		ROS_INFO("Motors Not off: %x",hr);
	return (hr);
	}else if(DEBUG) ROS_INFO("Motor Off");
	//* Release robot handle *
	bCap_RobotRelease(fd, &hRobot);
	//* Release controller handle *
	bCap_ControllerDisconnect(fd, &hCtrl);
	//* Stop b-CAP service (Very important in UDP/IP connection) *
	bCap_ServiceStop(fd);
	//* Close socket *
	bCap_Close_Client(&fd);

}

//Denso initialization function
HRESULT initArm(void){
  
  HRESULT hr; //Result
  BSTR bstrCommand;  //Command
  VARIANT vntParam;  //Command Parameters
  VARIANT vntResult;  //Variant to hold return values
  int32_t eStatus;  //Error code
  VARIANT vJnt, vSpd, vPos; //Joint, Position, and speed variants
  double *pdArray; //Array for reading joint angles
  /* Init socket */
  hr = bCap_Open_Client(SERVER_IP_ADDRESS, SERVER_PORT_NUM, 0, &fd);
  if FAILED(hr) return (hr);
  else if(DEBUG) ROS_INFO("Socket Created");
  /* Start b-CAP service */
  hr = bCap_ServiceStart(fd, NULL);
  if FAILED(hr) return (hr);
  else if(DEBUG) ROS_INFO("b-CAP started");
  /* Get controller handle */
  BSTR bstrName, bstrProv, bstrMachine, bstrOpt;
  bstrName = SysAllocString(L"");
  bstrProv = SysAllocString(L"CaoProv.DENSO.VRC");
  bstrMachine = SysAllocString(L"localhost");
  bstrOpt = SysAllocString(L"");
  hr = bCap_ControllerConnect(fd, bstrName, bstrProv, bstrMachine, bstrOpt, &hCtrl);
  SysFreeString(bstrName);
  SysFreeString(bstrProv);
  SysFreeString(bstrMachine);
  SysFreeString(bstrOpt);
  if FAILED(hr) return (hr);
  else if(DEBUG) ROS_INFO("Received controller handle");

  /* Get robot handle */
  BSTR bstrRobotName, bstrRobotOpt;
  bstrRobotName = SysAllocString(L"Arm");
  bstrRobotOpt = SysAllocString(L"");
  hr = bCap_ControllerGetRobot(fd, hCtrl, bstrRobotName, bstrRobotOpt, &hRobot);
  SysFreeString(bstrRobotName);
  SysFreeString(bstrRobotOpt);
  if FAILED(hr) return (hr);
  else if(DEBUG) ROS_INFO("Received robot handle");
  
 
  //Get error flag handle
  BSTR nullstr;
  bstrCommand = SysAllocString(L"@ERROR_CODE");
  nullstr = SysAllocString(L"");
  hr = bCap_ControllerGetVariable(fd, hRobot, bstrCommand, nullstr, &hErr);
  SysFreeString(bstrCommand);
  SysFreeString(nullstr);
  if FAILED(hr){
  	ROS_INFO("Could not get handle for errors %x", hr);
   return (hr);
   }
  else {
  //Read error value
  vntResult.vt = VT_I4;
  hr = bCap_VariableGetValue(fd, hErr, &vntErr);
  if(DEBUG) ROS_INFO("Errors %d", vntErr.lVal);
  eStatus = vntErr.lVal;
  VariantClear(&vntErr);
  }
  //Sometimes it takes multiple calls to clear all the errors so its in a while loop
  while(ros::ok() && eStatus !=  0){
  	if(DEBUG) ROS_INFO("Clearing errors");
  	// Clear previous errors
  	bstrCommand = SysAllocString(L"ClearError");
  	vntParam.lVal = 0;
  	vntParam.vt = VT_I4;
  	hr = bCap_ControllerExecute(fd, hCtrl, bstrCommand, vntParam, &vntErr);
  	if (FAILED(hr))
  	  ROS_ERROR("[ClearError] command failed %x",hr);
 	 else if(DEBUG) ROS_INFO("Errors Cleared");
  	SysFreeString(bstrCommand);
  	VariantClear(&vntParam);
  	VariantClear(&vntErr);
  	//Check for error codes
  	hr = bCap_VariableGetValue(fd, hErr, &vntErr);
  	if(DEBUG) ROS_INFO("Errors %d", vntErr.lVal);
  	eStatus = vntErr.lVal;
  	VariantClear(&vntErr);
  }
  
  //Get ROBSLAVE task handle
  bstrCommand = SysAllocString(L"ROBSLAVE");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"1");
  hr = bCap_ControllerGetTask(fd, hCtrl, bstrCommand, NULL, &hTask);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr) {
  ROS_INFO("Could not find task handle %x", hr);
  return (hr);
  }else if(DEBUG) ROS_INFO("Got Handle: %d",hTask);
  
  //Wait a moment then begin ROBSLAVE task
  ros::Duration(1).sleep();
  hr = bCap_TaskStart(fd, hTask, 1, NULL);
  if FAILED(hr) {
  ROS_INFO("Could not start task: %x",hr);
  return (hr);
  }else if(DEBUG) ROS_INFO("Started Task");
  VARIANT vntPose;
  
  /* Get handle for current angle */
  bstrCommand = SysAllocString(L"@CURRENT_ANGLE");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"");
  nullstr = SysAllocString(L"");
  hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &jHandle);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  ROS_ERROR("Could not get handle %x", hr);
  return (hr);
  }
  //Read joint values
  hr = bCap_VariableGetValue(fd, jHandle, &vJnt);
  SafeArrayAccessData(vJnt.parray, (void**)&pdArray);
  memcpy(dJnt, pdArray, sizeof(dJnt));
  SafeArrayUnaccessData(vJnt.parray);
  VariantClear(&vJnt);
  if(DEBUG) ROS_INFO("Joint Read RET %x, Val %f -- %f -- %f -- %f -- %f -- %f Fig: %f",hr, dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5], dJnt[6]);
  
  /* Get handle for current position */
  bstrCommand = SysAllocString(L"@CURRENT_POSITION");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"");
  nullstr = SysAllocString(L"");
  hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &pHandle);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("Could not get handle %x\n", hr);
  return (hr);
  }
  //Read pose
  hr = bCap_VariableGetValue(fd, pHandle, &vPos);
  SafeArrayAccessData(vPos.parray, (void**)&pdArray);
  memcpy(dPos, pdArray, sizeof(dPos));
  SafeArrayUnaccessData(vPos.parray);
  VariantClear(&vPos);
  if(DEBUG) ROS_INFO("Pose Read RET %x, Val %f -- %f -- %f -- %f -- %f -- %f Fig: %f",hr, dPos[0], dPos[1], dPos[2], dPos[3], dPos[4], dPos[5], dPos[6]);
  
  //Get speed handle
  bstrCommand = SysAllocString(L"@SPEED");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"");
  nullstr = SysAllocString(L"");
  hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  ROS_ERROR("Could not get handle for speed %x\n", hr);
  return (hr);
  }
  else {
  //Read speed value
  vSpd.vt = VT_R4;
  hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
  if(DEBUG) ROS_INFO("Speed %f", vSpd.fltVal);
  VariantClear(&vSpd);
  }
  //Set speed vlaue
  vSpd.fltVal = 100;
  bstrCommand = SysAllocString(L"EXTSPEED");
  float dnt[3];
  dnt[0] = 10.0; //Velocity
  dnt[1] = 100.0; //Acceleration
  dnt[2] = 100.0; //Decceleration
  VARIANT vntS; //Holds the desired speed values
  vntS.vt = (VT_R4 | VT_ARRAY);
  vntS.parray = SafeArrayCreateVector(VT_R4, 0, 1);
  SafeArrayAccessData(vntS.parray, (void**)&pdArray);
  memcpy(pdArray, dnt, sizeof(dnt));
  SafeArrayUnaccessData(vntS.parray);
  hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntS, &vSpd);
  SysFreeString(bstrCommand);
  VariantClear(&vntS);
  VariantClear(&vSpd);
  if FAILED(hr){
  ROS_INFO("Ext Speed not set %x\n", hr);
  return (hr);
  }
  else if(DEBUG) ROS_INFO("Ext Speed Set");

  //Turn Motor ON
  bstrCommand = SysAllocString(L"Motor");
  vntParam.vt = VT_I2;
  vntParam.iVal = 1;
  hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  ROS_ERROR("FAIL %x\n", hr);
  return (hr);
  }
  else if(DEBUG) ROS_INFO("Motor On");

  //Home the arm
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"@P J(0,0,90,0,0,0,0)");
  bstrCommand = SysAllocString(L"");
  hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("FAIL %x\n", hr);
  return (hr);
  }
  else if(DEBUG) ROS_INFO("Homed");
  return(hr);
};


