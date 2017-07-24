// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>
#include "bcap/stdint.h"
//#include <atlbase.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>  
#include <wrock/arm_msg.h>

// bCAP (Always last)
#include "bcap/bcap_client.h"

#define E_BUF_FULL  (0x83201483)

#define SERVER_IP_ADDRESS "tcp:192.168.0.1"
#define SERVER_PORT_NUM 5007
#define PERIOD 100
#define AMPLITUDE 15

wrock::arm_msg arm_cmd;
bool move_flag = false;
char buffer[80];
int fd;
float dJnt[8];
VARIANT vntErr; //Variant to hold error codes
uint32_t hCtrl; //Controller Handle
uint32_t hRobot; //Robot Handle
uint32_t hTask; //Task Handle
uint32_t hErr; //Error Handle
uint32_t jHandle; //Joint angle handle

void 
arm_cb (const wrock::arm_msg::ConstPtr& msg)
{
  //Parse the message to denso format. set the move flag
  //5 represents the configuration (see denso manual setting-e pg 288)
  arm_cmd = *msg;
  int n=sprintf(buffer,"@P P(%f,%f,%f,%f,%f,%f,5)",arm_cmd.x,arm_cmd.y,arm_cmd.z,arm_cmd.rx,arm_cmd.ry,arm_cmd.rz);
  move_flag = true;

}

HRESULT initArm(void);

int main(int argc, char **argv)
{
  ros::init (argc, argv, "arm_node");
  ros::NodeHandle nh, nh_private("~");
  // Create a ROS subscriber for arm commands
  ros::Subscriber sub = nh.subscribe ("arm_cmd", 1, arm_cb);

  ///////////Transform Publisher
  ros::Rate loop_rate(30);
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_msgs::JointState joint_state;    

  tf::TransformBroadcaster broadcaster;
  //Comment this when using husky
  double blw=0, brw=0, flw=0, frw=0;

  //////////////////Publish initial values
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
  joint_state.position[4] = 0;
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
  //Publish odometry messages
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = 0;
  odom_trans.transform.translation.y = 0;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
  broadcaster.sendTransform(odom_trans);
  

  //Declare Denso variables
  HRESULT hr; //Result
  BSTR bstrCommand;  //Command
  VARIANT vntParam;  //Command Parameters
  VARIANT vntResult;  //Variant to hold return values
  VARIANT vJnt; //Joint angle variants
  double *pdArray; //Array for reading joint angles
  wchar_t wstr[100]; //Wide character array for arm motion commands
  int n; //Int for swprintf function
  
  //Initialize Denso Arm
  hr = initArm();
  if (FAILED(hr))
    ROS_ERROR("Init failed %x",hr);
    
  while(ros::ok()){
  
  //Read joint angles
  hr = bCap_VariableGetValue(fd, jHandle, &vJnt);
  SafeArrayAccessData(vJnt.parray, (void**)&pdArray);
  memcpy(dJnt, pdArray, sizeof(dJnt));
  SafeArrayUnaccessData(vJnt.parray);
  VariantClear(&vJnt);
  //Publish joint angles
  joint_state.header.stamp = ros::Time::now();
  joint_state.name[5] ="j1";
  joint_state.position[5] = 3.14/180*dJnt[0];
  joint_state.name[6] ="j2";
  joint_state.position[6] = -3.14/180*dJnt[1];
  joint_state.name[7] ="j3";
  joint_state.position[7] = -3.14/180*(dJnt[2]-90);
  joint_state.name[8] ="j4";
  joint_state.position[8] = -3.14/180*dJnt[3];
  joint_state.name[9] ="j5";
  joint_state.position[9] = -3.14/180*dJnt[4];
  joint_pub.publish(joint_state);
  
  //Listen for arm command
  ros::spinOnce();
  //If a command is received
  if(move_flag){      

    move_flag = false;
    ROS_INFO("Moving to: %s",buffer);
    n=swprintf(wstr,80, L"%s",buffer);
    vntParam.vt = VT_BSTR;
    vntParam.bstrVal = SysAllocString(wstr);
    bstrCommand = SysAllocString(L"");
    hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
    SysFreeString(bstrCommand);
    VariantClear(&vntParam);
    if (FAILED(hr))
      ROS_ERROR("Move command failed %x",hr);
      else ROS_INFO("Moved");
    } 
}


ROS_INFO("Shutting Down");
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
}else ROS_INFO("Motor Off");
/* Release robot handle */
bCap_RobotRelease(fd, &hRobot);
/* Release controller handle */
bCap_ControllerDisconnect(fd, &hCtrl);
/* Stop b-CAP service (Very important in UDP/IP connection) */
bCap_ServiceStop(fd);
/* Close socket */
bCap_Close_Client(&fd);

}

HRESULT initArm(void){
  
  HRESULT hr; //Result
  BSTR bstrCommand;  //Command
  VARIANT vntParam;  //Command Parameters
  VARIANT vntResult;  //Variant to hold return values
  int32_t eStatus;  //Error code
  uint32_t hSpd; //Speed Handle
  VARIANT vJnt, vSpd; //Joint and speed variants
  double *pdArray; //Array for reading joint angles
  wchar_t wstr[100]; //Wide character array for arm motion commands
    
  /* Init socket */
  hr = bCap_Open_Client(SERVER_IP_ADDRESS, SERVER_PORT_NUM, 0, &fd);
  if FAILED(hr) return (hr);
  else ROS_INFO("Socket Created");
  /* Start b-CAP service */
  hr = bCap_ServiceStart(fd, NULL);
  if FAILED(hr) return (hr);
  else ROS_INFO("b-CAP started");
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
  else ROS_INFO("Received controller handle");

  /* Get robot handle */
  BSTR bstrRobotName, bstrRobotOpt;
  bstrRobotName = SysAllocString(L"Arm");
  bstrRobotOpt = SysAllocString(L"");
  hr = bCap_ControllerGetRobot(fd, hCtrl, bstrRobotName, bstrRobotOpt, &hRobot);
  SysFreeString(bstrRobotName);
  SysFreeString(bstrRobotOpt);
  if FAILED(hr) return (hr);
  else ROS_INFO("Received robot handle");
  
 
  //Get error flag handle
  BSTR nullstr;
  bstrCommand = SysAllocString(L"@ERROR_CODE");
  nullstr = SysAllocString(L"");
  hr = bCap_ControllerGetVariable(fd, hRobot, bstrCommand, nullstr, &hErr);
  SysFreeString(bstrCommand);
  SysFreeString(nullstr);
  if FAILED(hr){
  printf("Could not get handle for errors %x\n", hr);
   return (hr);
   }
  else {
  //Read error value
  vntResult.vt = VT_I4;
  hr = bCap_VariableGetValue(fd, hErr, &vntErr);
  ROS_INFO("Errors %d", vntErr.lVal);
  eStatus = vntErr.lVal;
  VariantClear(&vntErr);
  }
  //Sometimes it takes multiple calls to clear all the errors so its in a while loop
  while(ros::ok() && eStatus !=  0){
  ROS_INFO("Clearing errors");
  // Clear previous errors
  bstrCommand = SysAllocString(L"ClearError");
  vntParam.lVal = 0;
  vntParam.vt = VT_I4;
  hr = bCap_ControllerExecute(fd, hCtrl, bstrCommand, vntParam, &vntErr);
  if (FAILED(hr))
    ROS_ERROR("[ClearError] command failed %x",hr);
  else ROS_INFO("Errors Cleared");
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  VariantClear(&vntErr);
  //Check for error codes
  hr = bCap_VariableGetValue(fd, hErr, &vntErr);
  ROS_INFO("Errors %d", vntErr.lVal);
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
  }else ROS_INFO("Got Handle: %d",hTask);
  
  //Wait a moment then begin ROBSLAVE task
  ros::Duration(1).sleep();
  hr = bCap_TaskStart(fd, hTask, 1, NULL);
  if FAILED(hr) {
  ROS_INFO("Could not start task: %x",hr);
  return (hr);
  }else ROS_INFO("Started Task");
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
  printf("Could not get handle %x\n", hr);
   return (hr);
   }
   //Read joint values
  hr = bCap_VariableGetValue(fd, jHandle, &vJnt);
  SafeArrayAccessData(vJnt.parray, (void**)&pdArray);
  memcpy(dJnt, pdArray, sizeof(dJnt));
  SafeArrayUnaccessData(vJnt.parray);
  VariantClear(&vJnt);
  ROS_INFO("RET %x, Val %f -- %f -- %f -- %f -- %f -- %f Fig: %f",hr, dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5], dJnt[6]);
  
  //Get speed handle
  bstrCommand = SysAllocString(L"@SPEED");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"");
  nullstr = SysAllocString(L"");
  hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("Could not get handle for speed %x\n", hr);
   return (hr);
   }
  else {
  //Read speed value
  vSpd.vt = VT_R4;
  hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
  ROS_INFO("Speed %f", vSpd.fltVal);
  VariantClear(&vSpd);
  }
  //Set speed vlaue
  vSpd.fltVal = 100;
  bstrCommand = SysAllocString(L"EXTSPEED");
  float dnt[3];
  dnt[0] = 50.0; //Velocity
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
  else ROS_INFO("Ext Speed Set");

  //Turn Motor ON
  bstrCommand = SysAllocString(L"Motor");
  vntParam.vt = VT_I2;
  vntParam.iVal = 1;
  hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("FAIL %x\n", hr);
   return (hr);
   }
  else ROS_INFO("Motor On");

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
  else ROS_INFO("Homed");
return(hr);
};


