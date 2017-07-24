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
void 
arm_cb (const wrock::arm_msg::ConstPtr& msg)
{
ROS_INFO("GOT A COMMAD");
  arm_cmd = *msg;
  int n=sprintf(buffer,"@P P(%f,%f,%f,%f,%f,%f,5)",arm_cmd.x,arm_cmd.y,arm_cmd.z,arm_cmd.rx,arm_cmd.ry,arm_cmd.rz);
  move_flag = true;

}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "arm_node");
  ros::NodeHandle nh, nh_private("~");
    // Create a ROS subscriber for arm commands
  ros::Subscriber sub = nh.subscribe ("arm_cmd", 1, arm_cb);

    
  int fd;
VARIANT vntResult;
uint32_t hCtrl, hRobot, hTask, hTask2, varres;
HRESULT hr;
int i, j;
double *pdArray;
double *pdArrays;


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
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
            odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
        broadcaster.sendTransform(odom_trans);
ROS_INFO("Starting");
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
BSTR bstrCommand;
VARIANT vntParam;
BSTR bstr1;
VARIANT vntRet;
//Get error flag handle
VARIANT vSpd;
int32_t eStatus;
uint32_t hSpd;
BSTR nullstr;
bstrCommand = SysAllocString(L"@ERROR_CODE");
vntParam.bstrVal = SysAllocString(L"");
nullstr = SysAllocString(L"");
vSpd.vt = VT_I4;
hr = bCap_ControllerGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("Could not get handle for errors %x\n", hr);
 return (hr);
 }
else {
//Read error value
hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
ROS_INFO("Errors %d", vSpd.lVal);
eStatus = vSpd.lVal;
VariantClear(&vSpd);
}
while(ros::ok() && eStatus !=  0){
ROS_INFO("Clearing errors");
// Clear previous errors
bstr1 = SysAllocString(L"ClearError");
vntParam.lVal = 0;
vntParam.vt = VT_I4;
hr = bCap_ControllerExecute(fd, hCtrl, bstr1, vntParam, &vntRet);
if (FAILED(hr))
  ROS_ERROR("[ClearError] command failed %x",hr);
else ROS_INFO("Errors Cleared");
SysFreeString(bstr1);
VariantClear(&vntParam);
VariantClear(&vntRet);
hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
ROS_INFO("Errors %d", vSpd.lVal);
eStatus = vSpd.lVal;
VariantClear(&vSpd);
}
//Get ROBSLAVE task handle
bstrCommand = SysAllocString(L"ROBSLAVE");
vntParam.bstrVal = SysAllocString(L"1");
vntParam.vt = VT_BSTR;
//hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
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
uint32_t jHandle;
float dJnt[8];
bstrCommand = SysAllocString(L"@CURRENT_ANGLE");
vntParam.bstrVal = SysAllocString(L"");
nullstr = SysAllocString(L"");
vntParam.vt = VT_BSTR;
hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &jHandle);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("Could not get handle %x\n", hr);
 return (hr);
 }
 //Read joint values
VARIANT vJnt;
hr = bCap_VariableGetValue(fd, jHandle, &vJnt);
SafeArrayAccessData(vJnt.parray, (void**)&pdArray);
memcpy(dJnt, pdArray, sizeof(dJnt));
SafeArrayUnaccessData(vJnt.parray);
VariantClear(&vJnt);
ROS_INFO("RET %x, Val %f -- %f -- %f -- %f -- %f -- %f Fig: %f",hr, dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5], dJnt[6]);
//Get speed handle
bstrCommand = SysAllocString(L"@SPEED");
vntParam.bstrVal = SysAllocString(L"");
nullstr = SysAllocString(L"");
vntParam.vt = VT_BSTR;
hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("Could not get handle for speed %x\n", hr);
 return (hr);
 }
else {
//Read speed value
hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
ROS_INFO("Speed %f", vSpd.fltVal);
VariantClear(&vSpd);
}
//Set speed vlaue
vSpd.fltVal = 100;
bstrCommand = SysAllocString(L"EXTSPEED");
VARIANT vntS;
float dnt[3];
dnt[0] = 50.0;
dnt[1] = 100.0;
dnt[2] = 100.0;
vntS.vt = (VT_R4 | VT_ARRAY);
vntS.parray = SafeArrayCreateVector(VT_R4, 0, 1);
SafeArrayAccessData(vntS.parray, (void**)&pdArray);
memcpy(pdArray, dnt, sizeof(dnt));
SafeArrayUnaccessData(vntS.parray);
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntS, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntS);
if FAILED(hr){
ROS_INFO("Ext Speed not set %x\n", hr);
 return (hr);
 }
else ROS_INFO("Ext Speed Set");

bstrCommand = SysAllocString(L"Motor");
vntParam.iVal = 1;
vntParam.vt = VT_I2;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);

if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Motor On");
/*
vntParam.vt = VT_BSTR;
vntParam.bstrVal = SysAllocString(L"@P P(-20.411865,19.493317,128.634155,-2.261426,15.894280,15.777246
,1)");
262.573486 -- 27.013416 -- 271.966553 -- 176.741867 -- 15.682789 -- 171.900070

bstrCommand = SysAllocString(L"");
hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
*/

vntParam.vt = VT_BSTR;
int n;
wchar_t wstr[100];
int tester = 0;
int pt = 0;
//vntParam.bstrVal = SysAllocString(wstr);
//printf("Sending: %S\n",vntParam.bstrVal);
vntParam.bstrVal = SysAllocString(L"@P J(0,0,90,0,0,0,0)");
//vntParam.bstrVal = SysAllocString(L"@P P(262.581024,27.017609,371.975250,176.741241,15.685188,171.900635,5)");
bstrCommand = SysAllocString(L"");
hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("FAIL %x\n", hr);
bstrCommand = SysAllocString(L"@ERROR_CODE");
vntParam.bstrVal = SysAllocString(L"");
nullstr = SysAllocString(L"");
vSpd.vt = VT_I4;
hr = bCap_ControllerGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
ROS_INFO("Errors %x", vSpd.lVal);
eStatus = vSpd.lVal;
VariantClear(&vSpd);
 return (hr);
 }
else ROS_INFO("Homed");
VARIANT vErr;
vErr.vt = VT_I4;
while(ros::ok()){
hr = bCap_VariableGetValue(fd, jHandle, &vJnt);
SafeArrayAccessData(vJnt.parray, (void**)&pdArray);
memcpy(dJnt, pdArray, sizeof(dJnt));
SafeArrayUnaccessData(vJnt.parray);
VariantClear(&vJnt);
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
    

    ros::spinOnce();
    //ROS_INFO("RET %x, Val %f -- %f -- %f -- %f -- %f -- %f Fig: %f",hr, dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5], dJnt[6]);


     /* hr = bCap_VariableGetValue(fd, hSpd, &vErr);
      eStatus = vErr.lVal;
      ROS_INFO("Errorstatus %f -- %f", eStatus, vErr.lVal);
      VariantClear(&vErr);*/
if(move_flag){      
     /* while(eStatus != 0 && ros::ok()){
        bstr1 = SysAllocString(L"ClearError");
        vntParam.vt = VT_I4;
        vntParam.lVal = 0;
        hr = bCap_ControllerExecute(fd, hCtrl, bstr1, vntParam, &vntRet);
        if (FAILED(hr))
          ROS_ERROR("[ClearError] command failed %x",hr);
        //else ROS_INFO("ErrorCleared %f", eStatus);
        SysFreeString(bstr1);
        VariantClear(&vntParam);
        VariantClear(&vntRet);
        hr = bCap_VariableGetValue(fd, hSpd, &vErr);
          if (FAILED(hr))
          ROS_ERROR("[GetError] command failed %x",hr);
          else{
        eStatus = vErr.lVal;
        VariantClear(&vErr);
        //ROS_INFO("ErrorCleared %f", eStatus);
        }
        ROS_INFO("ErrorCleared %ld", eStatus);
        }*/
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

};
