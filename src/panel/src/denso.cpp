// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>

#include "bcap/stdint.h"
//#include <atlbase.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
// bCAP (Always last)
#include "bcap/bcap_client.h"

#define E_BUF_FULL  (0x83201483)

#define SERVER_IP_ADDRESS "tcp:192.168.0.1"
#define SERVER_PORT_NUM 5007
#define PERIOD 100
#define AMPLITUDE 15


int main(int argc, char **argv)
{
  ros::init (argc, argv, "close_connection");
  ros::NodeHandle nh, nh_private("~");
  
  int fd;
VARIANT vntResult;
uint32_t hCtrl, hRobot, hTask, hTask2, varres;
HRESULT hr;
int i, j;
double *pdArray;
double *pdArrays;

/* Init socket */
hr = bCap_Open_Client(SERVER_IP_ADDRESS, SERVER_PORT_NUM, 0, &fd);
if FAILED(hr) return (hr);
else ROS_INFO("SUCCESS");
/* Start b-CAP service */
hr = bCap_ServiceStart(fd, NULL);
if FAILED(hr) return (hr);
else ROS_INFO("SUCCESS");
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
else ROS_INFO("SUCCESS");
/* Get robot handle */
BSTR bstrRobotName, bstrRobotOpt;
bstrRobotName = SysAllocString(L"Arm");
bstrRobotOpt = SysAllocString(L"");
hr = bCap_ControllerGetRobot(fd, hCtrl, bstrRobotName, bstrRobotOpt, &hRobot);
SysFreeString(bstrRobotName);
SysFreeString(bstrRobotOpt);
if FAILED(hr) return (hr);
else ROS_INFO("SUCCESS! %d",hr);
/* Get arm control authority */
BSTR bstrCommand;
VARIANT vntParam;
/*
bstrCommand = SysAllocString(L"Takearm");
vntParam.bstrVal = SysAllocString(L"");
vntParam.vt = VT_BSTR;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
ROS_INFO("Authority?");
if FAILED(hr) {
ROS_INFO("Failed %d",hr);
return (hr);
}
else ROS_INFO("Authority");
// Motor on */

BSTR bstr1;
VARIANT vntRet;
    // Clear previous errors
    bstr1 = SysAllocString(L"ClearError");
    vntParam.lVal = 0;
    vntParam.vt = VT_I4;
    hr = bCap_ControllerExecute(fd, hCtrl, bstr1, vntParam, &vntRet);
    if (FAILED(hr))
      ROS_ERROR("[ClearError] command failed %x",hr);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Errors Cleared");




bstrCommand = SysAllocString(L"ROBSLAVE");
vntParam.bstrVal = SysAllocString(L"1");
vntParam.vt = VT_BSTR;
//hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
hr = bCap_ControllerGetTask(fd, hCtrl, bstrCommand, NULL, &hTask);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr) {
ROS_INFO("Failure %x %x",E_INVALIDARG, hr);
return (hr);
}else ROS_INFO("Got Handle: %d",hTask);
ros::Duration(1).sleep();
hr = bCap_TaskStart(fd, hTask, 1, NULL);

if FAILED(hr) {
ROS_INFO("Failure %x",hr);
return (hr);
}else ROS_INFO("Started Task");


/*
bstrCommand = SysAllocString(L"USEREXTENSION");
vntParam.bstrVal = SysAllocString(L"1");
vntParam.vt = VT_BSTR;
//hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
hr = bCap_ControllerGetTask(fd, hCtrl, bstrCommand, NULL, &hTask2);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr) {
ROS_INFO("Failure %x %x",E_INVALIDARG, hr);
return (hr);
}else ROS_INFO("Got Handle2: %d",hTask2);
ros::Duration(1).sleep();
hr = bCap_TaskStart(fd, hTask2, 1, NULL);

if FAILED(hr) {
ROS_INFO("Failure %x",hr);
return (hr);
}else ROS_INFO("Started Task2");



// Move to first pose */
VARIANT vntPose;
/*
BSTR bstrMoveOpt;
vntPose.bstrVal = SysAllocString(L"@E J1");
vntPose.vt = VT_BSTR;
bstrMoveOpt = SysAllocString(L"");
hr = bCap_RobotMove(fd, hRobot, 1L, vntPose, bstrMoveOpt);
VariantClear(&vntPose);
SysFreeString(bstrMoveOpt);
if FAILED(hr) return (hr); */
/* Get current angle */
/*double dJnt[8];
bstrCommand = SysAllocString(L"slvChangeMode");
vntParam.iVal = 1;
vntParam.vt = VT_I2;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
//SafeArrayAccessData(vntResult.parray, (void**)&pdArray);
//memcpy(dJnt, pdArray, sizeof(dJnt));
//SafeArrayUnaccessData(vntResult.parray);
SysFreeString(bstrCommand);
VariantClear(&vntParam);

/* Motor on */ /////////////////////////////////////////////////
/*bstrCommand = SysAllocString(L"Motor");
vntParam.iVal = 1;
vntParam.vt = VT_I2;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);

if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Sslave");
///////////////////////////*/
/* Get current angle */
float dJnt[8];
uint32_t hSpd;
BSTR nullstr;
bstrCommand = SysAllocString(L"@CURRENT_POSITION");
vntParam.bstrVal = SysAllocString(L"");
nullstr = SysAllocString(L"");
vntParam.vt = VT_BSTR;
hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
//SafeArrayAccessData(vntResult.parray, (void**)&pdArray);
//memcpy(dJnt, pdArray, sizeof(dJnt));
//SafeArrayUnaccessData(vntResult.parray);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("POSE %d", hSpd);
//ROS_INFO("SUCCESS: %f -- %f -- %f -- %f -- %f -- %f", dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5]);
VARIANT vSpd;
hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
SafeArrayAccessData(vSpd.parray, (void**)&pdArray);
memcpy(dJnt, pdArray, sizeof(dJnt));
SafeArrayUnaccessData(vSpd.parray);
VariantClear(&vSpd);
ROS_INFO("RET %x, Val %f -- %f -- %f -- %f -- %f -- %f",hr, dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5]);

bstrCommand = SysAllocString(L"@SPEED");
vntParam.bstrVal = SysAllocString(L"");
nullstr = SysAllocString(L"");
vntParam.vt = VT_BSTR;
hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
//SafeArrayAccessData(vntResult.parray, (void**)&pdArray);
//memcpy(dJnt, pdArray, sizeof(dJnt));
//SafeArrayUnaccessData(vntResult.parray);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else {
hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
ROS_INFO("Speed %f", vSpd.fltVal);
VariantClear(&vSpd);
}

vSpd.fltVal = 100;

/*hr = bCap_VariablePutValue(fd, hSpd, vSpd);
ROS_INFO("putvalFAIL %x\n", hr);
 if FAILED(hr){
 return (hr);
 }else {
ROS_INFO("Set SpeedEXT %f", vSpd.fltVal);
}*/
bstrCommand = SysAllocString(L"EXTSPEED");
VARIANT vntS;
float dnt[3];
dnt[0] = 100.0;
dnt[1] = 100.0;
dnt[2] = 99.0;
vntS.vt = (VT_R4 | VT_ARRAY);
vntS.parray = SafeArrayCreateVector(VT_R4, 0, 1);
SafeArrayAccessData(vntS.parray, (void**)&pdArray);
memcpy(pdArray, dnt, sizeof(dnt));
SafeArrayUnaccessData(vntS.parray);
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntS, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntS);

if FAILED(hr){
ROS_INFO("Ext Speed %x\n", hr);
 return (hr);
 }
else ROS_INFO("Ext Speed Set");

bstrCommand = SysAllocString(L"@EXTSPEED");
vntParam.bstrVal = SysAllocString(L"");
nullstr = SysAllocString(L"");
vntParam.vt = VT_BSTR;
hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
//SafeArrayAccessData(vntResult.parray, (void**)&pdArray);
//memcpy(dJnt, pdArray, sizeof(dJnt));
//SafeArrayUnaccessData(vntResult.parray);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }else {
hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
ROS_INFO("SpeedEXT %f", vSpd.fltVal);
VariantClear(&vSpd);
}

    // Clear previous errors
    bstr1 = SysAllocString(L"ClearError");
    vntParam.lVal = 0;
    vntParam.vt = VT_I4;
    hr = bCap_ControllerExecute(fd, hCtrl, bstr1, vntParam, &vntRet);
    if (FAILED(hr))
      ROS_ERROR("[ClearError] command failed %x",hr);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Errors Cleared");


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
bstrCommand = SysAllocString(L"Takearm");
vntParam.iVal = 1;
vntParam.vt = VT_I2;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);

if FAILED(hr){
ROS_INFO("Arm FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Takearm");
*/
int32_t lval;
lval = 1;
vntParam.vt = VT_BSTR;
vntParam.bstrVal = SysAllocString(L"P1");
bstrCommand = SysAllocString(L"");
hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
SysFreeString(bstrCommand);
VariantClear(&vntParam);

if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Moved");


ros::Duration(.2).sleep();


lval = 1;
vntParam.vt = VT_BSTR;
vntParam.bstrVal = SysAllocString(L"P0");
bstrCommand = SysAllocString(L"");
hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
SysFreeString(bstrCommand);
VariantClear(&vntParam);

if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Moved");

/*
uint32_t ret;

bstrCommand = SysAllocString(L"slvGetMode");
vntParam.bstrVal = SysAllocString(L"2");
vntParam.vt = VT_I2;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
//SafeArrayAccessData(vntResult.parray, (void**)&pdArray);
//memcpy(ret, pdArray, sizeof(ret));
//SafeArrayUnaccessData(vntResult.parray);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("MODE %d",vntResult.iVal);


*/


//ROS_INFO("SUCCESS: %f -- %f -- %f -- %f -- %f -- %f", dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5]);
/*start slave mode (Mode 0, J Type) 
bstrCommand = SysAllocString(L"slvChangeMode");
vntParam.bstrVal = SysAllocString(L"2");
vntParam.vt = VT_BSTR;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr) return (hr);
/* Execute slave move 
bstrCommand = SysAllocString(L"slvMove");
vntPose.vt = VT_R8 | VT_ARRAY;
vntPose.parray = SafeArrayCreateVector(VT_R8, 0, 8);
for(i = 0; i < PERIOD; i++)
{
SafeArrayAccessData(vntPose.parray, (void**)&pdArray);
pdArray[0] = dJnt[0] + i / 10.0;
pdArray[1] = dJnt[1] + AMPLITUDE*sin(2*M_PI*i/PERIOD);
for(j = 2; j < 8; j++)
{
pdArray[j] = dJnt[j];
}
SafeArrayUnaccessData(vntPose.parray);
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntPose, &vntResult);
ROS_INFO("MOVE: %x",hr);
//* if return code is not S_OK, then wait for 8 msec 
if(hr != 0)
{
ros::Duration(0.5).sleep();
//* if return code is E_BUF_FULL, then retry previous packet 
if(FAILED(hr)){
if(hr == E_BUF_FULL){
i--;
}else{
break;
}
}
}
}
SysFreeString(bstrCommand);
/* Stop robot 
bstrCommand = SysAllocString(L"slvMove");
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntPose, &vntResult);
SysFreeString(bstrCommand);
if FAILED(hr) return (hr);
/* Stop slave mode 
bstrCommand = SysAllocString(L"slvChangeMode");
vntParam.bstrVal = SysAllocString(L"0");
vntParam.vt = VT_BSTR;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr) return (hr);*/

ros::Duration(1).sleep();
bstrCommand = SysAllocString(L"Motor");
vntParam.iVal = 0;
vntParam.vt = VT_I2;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);

if FAILED(hr){
printf("FAIL %x\n", hr);
 return (hr);
 }
else ROS_INFO("Motor On");
if FAILED(hr) {
ROS_INFO("Motors Not off: %x",hr);
return (hr);
}else ROS_INFO("Motor Off");
/* Release arm control authority 
bstrCommand = SysAllocString(L"Givearm");
vntParam.bstrVal = SysAllocString(L"");
vntParam.vt = VT_BSTR;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr) return (hr);
else ROS_INFO("SUCCESS");
/* Release robot handle */
bCap_RobotRelease(fd, &hRobot);
/* Release controller handle */
bCap_ControllerDisconnect(fd, &hCtrl);
/* Stop b-CAP service (Very important in UDP/IP connection) */
bCap_ServiceStop(fd);
/* Close socket */
bCap_Close_Client(&fd);


};
