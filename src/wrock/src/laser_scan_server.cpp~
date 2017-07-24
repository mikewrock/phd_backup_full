#include "ros/ros.h"
#include "wrock/laser_scan.h"	
#include "wrock/cube_msg.h"	
#include <tf/transform_datatypes.h>	
#include <math.h>

#include <sensor_msgs/JointState.h>
// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud.h"
bool do_scan =false;

double joint1;
bool add(wrock::laser_scan::Request  &req,
         wrock::laser_scan::Response &res)
{
do_scan = true;
return true;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint1 = msg->position[4];

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_server");
  ros::NodeHandle n;
  ros::Publisher cmd_pub = n.advertise<wrock::cube_msg>("joint_cmd", 100);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
  ros::ServiceServer service = n.advertiseService("laser_scan", add);
  ros::Subscriber joint_sub = n.subscribe("joint_states", 1, jointCallback);
  ros::ServiceClient    client = n.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
  laser_assembler::AssembleScans2 srv;
wrock::cube_msg cube_cmd;
while(ros::ok()){
  while(ros::ok() && do_scan == false){ 
    ros::spinOnce();
  }
  do_scan = false;
  joint1 = 1;
  ROS_INFO("Waiting for j1");
  while(joint1 < -0.1 || joint1 > 0.1){
  } 
  cube_cmd.j1 = 2.5;
  cube_cmd.vel = 1.5;
  cube_cmd.acc = 2;
  cube_cmd.pose = true;
  cmd_pub.publish(cube_cmd);
  ros::spinOnce();
  while(joint1 < 2.4){
  ros::spinOnce();
  } 
  srv.request.begin = ros::Time::now();
  cube_cmd.j1 = 0;
  cube_cmd.vel = 0.25;
  cmd_pub.publish(cube_cmd);
  ros::spinOnce();
  while(joint1 > 0){
  ros::spinOnce();
  } 
  ROS_INFO("done");srv.request.end   = ros::Time::now();
  // Make the service call
  if (client.call(srv))
  {
    ROS_INFO("Published Cloud ", (uint32_t)(srv.response.cloud.width)) ;
    pub.publish(srv.response.cloud);
  }
  else
  {
    ROS_ERROR("Error making service call\n") ;
  }
  ros::spinOnce();
  }
  return 0;
}
