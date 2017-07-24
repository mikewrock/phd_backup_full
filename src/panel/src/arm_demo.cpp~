#include <wrock/cube_msg.h>
#include <ros/ros.h>
#include <string>
#include <cstdio>

#include <sensor_msgs/JointState.h>
// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud.h"
int vel = 1;
int acc = 2; 
int ret = 0;
double joint1;

wrock::cube_msg cube_cmd;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint1 = msg->position[4];

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube_demo");
/*
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<wrock::AddTwoInts>("add_two_ints");
  wrock::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
*/  
  ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<wrock::cube_msg>("joint_cmd", 100);
ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
    ros::Subscriber joint_sub = n.subscribe("joint_states", 1, jointCallback);
    // Create the service client for calling the assembler
  ros::ServiceClient    client = n.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
laser_assembler::AssembleScans2 srv;
joint1 = 1;
ROS_INFO("Waiting for j1");
while(joint1 < -0.1 || joint1 > 0.1){
ros::spinOnce();
//printf("j1: %f\n",joint1);
} 
cube_cmd.j1 = 2.5;
cube_cmd.j2 = 0;
cube_cmd.j3 = 0;
cube_cmd.j4 = 0;
cube_cmd.j5 = 0;
cube_cmd.vel = 1.5;
cube_cmd.acc = 2;
cube_cmd.pose = true;
        //send the joint state and transform
    // Populate our service request based on our timer callback times
cmd_pub.publish(cube_cmd);
ros::spinOnce();
printf("j1: %f\n",joint1);
ROS_INFO("going to start");
while(joint1 < 2.4){
ros::spinOnce();
//printf("j1: %f\n",joint1);
} 
 srv.request.begin = ros::Time::now();
cube_cmd.j1 = 0;

cube_cmd.vel = 0.25;
cmd_pub.publish(cube_cmd);
ros::spinOnce();
ROS_INFO("waiting for end");
while(joint1 > 0){
ros::spinOnce();
} 
ROS_INFO("done");
 srv.request.end   = ros::Time::now();
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
while(ros::ok()){
}
  return 0;
}



