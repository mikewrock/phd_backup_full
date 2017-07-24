#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/conversions.h>
#include <tf/transform_listener.h>
#include "laser_assembler/AssembleScans2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
ros::Publisher pub;
float px, py, pz;
int cloud_ctr;
sensor_msgs::PointCloud saved_cloud1;
sensor_msgs::PointCloud saved_cloud2;
sensor_msgs::PointCloud localized_cloud1;
sensor_msgs::PointCloud localized_cloud2;
  tf::TransformListener *listener;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

//sensor_msgs::PointCloud cloud_msg2;
//listener.waitForTransform("/world", (*pcl_in).header.frame_id, (*pcl_in).header.stamp, ros::Duration(5.0)); 
    
  // Publish the data*/
if(cloud_ctr == 0){
//pcl::fromROSMsg(*cloud_msg,saved_cloud1);  
sensor_msgs::convertPointCloud2ToPointCloud (*cloud_msg, saved_cloud1);

ros::Time trans = ros::Time::now()+ ros::Duration(0.5);
ROS_INFO("waitinForTransform");
listener->waitForTransform("/marker", (*cloud_msg).header.frame_id, (*cloud_msg).header.stamp + ros::Duration(0.5), ros::Duration(5.0)); 
ROS_INFO("transformingPointCloud");
listener->transformPointCloud ("/marker", saved_cloud1, localized_cloud1);
//pcl_ros::transformPointCloud("/marker", trans, saved_cloud1,"/base_link",localized_cloud1,*listener);
pub.publish (localized_cloud1);
}
else{
//pcl::fromROSMsg(*cloud_msg,saved_cloud2);  
sensor_msgs::convertPointCloud2ToPointCloud (*cloud_msg, saved_cloud2);
listener->waitForTransform("/marker", (*cloud_msg).header.frame_id, (*cloud_msg).header.stamp, ros::Duration(5.0)); 
ros::Time trans = ros::Time::now();
listener->transformPointCloud ("/marker", trans, saved_cloud2, "/base_link", localized_cloud2);
//pcl_ros::transformPointCloud("/marker", trans, saved_cloud1,"/base_link",localized_cloud1,*listener);
pub.publish (localized_cloud1);

}
++cloud_ctr;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("assembled_cloud", 1, cloud_cb);
listener    = new tf::TransformListener();
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud> ("local_cloud", 1);
ros::service::waitForService("assemble_clouds");
  ros::ServiceClient client = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_clouds");
  laser_assembler::AssembleScans2 srv;
  srv.request.begin = ros::Time::now();
cloud_ctr = 0;
while(ros::ok() && cloud_ctr < 2){

  ros::spinOnce();

}
  //srv.request.end = ros::Time::now();
  //if (client.call(srv))
   // printf("Got cloud with %u points\n", srv.response.cloud.points.size());
  //else
   // printf("Service call failed\n");

  //tf::TransformBroadcaster broadcaster;
  // Spin
 /* while (ros::ok()){
  ros::spinOnce();
tf::Transform odom_trans;
    odom_trans.setOrigin( tf::Vector3(px, py, pz) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
	odom_trans.setRotation(q);
        //send the joint state and transform
        broadcaster.sendTransform(tf::StampedTransform(odom_trans, ros::Time::now(), "global", "laser"));
  }*/
}
