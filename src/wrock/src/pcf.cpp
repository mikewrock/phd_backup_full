#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <pcl/conversions.h>
#include "pcl_ros/transforms.h"
ros::Publisher pub;
float px, py, pz;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
ROS_INFO("GOT A CLOUD");
/*
float height = cloud_msg->height;
float width = cloud_msg->width;
float rowstep = cloud_msg->row_step;
float pointstep = cloud_msg->point_step;
float data = cloud_msg->data[2930000];
std::string name = cloud_msg->fields[3].name;
float offset = cloud_msg->fields[3].offset;
float datatype = cloud_msg->fields[3].datatype;
float count = cloud_msg->fields[3].count;
printf("Height: %f\n",height);
printf("Width: %f\n",width);
printf("Data: %f\n",data);
printf("Name: %s\n",name.c_str());
printf("Offset: %f\n",offset);
printf("Datatype: %f\n",datatype);
printf("Count: %f\n",count);
printf("Rowstep: %f\n",rowstep);
printf("Pointstep: %f\n",pointstep);*/
sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;
cloud_msg2.fields[3].name = "intensity";
pcl::PointCloud<pcl::PointXYZI> cloud;
  
pcl::fromROSMsg(cloud_msg2,cloud);

float ctr =0;

px = 0;
py = 0;
pz = 0;
BOOST_FOREACH (const pcl::PointXYZI& pt, cloud.points){
    //printf ("\t(%f, %f, %f, %f)\n", pt.x, pt.y, pt.z, pt.intensity)
	if(pt.intensity > 1100 && pt.intensity < 2000) {
		px += pt.x;
		py += pt.y;
		pz += pt.z;
		++ctr;
//printf ("\t(X: %f,Y: %f,Z: %f) %f\n", pt.x,pt.y,pt.z,ctr);
}
}
px=px/ctr;
py=py/ctr;
pz=pz/ctr;
printf ("\t(X: %f,Y: %f,Z: %f)\n", px,py,pz);
geometry_msgs::Point output;
output.x = px;
output.y = py;
output.z = pz;
    
/*

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data*/
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("assembled_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<geometry_msgs::Point> ("laser_trans", 1);

  //tf::TransformBroadcaster broadcaster;
ros::spin();
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
