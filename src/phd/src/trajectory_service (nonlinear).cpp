#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <pcl/conversions.h>
#include "pcl_ros/transforms.h"
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include "std_msgs/String.h"
#include <sstream>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree.h>
#include <vector>
#include <ctime>
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/foreach.hpp>

#include "phd/trajectory_service.h"

#define DEBUG 1
#define WORKSPACE 1
#define HEIGHT_STEP .2
#define MAX_HEIGHT 1.3
#define OFFSET 0.1
#define POINTS 15


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surface (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr main_cloud (new pcl::PointCloud<pcl::PointXYZI>);
//make the publisher global so we can publish from the callback
ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher dir_pub;
ros::Publisher path_pub;

//calculates the distance between two points
float vec_length(pcl::PointXYZI pointA,pcl::PointXYZI pointB){

	return sqrt(pow(pointB.x-pointA.x,2)+pow(pointB.y-pointA.y,2)+pow(pointB.z-pointA.z,2));

}

//calculates a unit vector from the cross product
pcl::PointXYZI unit_cross(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB){
	//ROS_INFO("Crossing %f - %f - %f / %f - %f - %f", vectorA.x,vectorA.y,vectorA.z,vectorB.x,vectorB.y,vectorB.z);
	pcl::PointXYZI result;
	result.z = vectorA.x*vectorB.y-vectorB.x*vectorA.y;
	result.x = vectorA.y*vectorB.z-vectorB.y*vectorA.z;
	result.y = vectorA.z*vectorB.x-vectorB.z*vectorA.x;
	float length = sqrt(pow(result.x,2)+pow(result.y,2)+pow(result.z,2));
	result.x = result.x/length;
	result.y = result.y/length;
	result.z = result.z/length;
	if(result.z < 0){
		result.x *= -1;
		result.y *= -1;
		result.z *= -1;
	}
	return result;

}
//calculates a unit vector
pcl::PointXYZI unit_vector(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB){
ROS_INFO("Crossing %f - %f - %f / %f - %f - %f", vectorA.x,vectorA.y,vectorA.z,vectorB.x,vectorB.y,vectorB.z);
	pcl::PointXYZI result;
	float d_total = vec_length(vectorA,vectorB);
	result.x = (vectorB.x-vectorA.x)/d_total;
	result.y = (vectorB.y-vectorA.y)/d_total;
	result.z = (vectorB.z-vectorA.z)/d_total;
	return result;

}
//calculates a unit vector
pcl::PointXYZI unit_vector(pcl::PointXYZI vectorA){
	pcl::PointXYZI result;
	float d_total = sqrt(pow(vectorA.x,2)+pow(vectorA.y,2)+pow(vectorA.z,2));
	result.x = (vectorA.x)/d_total;
	result.y = (vectorA.y)/d_total;
	result.z = (vectorA.z)/d_total;
	return result;

}

//finds nearest point to given coords
pcl::PointXYZI find_pt(pcl::PointXYZI target_pt){
	int K = 1;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	pcl::PointXYZI searchPoint;
	float resolution = 128.0f;
	pcl::PointXYZI foundpt;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);

	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();
ROS_INFO("looking near %f/%f/%f",target_pt.x,target_pt.y,target_pt.z);
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		foundpt = cloud->points[pointIdxNKNSearch[0]];
		ROS_INFO("Found pt %f/%f/%f near %f/%f/%f",foundpt.x,foundpt.y,foundpt.z,target_pt.x,target_pt.y,target_pt.z);
	}

	return foundpt;

}

//find normal at given pt
pcl::PointXYZI find_normal(pcl::PointXYZI target_pt){

	pcl::PointXYZI foundpt;
	pcl::PointCloud<pcl::PointXYZI> cloudB;
	cloudB.push_back (target_pt);
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	// set parameters
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudptr = cloudB.makeShared();
	ne.setInputCloud (cloudptr);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_surface);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.05);
	ne.setViewPoint (0, 0, 0);
	// Compute the features
	ne.compute (*cloud_normals);
	foundpt.x = -cloud_normals->points[0].normal[0];
	foundpt.y = -cloud_normals->points[0].normal[1];
	foundpt.z = -cloud_normals->points[0].normal[2];
	//if(sqrt(pow(target_pt.x-foundpt.x
	return foundpt;

}

bool generate (phd::trajectory_service::Request  &req,
         phd::trajectory_service::Response &res)
{
	ROS_INFO("Received Trajectory Request");
	pcl::PointCloud<pcl::PointXYZI> cloudB;
	sensor_msgs::PointCloud2 cloud_msg2 = req.cloud_in;
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg2.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_msg2,*cloud);
	sensor_msgs::PointCloud2 cloud_msg1 = req.cloud_surface;
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg1.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_msg1,*cloud_surface);
	pcl::PointXYZI maxX;
	pcl::PointXYZI maxY;
	pcl::PointXYZI maxZ;  
	pcl::PointXYZI minX;
	pcl::PointXYZI minY;
	pcl::PointXYZI minZ;  
	pcl::PointXYZI start_pt;
	float maxD = 0;
	float D_calc;
	minX.x = 100;
	minX.z = 100;
	maxX.x = 0;
	maxX.z = 0;
	visualization_msgs::Marker dir_marker;
	dir_marker.header.frame_id = "/base_link";
	dir_marker.header.stamp = ros::Time::now();
	dir_marker.ns = "basic_shapes";
	dir_marker.id = 0;
	dir_marker.type = visualization_msgs::Marker::ARROW;
	dir_marker.action = visualization_msgs::Marker::ADD;
	dir_marker.pose.position.x = 0;
	dir_marker.pose.position.y = 0;
	dir_marker.pose.position.z = 0;
	dir_marker.pose.orientation.x = 0.0;
	dir_marker.pose.orientation.y = 0.0;
	dir_marker.pose.orientation.z = 0.0;
	dir_marker.pose.orientation.w = 1.0;
	dir_marker.scale.x = 0.01;
	dir_marker.scale.y = 0.015;
	dir_marker.color.r = 1.0f;
	dir_marker.color.g = 1.0f;
	dir_marker.color.b = 1.0f;
	dir_marker.color.a = 1.0;
	dir_marker.lifetime = ros::Duration();
	dir_marker.points.resize(2);
	BOOST_FOREACH (pcl::PointXYZI& p, cloud->points){
		if(p.z < minX.z && (p.x+p.y+p.z) != 0){ 
			minX = p;
		}
		if(p.z > maxX.z){ 
			maxX = p;
		}

	}
	maxX.intensity = 0;
	BOOST_FOREACH (pcl::PointXYZI& p, cloud->points){
		if(fabs(p.z-minX.z)<0.02){
			BOOST_FOREACH (pcl::PointXYZI& q, cloud->points){
				if(fabs(p.z-q.z)<0.02){			
					D_calc = sqrt(pow(q.x-p.x,2)+pow(q.y-p.y,2));
					if(D_calc > maxX.intensity && (p.x+p.y+p.z) != 0 && (q.x+q.y+q.z) != 0){ 
						minX = p;
						start_pt = p;
						dir_marker.points[0].x = p.x;
						dir_marker.points[0].y = p.y;
						dir_marker.points[0].z = p.z;
						dir_marker.points[1].x = q.x;
						dir_marker.points[1].y = q.y;
						dir_marker.points[1].z = q.z;
						ROS_INFO("Start %f-%f-%f",p.x,p.y,p.z);
						maxY = q;
						maxX.x = q.x;
						maxX.y = q.y;
						maxX.intensity = D_calc;
					}
				}
			}
		}

	}
	/*BOOST_FOREACH (pcl::PointXYZI& p, cloud->points){
		D_calc = sqrt(pow(minX.x-p.x,2)+pow(minX.y-p.y,2));
		if(D_calc> maxD && fabs(p.z-minX.z)<0.02 && (p.x+p.y+p.z) != 0 && D_calc < (WORKSPACE+0.05)){ 
			maxX.x = p.x;
			maxX.y = p.y;
			maxY = p;
			maxD = D_calc;
			dir_marker.points[1].x = p.x;
			dir_marker.points[1].y = p.y;
			dir_marker.points[1].z = p.z;
			ROS_INFO("End %f-%f-%f",p.x,p.y,p.z);
		}
	}*/

	dir_pub.publish(dir_marker);
	ROS_INFO("Maximums %f - %f / %f", minX.x,maxX.x,maxX.z);
	float w_space = sqrt(pow((maxX.x-minX.x),2)+pow((maxX.y-minX.y),2));
	if(w_space > WORKSPACE) w_space = WORKSPACE;
	pcl::PointXYZI second_pt;
	pcl::PointXYZI foundpt;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	pcl::PointXYZI searchPoint;
	int K = 250;
	float resolution = 128.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);

	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();
	ROS_INFO("Points added %f-%f-%f",start_pt.x,start_pt.y,start_pt.z);

	uint32_t shape = visualization_msgs::Marker::LINE_LIST;
	visualization_msgs::Marker marker;

	visualization_msgs::Marker path;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	//marker.scale.y = 0.015;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();
	
	float maxDistance=0;
	float newDistance=0;
	float weightedDistance=0;
	float newWeightedDistance=0;

	//find furthest point with constant Z
	if ( octree.nearestKSearch (start_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) //make sure we find something
	{
		//look at every found point and check for the furthest one with similar Z value (horizontal)		
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
			//first make sure the points are somewhat horizontal
			if(fabs(start_pt.z - cloud->points[ pointIdxNKNSearch[i] ].z) < 0.02){
				//caluclate XY the distance between points				
				newDistance = sqrt((start_pt.x - cloud->points[ pointIdxNKNSearch[i] ].x)*(start_pt.x - cloud->points[ pointIdxNKNSearch[i] ].x)+(start_pt.y - cloud->points[ pointIdxNKNSearch[i] ].y)*(start_pt.y - cloud->points[ pointIdxNKNSearch[i] ].y));
				//if we've found a longer distance, save the point				
				if(newDistance > maxDistance){
					maxDistance = newDistance;
					second_pt = cloud->points[ pointIdxNKNSearch[i] ];
				}
			}

		}
	}else ROS_INFO("Couldnt find neighbours");
	float d_total = vec_length(start_pt,second_pt);
	pcl::PointXYZI unit_v;
	pcl::PointXYZI zero_vector;
	pcl::PointXYZI dir_vector;
	pcl::PointXYZI hdir_vector;
	pcl::PointXYZI norm;
	pcl::PointXYZI end_pt;
	pcl::PointXYZI via_pt;
	pcl::PointXYZI target_pt;
	pcl::PointXYZI normal_pt;
	std::vector<pcl::PointXYZI> via_pts;
	std::vector<pcl::PointXYZI> target_pts;
	std::vector<pcl::PointXYZI> normal_pts;
	phd::trajectory_point t_pt;
	phd::trajectory_msg t_msg;

	zero_vector.x = 0;
	zero_vector.y = 0;
	zero_vector.z = 0;
	//dir_vector = unit_vector(start_pt,second_pt);
	hdir_vector = unit_vector(start_pt,maxY);
	int pt_ctr = 0;
	int hpt_ctr;
	int h_pts = POINTS;
	target_pt = start_pt;
	while(target_pt.z < maxX.z && target_pt.z  < MAX_HEIGHT){
		//ROS_INFO("MaxX.z %f pt.z %f",maxX.z,target_pt.z);
		hpt_ctr = 1;
		via_pts.push_back(find_pt(target_pt));
		normal_pts.push_back(find_normal(via_pts[pt_ctr]));
		while(hpt_ctr <= h_pts){	
			target_pt.x = via_pts[pt_ctr].x+((w_space*hdir_vector.x)/h_pts);
			target_pt.y = via_pts[pt_ctr].y+((w_space*hdir_vector.y)/h_pts);
			target_pt.z = via_pts[pt_ctr].z;
			target_pts.push_back(target_pt);
			via_pts.push_back(find_pt(target_pt));
			++pt_ctr;
			++hpt_ctr;
			normal_pts.push_back(find_normal(via_pts[pt_ctr]));
		}
		hdir_vector = unit_vector(via_pts[pt_ctr],via_pts[pt_ctr-(h_pts)]);
		dir_vector = unit_cross(find_normal(via_pts[pt_ctr]),unit_vector(via_pts[pt_ctr-h_pts],via_pts[pt_ctr]));
			//ROS_INFO("Find %f %f %f", dir_vector.x, dir_vector.y, dir_vector.z);
		target_pt.x = via_pts[pt_ctr].x + dir_vector.x*HEIGHT_STEP;
		target_pt.y = via_pts[pt_ctr].y + dir_vector.y*HEIGHT_STEP;
		target_pt.z = via_pts[pt_ctr].z + dir_vector.z*HEIGHT_STEP;
		target_pts.push_back(target_pt);
		if(target_pt.z> maxX.z || target_pt.z > MAX_HEIGHT){
			ROS_INFO("Breaking");
			break;
		}
		via_pts.push_back(find_pt(target_pt));
		++pt_ctr;
		normal_pts.push_back(find_normal(via_pts[pt_ctr]));
		dir_vector = unit_cross(find_normal(via_pts[pt_ctr]),unit_vector(via_pts[pt_ctr-1],via_pts[pt_ctr]));
		//dir_vector = - old dir_vector?
		hpt_ctr = 1;
		while(hpt_ctr <= h_pts){	
			target_pt.x = via_pts[pt_ctr].x+((w_space*hdir_vector.x)/h_pts);
			target_pt.y = via_pts[pt_ctr].y+((w_space*hdir_vector.y)/h_pts);
			target_pt.z = via_pts[pt_ctr].z;
			target_pts.push_back(target_pt);
			via_pts.push_back(find_pt(target_pt));
			++pt_ctr;
			++hpt_ctr;
			normal_pts.push_back(find_normal(via_pts[pt_ctr]));
			//update hdir here
		}
		hdir_vector = unit_vector(via_pts[pt_ctr-1],via_pts[pt_ctr-(1+h_pts)]);
		dir_vector = unit_cross(find_normal(via_pts[pt_ctr]),unit_vector(via_pts[pt_ctr-h_pts],via_pts[pt_ctr]));
		target_pt.x = via_pts[pt_ctr].x + dir_vector.x*HEIGHT_STEP;
		target_pt.y = via_pts[pt_ctr].y + dir_vector.y*HEIGHT_STEP;
		target_pt.z = via_pts[pt_ctr].z + dir_vector.z*HEIGHT_STEP;
		target_pts.push_back(target_pt);
		++pt_ctr;
		//dir_vector = unit_cross(find_normal(via_pts[pt_ctr]),unit_vector(via_pts[pt_ctr-1],via_pts[pt_ctr]));
		//dir_vector = - old dir_vector?
	}
	int num_pts = via_pts.size();
	for(int i = 0; i < num_pts; ++i){
		t_pt.x = via_pts[i].x;
		t_pt.y = via_pts[i].y;
		t_pt.z = via_pts[i].z;
		t_pt.nx = normal_pts[i].x;
		t_pt.ny = normal_pts[i].y;
		t_pt.nz = normal_pts[i].z;
		t_msg.points.push_back(t_pt);
		
	}


	// for(int ctr = 0; ctr < 12; ++ctr){
	/*
	marker.id = 0;
	marker.points[0].x = start_pt.x;
	marker.points[0].y = start_pt.y;
	marker.points[0].z = start_pt.z;
	marker.points[1].x = end_pt.x;
	marker.points[1].y = end_pt.y;
	marker.points[1].z = end_pt.z;
	// ROS_INFO("Found pt %f/%f/%f normal %f/%f/%f",foundpt[ctr].x,foundpt[ctr].y,foundpt[ctr].z,cloud_normals->points[ctr].normal[0],cloud_normals->points[ctr].normal[1],cloud_normals->points[ctr].normal[2]);
	//ROS_INFO("%lu",cloud_normals->points.size ());
	marker_pub.publish(marker);
	*/
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	path.header.frame_id = "/base_link";
	path.header.stamp = ros::Time::now();

	// Set the namespace and id for this path.  This serves to create a unique ID
	// Any path sent with the same namespace and id will overwrite the old one
	path.ns = "path_shapes";
	path.id = 0;

	// Set the path type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	path.type = visualization_msgs::Marker::LINE_STRIP;

	// Set the path action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	path.action = visualization_msgs::Marker::ADD;

	// Set the pose of the path.  This is a full 6DOF pose relative to the frame/time specified in the header
	path.pose.position.x = 0;
	path.pose.position.y = 0;
	path.pose.position.z = 0;
	path.pose.orientation.x = 0.0;
	path.pose.orientation.y = 0.0;
	path.pose.orientation.z = 0.0;
	path.pose.orientation.w = 1.0;

	// Set the scale of the path -- 1x1x1 here means 1m on a side
	path.scale.x = 0.01;

	// Set the color -- be sure to set alpha to something non-zero!
	path.color.r = 0.0f;
	path.color.g = 1.0f;
	path.color.b = 0.0f;
	path.color.a = 1.0;
	path.lifetime = ros::Duration();
	path.points.resize(num_pts);
	marker.points.resize(2*num_pts);
		marker.id = 2;
	for(int i = 0; i < num_pts; ++i){
		
		path.points[i].x = via_pts[i].x-(OFFSET*normal_pts[i].x);
		path.points[i].y = via_pts[i].y-(OFFSET*normal_pts[i].y);
		path.points[i].z = via_pts[i].z-(OFFSET*normal_pts[i].z);

		marker.points[i*2].x = via_pts[i].x-(OFFSET*normal_pts[i].x);
		marker.points[i*2].y = via_pts[i].y-(OFFSET*normal_pts[i].y);
		marker.points[i*2].z = via_pts[i].z-(OFFSET*normal_pts[i].z);
		marker.points[(i*2)+1].x = via_pts[i].x;
		marker.points[(i*2)+1].y = via_pts[i].y;
		marker.points[(i*2)+1].z = via_pts[i].z;
	}

	path_pub.publish(path);
	ros::Duration(0.5).sleep();
		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();
	
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
		marker.id = 3;

	for(int i = 0; i < num_pts; ++i){
		marker.points[i*2].x = target_pts[i].x;
		marker.points[i*2].y = target_pts[i].y;
		marker.points[i*2].z = target_pts[i].z;
		marker.points[(i*2)+1].x = via_pts[i].x;
		marker.points[(i*2)+1].y = via_pts[i].y;
		marker.points[(i*2)+1].z = via_pts[i].z;
	}
		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();
	bool ret = true;
	return ret;
}


int
main (int argc, char** argv)
{
	


	// Initialize ROS
	ros::init (argc, argv, "trajectory_generation_service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("trajectory_gen", generate); 
	marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	path_pub = nh.advertise<visualization_msgs::Marker>( "path_marker", 0 );
	dir_pub = nh.advertise<visualization_msgs::Marker>( "dir_marker", 0 );
	ROS_INFO("Trajectory Generator online");
	ros::spin();





ros::spin();
  
  
}
