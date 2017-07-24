#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>

// ROS includes
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <rosbag/bag.h>
    #include <std_msgs/Float64.h>

    #include <rosbag/view.h>
    #include <boost/foreach.hpp>
    #define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube_move");

    ros::NodeHandle n;
    rosbag::Bag bag;
    bag.open("/home/mike/marker/test.bag", rosbag::bagmode::Write);

    std_msgs::Float64 f;
    f.data = 42;

    bag.write("fiducial", ros::Time::now(), f);
    f.data = 43;

    bag.write("fiducial", ros::Time::now(), f);
    f.data = 44;

    bag.write("fiducial", ros::Time::now(), f);
    f.data = 45;

    bag.write("fiducial", ros::Time::now(), f);
    f.data = 46;

    bag.write("fiducial", ros::Time::now(), f);
    f.data = 47;

    bag.write("fiducial", ros::Time::now(), f);

    bag.close();


bag.open("/home/mike/marker/test.bag", rosbag::bagmode::Read);


    rosbag::View view(bag, rosbag::TopicQuery("fiducial"));

    foreach(rosbag::MessageInstance const m, view)
    {

        std_msgs::Float64::ConstPtr i = m.instantiate<std_msgs::Float64>();
        if (i != NULL)
            std::cout << i->data << std::endl;
    }

    bag.close();

}




