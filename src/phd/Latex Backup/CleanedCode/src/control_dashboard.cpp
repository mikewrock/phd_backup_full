#include "control_dashboard.h"
#include <QDebug>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <QFileDialog>
#include <QDateTime>
#include <QTimer>

//A few useful defines
#define ARM 1
#define HUSKY 2
#define SPEED 10
#define PTP 1
#define CP 2
#define T 6
#define JOINT 3
#define STRING 4
#define SHUTDOWN 5
#define TARGET_TOLERANCE .005
#define DEBUG 1

namespace control_panel
{
	ControlPanel::ControlPanel( QWidget* parent )
	  : rviz::Panel( parent )
	  , control_panel()
	{
		//Initial setup
		ui_.setupUi(this);
		//Subscriber to notify a scan is complete
		cloud_sub_ = nh_.subscribe("finished_scan", 1, &ControlPanel::cpCB, this);
		//Subscriber to display arm pose
		pose_sub_ = nh_.subscribe("arm_pose", 1, &ControlPanel::poseCB, this);
		//Subscriber to listen for move_base results
		move_base_sub_ = nh_.subscribe("move_base/result", 1, &ControlPanel::movebaseCB, this);
		//Publisher to reset map
		reset_map_pub_ = nh_.advertise<std_msgs::String> ("syscommand", 1);
		//Service to clear costmaps
		reset_map_srv = nh_.serviceClient<phd::empty>("move_base/clear_costmaps");
		//Timer to control ROS spinning
		QTimer* spin_timer = new QTimer( this );
		//Initialize control panel
		int ret = control_panel.init();
		//SIGNAL connections
		connect(ui_.nav_mode_button, SIGNAL(clicked()), this, SLOT(do_nav()));
		connect(ui_.show_nav_button, SIGNAL(clicked()), this, SLOT(show_nav()));
		connect(ui_.fscan_button, SIGNAL(clicked()), this, SLOT(fake_scan()));
		connect(ui_.scan_button, SIGNAL(clicked()), this, SLOT(scan()));
		connect(ui_.set_home_button, SIGNAL(clicked()), this, SLOT(homing_scan()));
		connect(ui_.thickness_button, SIGNAL(clicked()), this, SLOT(thickness_from_file()));
		connect(ui_.cluster1, SIGNAL(clicked()), this, SLOT(cluster()));
		connect(ui_.gen_trajectory, SIGNAL(clicked()), this, SLOT(gen_trajectory()));
		connect(ui_.f_traj, SIGNAL(clicked()), this, SLOT(load_traj()));
		connect(ui_.pose_step, SIGNAL(clicked()), this, SLOT(pose_step()));
		connect(ui_.pose_loop_button, SIGNAL(clicked()), this, SLOT(pose_loop()));
		connect(ui_.arm_loop_button, SIGNAL(clicked()), this, SLOT(arm_step()));
		connect(ui_.step_button, SIGNAL(clicked()), this, SLOT(arm_step()));
		connect(ui_.soft_stop, SIGNAL(clicked()), this, SLOT(soft_stop()));
		connect(ui_.soft_stop_2, SIGNAL(clicked()), this, SLOT(soft_stop()));
		connect(ui_.reset_map_button, SIGNAL(clicked()), this, SLOT(reset_map()));
		connect(ui_.reset_map_button_2, SIGNAL(clicked()), this, SLOT(reset_map()));
		connect(ui_.clean_button, SIGNAL(clicked()), this, SLOT(set_clean()));
		connect(ui_.sprayed_button, SIGNAL(clicked()), this, SLOT(set_sprayed()));
		connect(ui_.calculate_button, SIGNAL(clicked()), this, SLOT(calculate_thickness()));
		connect(ui_.save_selection, SIGNAL(clicked()), this, SLOT(save_selection()));
		connect(spin_timer, SIGNAL(timeout()), this, SLOT(rosSpinner()));
		connect(ui_.shutdown_button, SIGNAL(clicked()), this, SLOT(onShutdownCommand()));
		connect(ui_.ptp_command_button, SIGNAL(clicked()), this, SLOT(onPTPCommand()));
		connect(ui_.cp_command_button, SIGNAL(clicked()), this, SLOT(onCPCommand()));
		connect(ui_.t_command_button, SIGNAL(clicked()), this, SLOT(onTCommand()));
		connect(ui_.speed_button, SIGNAL(clicked()), this, SLOT(onSpeedCommand()));
		connect(ui_.string_button, SIGNAL(clicked()), this, SLOT(onStringCommand()));
		connect(ui_.joint_command_button, SIGNAL(clicked()), this, SLOT(onJointCommand()));
		//Initialize various counters and booleans
		cluster_index = 0;
		traj_ctr = 0;
		traj_size = 0;
		auto_arm = false;
		auto_pose = false;
		auto_traj = false;
		//Begin the ROS spin timer
		spin_timer->start( 10 );
		//Containers for identifying status messages from control_panel
		std::stringstream ss, ss2;
		ss << "CLOUD";
		ss2 << "TRAJ";
		CLOUD.data = ss.str();
		TRAJ.data = ss2.str();
	}

	ControlPanel::~ControlPanel(){}
	
	//ros callbacks
	void ControlPanel::poseCB(const phd::arm_msg msg){
		//Update arm pose values to be displayed
		ui_.x_browser->setText(QString::number(msg.x));
		ui_.y_browser->setText(QString::number(msg.y));
		ui_.z_browser->setText(QString::number(msg.z));
		ui_.rx_browser->setText(QString::number(msg.rx));
		ui_.ry_browser->setText(QString::number(msg.ry));
		ui_.rz_browser->setText(QString::number(msg.rz));
		ui_.j1_browser->setText(QString::number(msg.j1));
		ui_.j2_browser->setText(QString::number(msg.j2));
		ui_.j3_browser->setText(QString::number(msg.j3));
		ui_.j4_browser->setText(QString::number(msg.j4));
		ui_.j5_browser->setText(QString::number(msg.j5));
		ui_.j6_browser->setText(QString::number(msg.j6));
		//Check to see if arm is executing a trajectory and reached its goal
		float dis = tgt_distance(msg,arm_tgt);
		ui_.target_dis->setText(QString::number(dis));
		if(auto_arm&&tgt_distance(msg,arm_tgt)<TARGET_TOLERANCE){
			//Call the arm_step function to continue executing trajectory points
			ControlPanel::arm_step();
		}	
	}
	//Callback function for when robot reaches its goal
	void ControlPanel::movebaseCB(const move_base_msgs::MoveBaseActionResult result){
		if(DEBUG) ROS_INFO("Goal Achieved");
		auto_traj = false;
		//If the robot is running autonomously reset trajectory size counter and relevant parameters in control_panel then perform a new laser scan
		if(auto_pose){
			traj_size = 0;
			control_panel.reset_things();
			control_panel.scan(ui_.marker_name_box->text().toStdString(),false);

		}
	}
	//Callback for receiving commands from control_panel
	void ControlPanel::cpCB(const std_msgs::String msg){
		if(DEBUG) ROS_INFO("Got a %s message", msg.data.c_str());
		//Control_panel got a new cloud
		if(auto_pose && msg.data==CLOUD.data){
			//Generate a new trajectory
			traj_size = control_panel.gen_trajectory(ui_.marker_name_box->text().toStdString());
			auto_traj = true;
		}
		//Control_panel got a new trajectory
		if(auto_traj && msg.data == TRAJ.data){
			traj_ctr = 0;
			//Execute arm trajectory
			ControlPanel::arm_step();
		}

	}
	//This gets called each time the spin timer finishes
	void ControlPanel::rosSpinner(){
		ros::spinOnce();
	}
	//Helper function for calculating the arm's distance to its target
	float ControlPanel::tgt_distance(phd::arm_msg arm, phd::arm_msg tgt){
		float ret = sqrt(pow(arm.x-tgt.x,2)+pow(arm.y-tgt.y,2)+pow(arm.z-tgt.z,2));
		return ret;
	}

	//Move laser scanner to its navigation position (horizontal) or whatever position is specified by the user in the text box
	void ControlPanel::do_nav(){
		float pos = ui_.pos_box->text().toFloat();
		  control_panel.nav_mode(pos);
	}
	//Reset the map
	void ControlPanel::reset_map(){
		//Publish "reset" message to reset map
		std_msgs::String msg;
		std::stringstream ss;
		ss << "reset";
		msg.data = ss.str();
		reset_map_pub_.publish(msg);
		//Clear the costmap by calling and empty service
		phd::empty srv;
		reset_map_srv.call(srv);
		//Re-centre the map on the world origin
		tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion q;
		transform.setOrigin( tf::Vector3(0, 0, 0) );
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
		  
	}
	//Show the robot's goal location and path
	void ControlPanel::show_nav(){

		  control_panel.show_nav();
	}
	//Save selection of cloud
	void ControlPanel::save_selection(){

		  control_panel.save_selection();
	}
	//Manually set the "before" section for thickness estimation
	void ControlPanel::set_clean(){

		  control_panel.set_clean();
	}
	//Manually set the "after" section for thickness estimation
	void ControlPanel::set_sprayed(){

		  control_panel.set_sprayed();
	}
	//Manually generate thickness estimate
	void ControlPanel::calculate_thickness(){

		  control_panel.calculate_thickness(ui_.save_name_box->text().toStdString());
	}
	//Perform a laser scan and tell control_panel the fiducial name and whether to localize the scan
	void ControlPanel::scan(){

		  control_panel.scan(ui_.marker_name_box->text().toStdString(),false);

	}//Perform a laser scan and tell control_panel the fiducial name and to set the pose as global origin 
	void ControlPanel::homing_scan(){

		  control_panel.scan(ui_.marker_name_box->text().toStdString(),true);

	}
	//Load a scan from file
	void ControlPanel::fake_scan(){

		  control_panel.fscan(ui_.filename_box->text().toStdString(),ui_.auto_localize->isChecked(),ui_.set_home->isChecked(),ui_.marker_name_box->text().toStdString(),ui_.autocrop_box->isChecked());
	}
	//Calculate thickness from the two files specified
	void ControlPanel::thickness_from_file(){

		  control_panel.thickness_from_file(ui_.before_box->text().toStdString(),ui_.after_box->text().toStdString());
	}
	//Cluster the high intensity points in the selected area
	void ControlPanel::cluster(){
		//Call the cluster function in control_panel and pass it which marker point to be clustered, then update the text in the panel
		control_panel.cluster(ui_.marker_name_box->text().toStdString(),cluster_index);
		if(cluster_index == 0){
		 ui_.cluster1->setText("Cluster Pt. 2");
		 cluster_index+=1;
		 ui_.marker_info->setText("Ready to cluster");
		}
		else if(cluster_index == 1){
		 ui_.cluster1->setText("Cluster Pt. 3");
		 ++cluster_index;
		}
		else if(cluster_index == 2){
		 ui_.cluster1->setText("Cluster Pt. 1");
		 ui_.marker_info->setText("Marker Recorded");
		//Once the final marker point is clustered, we can enable auto localization
		 ui_.auto_localize->setCheckState(Qt::Checked);
		 cluster_index = 0;
		}
	}
	//Generate a trajectory for the arm
	void ControlPanel::gen_trajectory(){

		  traj_size = control_panel.gen_trajectory(ui_.marker_name_box->text().toStdString());

	}
	//Send a point-to-point command to the arm through control_panel
	void ControlPanel::onPTPCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), PTP);
	}
	//Send a continuous path motion command to the arm through control_panel
	void ControlPanel::onCPCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), CP);
	}
	//Send a target position command to the arm through control_panel
	void ControlPanel::onTCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), T);
	}
	//Send a shut down command to the arm through control_panel
	void ControlPanel::onShutdownCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), SHUTDOWN);
	}
	//Send a joint command to the arm through control_panel
	void ControlPanel::onJointCommand(){

		control_panel.send_joint_command(ui_.j1_box->text().toFloat(),ui_.j2_box->text().toFloat(),ui_.j3_box->text().toFloat(),ui_.j4_box->text().toFloat(),ui_.j5_box->text().toFloat(),ui_.j6_box->text().toFloat());

	}
	//Send a user-entered string to the arm through control_panel
	void ControlPanel::onStringCommand(){

		control_panel.send_string(ui_.string_box->text().toStdString());

	}
	//Set the speed of the arm through control_panel
	void ControlPanel::onSpeedCommand(){
		float speed = ui_.speed_box->text().toFloat();
		control_panel.set_speed(speed);

	}
	//Load a saved trajectory from file
	void ControlPanel::load_traj(){

		traj_size = control_panel.load_traj(ui_.filename_box_5->text().toStdString());

	}
	//Move to the next shotcrete location
	void ControlPanel::pose_step(){
		
		control_panel.find_and_move(ui_.step->isChecked());
	
	}
	//Move the arm to the next point in the trajectory array
	void ControlPanel::arm_step(){

		if(DEBUG) ROS_INFO("ctr %d, size %d", traj_ctr,traj_size);
		//If the trajectory is complete
		if(traj_ctr>=traj_size){
			if(DEBUG) ROS_INFO("Arm Trajectory Complete");
			auto_arm = false;
			traj_ctr = 0;
			//If the robot is in autonomous mode, move to the next location for shotcrete
			if(auto_pose) ControlPanel::pose_loop();
		}
		//If the trajectory is not complete tell control_panel to move the arm to the point
		else{
			arm_tgt = control_panel.step(traj_ctr,ui_.step->isChecked(),ui_.fig_box->text().toFloat());
			++traj_ctr;
			auto_arm = true;
		}
	}
	/*If the user initiates autonomous mode, we set auto_pose to be true then move to the first shotcreting loction. 
	This function uses the assumption the robot starts at an area that does not need shotcrete, 
	if operating procedures requires the robot start at an unshotcreted area, replace the find_and_move call with
	a scan() call*/
	void ControlPanel::pose_loop(){			
		auto_pose = true;
		control_panel.find_and_move(ui_.step->isChecked());
	}
	//Software E-Stop
	void ControlPanel::soft_stop(){
		auto_arm = false;
		auto_pose = false;
		auto_traj = false;
		control_panel.soft_stop();
		control_panel.reset_things();
	}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_panel::ControlPanel,rviz::Panel)
