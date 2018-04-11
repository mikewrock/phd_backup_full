#include "control_dashboard.h"
#include <QDebug>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <QFileDialog>
#include <QDateTime>
#include <QTimer>

#define FORWARD 0
#define REVERSE 1
#define RIGHT 2
#define LEFT 3
#define UP 4
#define DOWN 5
#define PITCH_UP 6
#define YAW_UP 7
#define ROLL_UP 8
#define PITCH_DOWN 9
#define YAW_DOWN 10
#define ROLL_DOWN 11
#define ARM 1
#define HUSKY 2
#define SPEED 10
#define PTP 1
#define CP 2
#define T 6
#define JOINT 3
#define STRING 4
#define SHUTDOWN 5
#define TARGET_TOLERANCE .01

namespace control_panel
{
	ControlPanel::ControlPanel( QWidget* parent )
	  : rviz::Panel( parent )
	  , control_panel()
	//, vel_control()
	{
		//Initial setup
		ui_.setupUi(this);
		//traj sub
		traj_sub_ = nh_.subscribe("dir_marker", 1, &ControlPanel::trajCB, this);
		//cloud sub
		cloud_sub_ = nh_.subscribe("finished_scan", 1, &ControlPanel::cloudCB, this);
		//pose sub
		pose_sub_ = nh_.subscribe("arm_pose", 1, &ControlPanel::poseCB, this);
		//move_base sub
		move_base_sub_ = nh_.subscribe("move_base/result", 1, &ControlPanel::movebaseCB, this);
		reset_map_pub_ = nh_.advertise<std_msgs::String> ("syscommand", 1);
		reset_map_srv = nh_.serviceClient<phd::empty>("move_base/clear_costmaps");
		QTimer* spin_timer = new QTimer( this );
		//Initialize control panel
		int ret = control_panel.init();
		//Initialize the manual operation node
		//ret = vel_control.init();
		//SIGNAL connections
		connect(ui_.nav_mode_button, SIGNAL(clicked()), this, SLOT(do_nav()));
		connect(ui_.est_pos, SIGNAL(clicked()), this, SLOT(do_estimate()));
		//connect(ui_.exe_poses, SIGNAL(clicked()), this, SLOT(pose_step()));
		connect(ui_.show_nav_button, SIGNAL(clicked()), this, SLOT(show_nav()));
		connect(ui_.fscan_button, SIGNAL(clicked()), this, SLOT(fake_scan()));
		connect(ui_.scan_button, SIGNAL(clicked()), this, SLOT(scan()));
		connect(ui_.thickness_button, SIGNAL(clicked()), this, SLOT(thickness()));
		connect(ui_.localization_button, SIGNAL(clicked()), this, SLOT(localization_scan()));
		connect(ui_.localization_button_2, SIGNAL(clicked()), this, SLOT(localization_scan_2()));
		connect(ui_.cluster1, SIGNAL(clicked()), this, SLOT(cluster_1()));
		connect(ui_.start_point, SIGNAL(clicked()), this, SLOT(start_pt()));
		connect(ui_.gen_trajectory, SIGNAL(clicked()), this, SLOT(gen_trajectory()));
		connect(ui_.scan_360, SIGNAL(clicked()), this, SLOT(scan_360()));
		connect(ui_.f_traj, SIGNAL(clicked()), this, SLOT(load_traj()));
		connect(ui_.pose_button, SIGNAL(clicked()), this, SLOT(do_poses()));
		connect(ui_.pose_step, SIGNAL(clicked()), this, SLOT(pose_step()));
		connect(ui_.pose_loop_button, SIGNAL(clicked()), this, SLOT(pose_loop()));
		connect(ui_.arm_loop_button, SIGNAL(clicked()), this, SLOT(arm_loop()));
		connect(ui_.step_button, SIGNAL(clicked()), this, SLOT(arm_step()));
		connect(ui_.soft_stop, SIGNAL(clicked()), this, SLOT(soft_stop()));
		connect(ui_.reset_map_button, SIGNAL(clicked()), this, SLOT(reset_map()));
		connect(ui_.clean_button, SIGNAL(clicked()), this, SLOT(set_clean()));
		connect(ui_.sprayed_button, SIGNAL(clicked()), this, SLOT(set_sprayed()));
		connect(ui_.calculate_button, SIGNAL(clicked()), this, SLOT(calculate_thickness()));
		connect(ui_.compare_button, SIGNAL(clicked()), this, SLOT(compare_results()));
		connect(ui_.save_selection, SIGNAL(clicked()), this, SLOT(save_selection()));


		connect(spin_timer, SIGNAL(timeout()), this, SLOT(rosSpinner()));
		  //connect(ui_.launch_node_button, SIGNAL(clicked()), this, SLOT(onLaunchNode()));
		  connect(ui_.shutdown_button, SIGNAL(clicked()), this, SLOT(onShutdownCommand()));
		  //connect(ui_.clear_error_button, SIGNAL(clicked()), this, SLOT(onClearErrorCommand()));
		  connect(ui_.ptp_command_button, SIGNAL(clicked()), this, SLOT(onPTPCommand()));
		  connect(ui_.cp_command_button, SIGNAL(clicked()), this, SLOT(onCPCommand()));
		  connect(ui_.t_command_button, SIGNAL(clicked()), this, SLOT(onTCommand()));
		  connect(ui_.speed_button, SIGNAL(clicked()), this, SLOT(onSpeedCommand()));
		  connect(ui_.string_button, SIGNAL(clicked()), this, SLOT(onStringCommand()));
		  connect(ui_.joint_command_button, SIGNAL(clicked()), this, SLOT(onJointCommand()));
		//connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
		//Index for keeping track of which marker point is being clustered
		cluster_index = 0;
		pose_ctr = 5;
		traj_ctr = 0;
		traj_size = 0;
		auto_arm = false;
		auto_pose = false;
		auto_traj = false;
		spin_timer->start( 10 );
	}

	ControlPanel::~ControlPanel(){}
	
	//ros callbacks
	void ControlPanel::poseCB(const phd::arm_msg msg){
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
		if(auto_arm&&tgt_distance(msg,arm_tgt)<TARGET_TOLERANCE) ControlPanel::arm_loop();
	}
	void ControlPanel::movebaseCB(const move_base_msgs::MoveBaseActionResult result){

		ROS_INFO("Goal Achieved :)");
		auto_traj = false;
		//do trajectory
		if(auto_pose){
			auto_arm = true;
			control_panel.scan(ui_.marker_name_box->text().toStdString(),ui_.set_home->isChecked());

		}
	}

	void ControlPanel::cloudCB(const std_msgs::String msg){
		ROS_INFO("Got a cloud - Dashboard");
		if(auto_pose && auto_arm){
			traj_size = control_panel.gen_trajectory(ui_.marker_name_box->text().toStdString());
			auto_traj = true;
			traj_ctr = 0;
		}

	}
	void ControlPanel::trajCB(const visualization_msgs::Marker mark){
		ROS_INFO("Got a traj - Dashboard");
		if(auto_traj){
			ControlPanel::arm_loop();
		}

	}

	void ControlPanel::rosSpinner(){
		ros::spinOnce();
	}

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
		std_msgs::String msg;

		std::stringstream ss;
		ss << "reset";
		msg.data = ss.str();
		
		phd::empty srv;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	transform.setOrigin( tf::Vector3(0, 0, 0) );
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
		reset_map_srv.call(srv);
		reset_map_pub_.publish(msg);
		  
	}
	//Show the robot's goal location and path
	void ControlPanel::show_nav(){

		  control_panel.show_nav();
	}
	//
	void ControlPanel::save_selection(){

		  control_panel.save_selection();
	}
	//
	void ControlPanel::set_clean(){

		  control_panel.set_clean();
	}
	//
	void ControlPanel::set_sprayed(){

		  control_panel.set_sprayed();
	}
	//
	void ControlPanel::calculate_thickness(){

		  control_panel.calculate_thickness(ui_.save_name_box->text().toStdString());
	}
	void ControlPanel::compare_results(){

		  control_panel.compare_results(ui_.save_name_box->text().toStdString(),ui_.before_box->text().toStdString(),ui_.after_box->text().toStdString());
	}
	//Perform a laser scan and tell control_panel the fiducial name and whether to set the pose as global origin 
	void ControlPanel::scan(){

		  control_panel.scan(ui_.marker_name_box->text().toStdString(),ui_.set_home->isChecked());
		  	if(ui_.set_home->isChecked()){
				  ui_.set_home->setCheckState(Qt::Unchecked);
				  ui_.scan_button->setText("Scan");
			  }
	}
	//Perform a fake scan using a pointcloud file (.pcl
	void ControlPanel::fake_scan(){

		  control_panel.fscan(ui_.filename_box->text().toStdString(),ui_.auto_localize->isChecked(),ui_.set_home->isChecked(),ui_.marker_name_box->text().toStdString(),ui_.autocrop_box->isChecked());
	}
	//Calculate thickness from the two files specified
	void ControlPanel::thickness(){

		  control_panel.thickness(ui_.before_box->text().toStdString(),ui_.after_box->text().toStdString());
	}
	//Estimate the position correction through registration
	void ControlPanel::do_estimate(){

		  control_panel.estimate(ui_.marker_name_box->text().toStdString());
	}
	//Cluster the high intensity points in the selected area
	void ControlPanel::cluster_1(){
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
		 ui_.auto_localize->setCheckState(Qt::Checked);
		 cluster_index = 0;
		}
	}
	//Set the start point for trajectory generation
	void ControlPanel::start_pt(){
	
		control_panel.start_pt();

	}
	//Generate a trajectory for the arm to spray
	void ControlPanel::gen_trajectory(){

		  traj_size = control_panel.gen_trajectory(ui_.marker_name_box->text().toStdString());
		 //ui_.gen_trajectory->setEnabled(false);

	}
	//Perform a laser scan and set current location as the coordinate frame to transform all future scans in to
	void ControlPanel::localization_scan(){
		
		control_panel.lscan();

	}
	//Testing function
	void ControlPanel::localization_scan_2(){

		control_panel.lscan();

	}

	void ControlPanel::onPTPCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), PTP);
	}
	void ControlPanel::onCPCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), CP);
	}


	void ControlPanel::onTCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), T);
	}
	void ControlPanel::onShutdownCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), SHUTDOWN);
	}


	void ControlPanel::onJointCommand(){

		control_panel.send_joint_command(ui_.j1_box->text().toFloat(),ui_.j2_box->text().toFloat(),ui_.j3_box->text().toFloat(),ui_.j4_box->text().toFloat(),ui_.j5_box->text().toFloat(),ui_.j6_box->text().toFloat());

	}
	void ControlPanel::onStringCommand(){

		control_panel.send_string(ui_.string_box->text().toStdString());

	}
	void ControlPanel::onSpeedCommand(){
		float speed = ui_.speed_box->text().toFloat();
		control_panel.set_speed(speed);

	}
	void ControlPanel::scan_360(){

		control_panel.scan_360();

	}
	
	void ControlPanel::load_traj(){

		traj_size = control_panel.load_traj(ui_.filename_box_5->text().toStdString());

	}
	
	void ControlPanel::do_poses(){

		pose_size = control_panel.calc_poses();
		ui_.pose_button->setText(QString("%1 Poses").arg(QString::number(pose_size)));

	}
	void ControlPanel::pose_step(){
		/*if(pose_size>0){
			if(pose_ctr>=pose_size){
				auto_pose = false;
				pose_ctr = 0;
				ROS_INFO("Pose Trajectory Complete");
			}
			else{
				control_panel.pose_step(pose_ctr);
				++pose_ctr;
			}
			ui_.pose_ctr->setText(QString::number(pose_ctr));
		}else{*/
			control_panel.find_and_move(ui_.step->isChecked());
		//}
	
	}
	void ControlPanel::arm_step(){

		if(traj_ctr>=traj_size){
			ROS_INFO("Arm Trajectory Complete");
			auto_arm = false;
			traj_ctr = 0;
			if(auto_pose) ControlPanel::pose_loop();
		}
		else{
			arm_tgt = control_panel.step(traj_ctr,ui_.step->isChecked(),ui_.fig_box->text().toFloat());
			++traj_ctr;
		}
	}
	void ControlPanel::pose_loop(){
			
		auto_pose = true;
		control_panel.find_and_move(ui_.step->isChecked());
	}
	void ControlPanel::arm_loop(){

		auto_arm = true;
		ControlPanel::arm_step();

	}
	void ControlPanel::soft_stop(){

		auto_arm = false;
		auto_pose = false;
		auto_traj = false;
		control_panel.soft_stop();

	}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_panel::ControlPanel,rviz::Panel)
