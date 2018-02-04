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
#define JOINT 3
#define STRING 4
#define SHUTDOWN 5

namespace control_panel
{
	ControlPanel::ControlPanel( QWidget* parent )
	  : rviz::Panel( parent )
	  , control_panel()
	, vel_control()
	{
		//Initial setup
		ui_.setupUi(this);
		//pose sub
		pose_sub_ = nh_.subscribe("arm_pose", 1, &ControlPanel::poseCB, this);
		QTimer* spin_timer = new QTimer( this );
		//Initialize control panel
		int ret = control_panel.init();
		//Initialize the manual operation node
		ret = vel_control.init();
		//SIGNAL connections
		connect(ui_.nav_mode_button, SIGNAL(clicked()), this, SLOT(do_nav()));
		connect(ui_.step_button, SIGNAL(clicked()), this, SLOT(do_step()));
		connect(ui_.est_pos, SIGNAL(clicked()), this, SLOT(do_estimate()));
		connect(ui_.exe_path, SIGNAL(clicked()), this, SLOT(exe_nav()));
		connect(ui_.exe_poses, SIGNAL(clicked()), this, SLOT(pose_step()));
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


		connect(spin_timer, SIGNAL(timeout()), this, SLOT(rosSpinner()));
		/*connect(ui_.fwdButton, SIGNAL(pressed()), this, SLOT(fwd_vel()));
		connect(ui_.revButton, SIGNAL(pressed()), this, SLOT(rev_vel()));
		connect(ui_.turnLeft, SIGNAL(pressed()), this, SLOT(left_vel()));
		connect(ui_.turnRight, SIGNAL(pressed()), this, SLOT(right_vel()));
		connect(ui_.armFwd, SIGNAL(pressed()), this, SLOT(arm_fwd_vel()));
		connect(ui_.armLeft, SIGNAL(pressed()), this, SLOT(arm_left_vel()));
		connect(ui_.armRight, SIGNAL(pressed()), this, SLOT(arm_right_vel()));
		connect(ui_.armRev, SIGNAL(pressed()), this, SLOT(arm_rev_vel()));
		connect(ui_.armUp, SIGNAL(pressed()), this, SLOT(arm_up_vel()));
		connect(ui_.armDown, SIGNAL(pressed()), this, SLOT(arm_down_vel()));
		connect(ui_.roll_neg, SIGNAL(pressed()), this, SLOT(roll_vel_neg()));
		connect(ui_.roll_pos, SIGNAL(pressed()), this, SLOT(roll_vel_pos()));
		connect(ui_.pitch_neg, SIGNAL(pressed()), this, SLOT(pitch_vel_neg()));
		connect(ui_.pitch_pos, SIGNAL(pressed()), this, SLOT(pitch_vel_pos()));
		connect(ui_.yaw_neg, SIGNAL(pressed()), this, SLOT(yaw_vel_neg()));
		connect(ui_.yaw_pos, SIGNAL(pressed()), this, SLOT(yaw_vel_pos()));
		connect(ui_.fwdButton, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.revButton, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.turnLeft, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.turnRight, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.armFwd, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.armLeft, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.armRight, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.armRev, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.armUp, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.armDown, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.roll_neg, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.roll_pos, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.pitch_neg, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.pitch_pos, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.yaw_neg, SIGNAL(released()), this, SLOT(clear_vel()));
		connect(ui_.yaw_pos, SIGNAL(released()), this, SLOT(clear_vel()));
*/
		  connect(ui_.launch_node_button, SIGNAL(clicked()), this, SLOT(onLaunchNode()));
		  connect(ui_.shutdown_button, SIGNAL(clicked()), this, SLOT(onShutdownCommand()));
		  connect(ui_.clear_error_button, SIGNAL(clicked()), this, SLOT(onClearErrorCommand()));
		  connect(ui_.ptp_command_button, SIGNAL(clicked()), this, SLOT(onPTPCommand()));
		  connect(ui_.cp_command_button, SIGNAL(clicked()), this, SLOT(onCPCommand()));
		  connect(ui_.speed_button, SIGNAL(clicked()), this, SLOT(onSpeedCommand()));
		  connect(ui_.string_button, SIGNAL(clicked()), this, SLOT(onStringCommand()));
		  connect(ui_.joint_command_button, SIGNAL(clicked()), this, SLOT(onJointCommand()));
		//connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
		//Index for keeping track of which marker point is being clustered
		cluster_index = 0;
		pose_ctr = 0;
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
	}

	void ControlPanel::rosSpinner(){
		ros::spinOnce();
	}

	//Move laser scanner to its navigation position (horizontal) or whatever position is specified by the user in the text box
	void ControlPanel::do_nav(){
		float pos = ui_.pos_box->text().toFloat();
		  control_panel.nav_mode(pos);
	}
	//Tell the robot to drive to the desired goal by publishing the goal location
	void ControlPanel::exe_nav(){

		  control_panel.exe_nav();
	}
	//Show the robot's goal location and path
	void ControlPanel::show_nav(){

		  control_panel.show_nav();
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

		  control_panel.fscan(ui_.filename_box->text().toStdString(),ui_.auto_localize->isChecked(),ui_.marker_name_box->text().toStdString());
	}
	//Calculate thickness from the two files specified
	void ControlPanel::thickness(){

		  control_panel.thickness(ui_.before_box->text().toStdString(),ui_.after_box->text().toStdString());
	}
	//Estimate the position correction through registration
	void ControlPanel::do_estimate(){

		  control_panel.estimate(ui_.marker_name_box->text().toStdString());
	}
	//Testing function, step through the arm's trajectory pose
	void ControlPanel::do_step(){

		  control_panel.step(ui_.step->isChecked());
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

		  control_panel.gen_trajectory(ui_.marker_name_box->text().toStdString());
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
	//A soft E-Stop
	void ControlPanel::clear_vel(){

		vel_control.stop();

	}
	//The remaining functions are for manual driving the robot
	void ControlPanel::fwd_vel(){

		vel_control.move(FORWARD,SPEED,HUSKY);

	}

	void ControlPanel::rev_vel(){

		vel_control.move(REVERSE,SPEED,HUSKY);

	}
	void ControlPanel::left_vel(){

		vel_control.move(LEFT,SPEED,HUSKY);

	}
	void ControlPanel::right_vel(){

		vel_control.move(RIGHT,SPEED,HUSKY);

	}
	void ControlPanel::arm_fwd_vel(){

		vel_control.move(FORWARD,SPEED,ARM);

	}
	void ControlPanel::arm_left_vel(){

		vel_control.move(LEFT,SPEED,ARM);

	}
	void ControlPanel::arm_right_vel(){

		vel_control.move(RIGHT,SPEED,ARM);

	}
	void ControlPanel::arm_rev_vel(){

		vel_control.move(REVERSE,SPEED,ARM);

	}
	void ControlPanel::arm_up_vel(){

		vel_control.move(UP,SPEED,ARM);

	}
	void ControlPanel::arm_down_vel(){

		vel_control.move(DOWN,SPEED,ARM);

	}
	void ControlPanel::roll_vel_neg(){

		vel_control.move(ROLL_DOWN,SPEED,ARM);

	}
	void ControlPanel::roll_vel_pos(){

		vel_control.move(ROLL_UP,SPEED,ARM);

	}
	void ControlPanel::pitch_vel_neg(){

		vel_control.move(PITCH_DOWN,SPEED,ARM);

	}
	void ControlPanel::pitch_vel_pos(){

		vel_control.move(PITCH_UP,SPEED,ARM);

	}
	void ControlPanel::yaw_vel_neg(){

		vel_control.move(YAW_DOWN,SPEED,ARM);

	}
	void ControlPanel::yaw_vel_pos(){

		vel_control.move(YAW_UP,SPEED,ARM);

	}

	void ControlPanel::onPTPCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), PTP);
	}
	void ControlPanel::onCPCommand(){

		  control_panel.send_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat(), CP);
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
	void ControlPanel::scan_360(){

		control_panel.scan_360();

	}
	
	void ControlPanel::load_traj(){

		control_panel.load_traj(ui_.filename_box_5->text().toStdString());

	}
	
	void ControlPanel::do_poses(){

		pose_nums = control_panel.calc_poses();
		ui_.pose_button->setText(QString("%1 Poses").arg(QString::number(pose_nums)));

	}
	void ControlPanel::pose_step(){

		control_panel.pose_step();
		++pose_ctr;
		ui_.pose_ctr->setText(QString::number(pose_ctr));

	}
	

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_panel::ControlPanel,rviz::Panel)
