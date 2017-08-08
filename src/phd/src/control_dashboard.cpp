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

namespace control_panel
{
	ControlPanel::ControlPanel( QWidget* parent )
	  : rviz::Panel( parent )
	  , control_panel()
	, vel_control()
	{
		//Initial setup
		ui_.setupUi(this);
		//Start the time for the manual operation of the robot
		QTimer* output_timer = new QTimer( this );
		//Initialize control panel
		int ret = control_panel.init();
		//Initialize the manual operation node
		ret = vel_control.init();
		//SIGNAL connections
		connect(ui_.nav_mode_button, SIGNAL(clicked()), this, SLOT(do_nav()));
		connect(ui_.step_button, SIGNAL(clicked()), this, SLOT(do_step()));
		connect(ui_.est_pos, SIGNAL(clicked()), this, SLOT(do_estimate()));
		connect(ui_.exe_path, SIGNAL(clicked()), this, SLOT(exe_nav()));
		connect(ui_.show_nav_button, SIGNAL(clicked()), this, SLOT(show_nav()));
		connect(ui_.scan_button, SIGNAL(clicked()), this, SLOT(do_scan()));
		connect(ui_.scan_button_2, SIGNAL(clicked()), this, SLOT(do_scan_2()));
		connect(ui_.scan_button_3, SIGNAL(clicked()), this, SLOT(do_scan_3()));
		connect(ui_.scan_button_4, SIGNAL(clicked()), this, SLOT(do_scan_4()));
		connect(ui_.localization_button, SIGNAL(clicked()), this, SLOT(localization_scan()));
		connect(ui_.localization_button_2, SIGNAL(clicked()), this, SLOT(localization_scan_2()));
		connect(ui_.cluster1, SIGNAL(clicked()), this, SLOT(cluster_1()));
		connect(ui_.start_point, SIGNAL(clicked()), this, SLOT(start_pt()));
		connect(ui_.gen_trajectory, SIGNAL(clicked()), this, SLOT(gen_trajectory()));
		connect(ui_.fwdButton, SIGNAL(pressed()), this, SLOT(fwd_vel()));
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
		//connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
		//Index for keeping track of which marker point is being clustered
		cluster_index = 0;
	}

	ControlPanel::~ControlPanel(){}

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
	//Perform and scan using the nodding head laser
	void ControlPanel::do_scan(){

		  control_panel.scan();
	}
	//Estimate the position correction through registration
	void ControlPanel::do_estimate(){

		  control_panel.estimate();
	}
	//Testing function, step through the arm strajectory pose
	void ControlPanel::do_step(){

		  control_panel.step();
	}
	void ControlPanel::do_scan_2(){

		  control_panel.fscan(2);
	}
	void ControlPanel::do_scan_3(){

		  control_panel.fscan(3);
	}
	void ControlPanel::do_scan_4(){

		  control_panel.fscan(4);
	}
	//Cluster the high intensity points in the selected area
	void ControlPanel::cluster_1(){
		//Call the cluster function in control_panel and pass it which marker point to be clustered, then update the text in the panel
		control_panel.cluster(cluster_index);
		if(cluster_index == 0){
		 ui_.cluster1->setText("Cluster Pt. 2");
		 cluster_index+=1;
		}
		else if(cluster_index == 1){
		 ui_.cluster1->setText("Cluster Pt. 3");
		 ++cluster_index;
		}
		else if(cluster_index == 2){
		 ui_.cluster1->setText("Cluster Pt. 1");
		 ui_.marker_info->setText("Marker Recorded");
		 cluster_index = 0;
		}
	}
	//Set the start point for trajectory generation
	void ControlPanel::start_pt(){
	
		control_panel.start_pt();

	}
	//Generate a trajectory for the arm to spray
	void ControlPanel::gen_trajectory(){

		  control_panel.gen_trajectory();
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
	

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_panel::ControlPanel,rviz::Panel)
