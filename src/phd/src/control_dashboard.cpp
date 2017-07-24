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
		ui_.setupUi(this);
		QTimer* output_timer = new QTimer( this );
		int ret = control_panel.init();
		ret = vel_control.init();
		//SIGNAL connections
		connect(ui_.nav_mode_button, SIGNAL(clicked()), this, SLOT(do_nav()));
		connect(ui_.step_button, SIGNAL(clicked()), this, SLOT(do_step()));
		connect(ui_.scan_button, SIGNAL(clicked()), this, SLOT(do_scan()));
		connect(ui_.scan_button_2, SIGNAL(clicked()), this, SLOT(do_scan_2()));
		connect(ui_.scan_button_3, SIGNAL(clicked()), this, SLOT(do_scan_3()));
		connect(ui_.scan_button_4, SIGNAL(clicked()), this, SLOT(do_scan_4()));
		connect(ui_.localization_button, SIGNAL(clicked()), this, SLOT(localization_scan()));
		connect(ui_.localization_button_2, SIGNAL(clicked()), this, SLOT(localization_scan_2()));
		connect(ui_.cluster1, SIGNAL(clicked()), this, SLOT(cluster_1()));
		connect(ui_.trajectory_1, SIGNAL(clicked()), this, SLOT(trajectory()));
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

		cluster_index = 0;
		trajectory_index = 0;
	}

	ControlPanel::~ControlPanel(){}

	void ControlPanel::do_nav(){
		float pos = ui_.pos_box->text().toFloat();
		  control_panel.nav_mode(pos);
	}
	void ControlPanel::do_scan(){

		  control_panel.scan();
	}
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
	void ControlPanel::cluster_1(){

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
	void ControlPanel::trajectory(){
	
		control_panel.trajectory(trajectory_index);
		/*if(trajectory_index == 0){
		 ui_.trajectory_1->setText("Trajectory Pt. 2");
		 trajectory_index+=1;
		}
		else if(trajectory_index == 1){
		 ui_.trajectory_1->setText("Trajectory Pt. 3");
		 ++trajectory_index;
		}
		else if(trajectory_index == 2){
		 ui_.trajectory_1->setText("Trajectory Pt. 1");
		 ui_.gen_trajectory->setEnabled(true);
		 trajectory_index = 0;
		}*/
	}
	void ControlPanel::gen_trajectory(){

		  control_panel.gen_trajectory();
		 //ui_.gen_trajectory->setEnabled(false);

	}
	void ControlPanel::localization_scan(){


	}
	void ControlPanel::localization_scan_2(){

		control_panel.lscan();

	}
	void ControlPanel::clear_vel(){

		vel_control.stop();

	}
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
