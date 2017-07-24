/**
 * @file /include/test_panel/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef vel_control_AQNODE_HPP_
#define vel_control_AQNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
// ROS
#include <ros/ros.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include "std_msgs/String.h"
#include <sstream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace control_panel {
namespace husky_control {


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:

	QNode()
	{}
	virtual ~QNode();
	void run();
	bool init();
	void move(int dir, float speed, int dev);
	void stop();


Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  ros::NodeHandle nh_;
  ros::Publisher pub;
	bool moving_base;
	bool moving_arm;
	int direction;
	float arm_speed;
	float base_speed;
};
}
}  // namespace 

#endif /* test_panel_QNODE_HPP_ */
