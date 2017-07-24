
//////////////////////////////// DOCUMENTATION /////////////////////////////////
//
// Fork of the rviz::SelectionTool:
// Drag with the left button to select objects in the 3D scene.
// Hold the Alt key to change viewpoint as in the Move tool.
// Additionally publishes selected points on /selected_points topic.
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2013 Robotics & Biology Laboratory (RBO) TU Berlin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


#ifndef MARKER_SELECTOR_H
#define MARKER_SELECTOR_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include "rviz/tool.h"

# include <QCursor>
# include <QObject>
#endif

#include "rviz/default_plugin/tools/selection_tool.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/extract_indices.h>

namespace rviz_plugin_marker_selector
{

class marker_selector;

class marker_selector : public rviz::SelectionTool
{
Q_OBJECT
public:
  marker_selector();
  virtual ~marker_selector();

  /*
   * Hooks on rviz::SelectionTool::processMouseEvent() to get and publish
   * selected points
   */
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );


public Q_SLOTS:
  /*
   * Creates the ROS topic
   */
  void updateTopic();

  void PointCloudsCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

protected:

  int _processSelectedAreaAndFindPoints();
  ros::NodeHandle nh_;
  ros::Publisher rviz_selected_pub_;
  ros::Subscriber pc_subs_;

  std::string tf_frame_;
  std::string rviz_cloud_topic_;
  std::string subs_cloud_topic_;
  bool selecting_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_;

  int num_selected_points_;
};
} // end namespace rviz_plugin_marker_selector

#endif // SELECTED_POINTS_PUBLISHER_H

////////////////////////////////////////////////////////////////////////////////

