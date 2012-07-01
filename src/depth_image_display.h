/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_IMAGE_DISPLAY_H
#define RVIZ_IMAGE_DISPLAY_H

#include <QObject>

#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/properties/forwards.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>

#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include "image_view.h"

#include <rviz/panel_dock_widget.h>

namespace ros
{
class NodeHandle;
}
namespace rviz
{
class RenderPanel;
class PanelDockWidget;
class PointCloudTransformer;
}
namespace image_transport
{
class ImageTransport;
}

using namespace rviz;
using namespace message_filters::sync_policies;

namespace depth_image_plugin
{

typedef boost::shared_ptr<rviz::PointCloudTransformer> PointCloudTransformerPtr;

/**
 * \class DepthImageDisplay
 *
 */
class DepthImageDisplay : public rviz::Display
{
Q_OBJECT
public:
  DepthImageDisplay();
  virtual ~DepthImageDisplay();

  virtual void onInitialize();

  const std::string& getDepthMapTopic()
  {
    return depth_topic_;
  }


  // Getter and setter for plugin properties:
  void setDepthMapTopic(const std::string& topic);

  const std::string& getDepthMapTransport()
  {
    return depth_transport_;
  }

  void setDepthMapTransport(const std::string& transport);

  const std::string& getRGBTopic()
  {
    return rgb_topic_;
  }

  void setRGBTopic(const std::string& topic);

  const std::string& getRGBTransport()
  {
    return rgb_transport_;
  }

  void setRGBTransport(const std::string& transport);

  bool isPointCloudVisible();

  void setPointCloudVisible(bool visible);

  bool isColoredPointCloudEnabled()
  {
    return coloredpointcloud_visible_;
  }
  void setColoredPointCloudEnabled(bool visible);

  bool isDepthMapVisible();

  void setDepthMapVisible(bool visible);

  const float& getDepthFilter()
  {
    return maxDepth_;
  }

  void setDepthFilter(const float depth);

  void createPointCloudProperties();

  void setPointCloudStyle(int style);

  int getPointCloudStyle()
  {
    return pointcloud_style_;
  }
  void setPointCloudBillboardSize(float size);

  float getPointCloudBillboardSize()
  {
    return pointcloud_billboard_size_;
  }

  float getPointCloudAlpha()
  {
    return pointcloud_alpha_;
  }

  void setPointCloudAlpha(float alpha);

  // Overrides from Display
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  /** Set the incoming message queue size. */
  void setQueueSize(int size);
  int getQueueSize();

protected Q_SLOTS:
  /** Enables or disables this display via its DisplayWrapper. */
  void setWrapperEnabled(bool enabled);

protected:
  typedef std::vector<rviz::PointCloud::Point> V_Point;

  void callbackDepthRGB(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg,
                        const sensor_msgs::CameraInfoConstPtr& info_msg);

  void callbackDepth(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  void processMessages(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::CameraInfoConstPtr& info_msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void depthMapWindowEnabled(bool enabled);

  virtual void fixedFrameChanged();

  void subscribe();
  void unsubscribe();

  void clear();
  void updateStatus();

  void onTransportEnumOptions(V_string& choices);

  // ROS stuff

  boost::shared_ptr<image_transport::SubscriberFilter> sub_depth_, sub_rgb_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > sub_info_;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyDepthRGBCameraInfo;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyDepthCameraInfo;
  typedef message_filters::Synchronizer<SyncPolicyDepthRGBCameraInfo> SynchronizerDepthRGBCameraInfo;
  typedef message_filters::Synchronizer<SyncPolicyDepthCameraInfo> SynchronizerDepthCameraInfo;

  boost::shared_ptr<SynchronizerDepthRGBCameraInfo> syncDepthRGBCameraInfo_;
  boost::shared_ptr<SynchronizerDepthCameraInfo> syncDepthCameraInfo_;

  image_transport::ImageTransport rgb_it_, depth_it_;

  std::string depth_topic_;
  std::string rgb_topic_;
  std::string depth_transport_;
  std::string rgb_transport_;
  std::string frame_;

  ros::NodeHandle* nh_;

  boost::mutex mutex_;

  // RVIZ properties
  ROSTopicStringPropertyWPtr depth_topic_property_;
  EditEnumPropertyWPtr depth_transport_property_;

  rviz::BoolPropertyWPtr coloredpointcloud_enabled_property_;
  bool coloredpointcloud_visible_;
  ROSTopicStringPropertyWPtr rgb_topic_property_;
  EditEnumPropertyWPtr rgb_transport_property_;

  rviz::BoolPropertyWPtr pointcloud_enabled_property_;
  bool pointcloud_visible_;

  rviz::BoolPropertyWPtr depthmap_enabled_property_;
  bool depthmap_visible_;

  rviz::FloatPropertyWPtr maxDepth_property_;
  float maxDepth_;

  rviz::FloatPropertyWPtr billboard_size_property_;
  rviz::FloatPropertyWPtr alpha_property_;
  rviz::EnumPropertyWPtr style_property_;
  int pointcloud_style_;
  float pointcloud_billboard_size_;
  float pointcloud_alpha_;

  // Ogre scene graph
  Ogre::SceneManager* image_scene_manager_;
  Ogre::SceneNode* image_scene_node_;
  Ogre::SceneNode* pc_scene_node_;

  rviz::PointCloud* cloud_;
  bool newCloud_;
  V_Point new_points_;

  ImageView depthMapView_;

  // Panel
  RenderPanel* render_panel_;
  PanelDockWidget* panel_container_;
  IntPropertyWPtr queue_size_property_;

  uint32_t image_count_;
  int queue_size_;

  image_geometry::PinholeCameraModel* cameraModel_;

  std::vector<std::string> transport_plugin_types_;

  ros::AsyncSpinner spinner_;
  ros::CallbackQueue cbqueue_;
};

} // namespace depth_image_plugin

#endif
