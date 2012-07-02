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

#include "depth_image_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/window_manager_interface.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/panel_dock_widget.h"
#include "rviz/display_wrapper.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>

#include "image_transport/camera_common.h"

// opencv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>

#include <sstream>

using namespace rviz;
namespace enc = sensor_msgs::image_encodings;

namespace depth_image_plugin
{

enum PointCloudStyles
{
  Points, ///< Points -- points are drawn as a fixed size in 2d space, ie. always 1 pixel on screen
  Billboards, ///< Billboards -- points are drawn as camera-facing quads in 3d space
  BillboardSpheres, ///< Billboard "spheres" -- cam-facing tris with a pixel shader that causes them to look like spheres
  Boxes, ///< Boxes -- Actual 3d cube geometry

  StyleCount,
};

DepthImageDisplay::DepthImageDisplay() :
    rviz::Display()
  , rgb_it_(update_nh_)
  , depth_it_( update_nh_)
  , depth_transport_("raw")
  , rgb_transport_("raw")
  , nh_(new ros::NodeHandle(update_nh_))
  , coloredpointcloud_visible_(true)
  , pointcloud_visible_(true)
  , depthmap_visible_(true)
  , maxDepth_(10.0f)
  , pointcloud_style_(Billboards)
  , pointcloud_billboard_size_(0.01)
  , pointcloud_alpha_(1.0f)
  , newCloud_(false)
  , depthMapView_()
  , panel_container_(0)
  , image_count_(0)
  , queue_size_(2)
  , cameraModel_(new image_geometry::PinholeCameraModel)
  , spinner_(1, &cbqueue_)
{
  // get vector of loadable image transport plugins
  transport_plugin_types_ = rgb_it_.getLoadableTransports();

  cloud_ = new rviz::PointCloud();

}

void DepthImageDisplay::onInitialize()
{

  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "Depth Image Display" << count++;
    image_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
  }

  image_scene_node_ = image_scene_manager_->getRootSceneNode()->createChildSceneNode();

  depthMapView_.initialize(image_scene_node_);

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);

  render_panel_->resize(640, 480);
  render_panel_->initialize(image_scene_manager_, vis_manager_);

  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  if (wm)
  {
    panel_container_ = wm->addPane(name_, render_panel_);
  }
  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  pc_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  pc_scene_node_->attachObject(cloud_);

  threaded_nh_.setCallbackQueue(&cbqueue_);
  spinner_.start();

}

DepthImageDisplay::~DepthImageDisplay()
{

  spinner_.stop();

  unsubscribe();

  scene_manager_->destroySceneNode(pc_scene_node_->getName());
  if (cloud_)
    delete cloud_;

  if (render_panel_)
  {
    if (panel_container_)
    {
      delete panel_container_;
    }
    else
    {
      delete render_panel_;
    }
  }

  image_scene_node_->getParentSceneNode()->removeAndDestroyChild(image_scene_node_->getName());

  if (nh_)
    delete (nh_);

  if (cameraModel_)
    delete (cameraModel_);

}

void DepthImageDisplay::setWrapperEnabled(bool enabled)
{
  // Have to use the DisplayWrapper disable function so the checkbox
  // gets checked or unchecked, since it owns the "enabled" property.
  DisplayWrapper* wrapper = vis_manager_->getDisplayWrapper(this);
  if (wrapper != NULL)
  {
    wrapper->setEnabled(enabled);
  }
}

void DepthImageDisplay::depthMapWindowEnabled(bool enabled)
{
  if (enabled)
  {
    if (render_panel_->parentWidget() == 0)
    {
      render_panel_->show();
    }
    else
    {
      panel_container_->show();
    }

    render_panel_->getRenderWindow()->setActive(true);
  }
  else
  {
    render_panel_->getRenderWindow()->setActive(false);

    if (render_panel_->parentWidget() == 0)
    {
      if (render_panel_->isVisible())
      {
        render_panel_->hide();
      }
    }
    else
    {
      if (panel_container_->isVisible())
      {
        panel_container_->close();
      }
    }
  }
}

void DepthImageDisplay::onEnable()
{
  subscribe();

  depthMapWindowEnabled(true);

  setWrapperEnabled(true);
}

void DepthImageDisplay::onDisable()
{

  unsubscribe();

  clear();

  depthMapWindowEnabled(false);

  setWrapperEnabled(false);
}

void DepthImageDisplay::subscribe()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (!isEnabled())
  {
    return;
  }

  depthMapView_.clear();

  try

  {
    syncDepthCameraInfo_.reset(new SynchronizerDepthCameraInfo(SyncPolicyDepthCameraInfo(queue_size_)));
    syncDepthRGBCameraInfo_.reset(new SynchronizerDepthRGBCameraInfo(SyncPolicyDepthRGBCameraInfo(queue_size_)));
    sub_depth_.reset(new image_transport::SubscriberFilter());
    sub_rgb_.reset(new image_transport::SubscriberFilter());
    sub_info_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

    if (!depth_topic_.empty())
    {

      if (rgb_topic_.empty() || (!coloredpointcloud_visible_))
      {
        sub_depth_->subscribe(depth_it_, depth_topic_, 1, image_transport::TransportHints(depth_transport_));
        std::string info_topic = image_transport::getCameraInfoTopic(depth_topic_);

        sub_info_->subscribe(*nh_, info_topic, 1);

        syncDepthCameraInfo_->connectInput(*sub_depth_, *sub_info_);
        syncDepthCameraInfo_->registerCallback(boost::bind(&DepthImageDisplay::callbackDepth, this, _1, _2));
      }
      else
      {
        sub_depth_->subscribe(depth_it_, depth_topic_, 1, image_transport::TransportHints(depth_transport_));
        sub_rgb_->subscribe(rgb_it_, rgb_topic_, 1, image_transport::TransportHints(rgb_transport_));

        std::string info_topic = image_transport::getCameraInfoTopic(depth_topic_);

        sub_info_->subscribe(*nh_, info_topic, 1);

        syncDepthRGBCameraInfo_->connectInput(*sub_depth_, *sub_rgb_, *sub_info_);
        syncDepthRGBCameraInfo_->registerCallback(boost::bind(&DepthImageDisplay::callbackDepthRGB, this, _1, _2, _3));
      }

    }
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }



}

void DepthImageDisplay::unsubscribe()
{
  clear();

  boost::mutex::scoped_lock lock(mutex_);

  try

  {
    syncDepthCameraInfo_.reset(new SynchronizerDepthCameraInfo(SyncPolicyDepthCameraInfo(queue_size_)));
    syncDepthRGBCameraInfo_.reset(new SynchronizerDepthRGBCameraInfo(SyncPolicyDepthRGBCameraInfo(queue_size_)));
    sub_depth_.reset(new image_transport::SubscriberFilter());
    sub_rgb_.reset(new image_transport::SubscriberFilter());
    sub_info_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error unsubscribing: ") + e.what());
  }

}

void DepthImageDisplay::setQueueSize(int size)
{
  if (size != queue_size_)
  {
    queue_size_ = size;
    propertyChanged(queue_size_property_);
  }
}

int DepthImageDisplay::getQueueSize()
{
  return queue_size_;
}

void DepthImageDisplay::setDepthMapTopic(const std::string& topic)
{
  unsubscribe();

  depth_topic_ = topic;
  clear();

  subscribe();

  propertyChanged(depth_topic_property_);
}

void DepthImageDisplay::setDepthMapTransport(const std::string& transport)
{
  depth_transport_ = transport;

  setDepthMapTopic(depth_topic_);

  propertyChanged(depth_transport_property_);
}

void DepthImageDisplay::setRGBTopic(const std::string& topic)
{
  unsubscribe();

  rgb_topic_ = topic;
  clear();

  subscribe();

  propertyChanged(rgb_topic_property_);
}

void DepthImageDisplay::setRGBTransport(const std::string& transport)
{
  rgb_transport_ = transport;

  setRGBTopic(rgb_topic_);

  propertyChanged(rgb_transport_property_);
}

void DepthImageDisplay::setColoredPointCloudEnabled(bool enabled)
{

  if (!enabled)
  {
    hideProperty(rgb_transport_property_);
    hideProperty(rgb_topic_property_);
  }
  else
  {
    showProperty(rgb_transport_property_);
    showProperty(rgb_topic_property_);
  }

  coloredpointcloud_visible_ = enabled;

  setDepthMapTopic(depth_topic_);

  propertyChanged(rgb_transport_property_);
}

bool DepthImageDisplay::isPointCloudVisible()
{
  return pointcloud_visible_;
}

void DepthImageDisplay::setPointCloudVisible(bool visible)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (pointcloud_visible_ == visible)
    return;

  pointcloud_visible_ = visible;

  if (!pointcloud_visible_)
  {
    cloud_->clear();

    hideProperty(rgb_transport_property_);
    hideProperty(rgb_topic_property_);
    hideProperty(billboard_size_property_);
    hideProperty(alpha_property_);
    hideProperty(style_property_);
    hideProperty(coloredpointcloud_enabled_property_);
  }
  else
  {
    showProperty(billboard_size_property_);
    showProperty(alpha_property_);
    showProperty(style_property_);
    showProperty(coloredpointcloud_enabled_property_);

    if (!coloredpointcloud_visible_)
    {
      hideProperty(rgb_transport_property_);
      hideProperty(rgb_topic_property_);
    }
    else
    {
      showProperty(rgb_transport_property_);
      showProperty(rgb_topic_property_);
    }
  }

  propertyChanged(pointcloud_enabled_property_);

}

bool DepthImageDisplay::isDepthMapVisible()
{
  return depthmap_visible_;
}

void DepthImageDisplay::setDepthMapVisible(bool visible)
{

  boost::mutex::scoped_lock lock(mutex_);

  depthmap_visible_ = visible;

  depthMapWindowEnabled(visible);

  propertyChanged(depthmap_enabled_property_);

}

void DepthImageDisplay::setDepthFilter(const float depth)
{
  boost::mutex::scoped_lock lock(mutex_);

  maxDepth_ = depth;
  depthMapView_.setMaxDepth(depth);

  propertyChanged(depthmap_enabled_property_);
}

void DepthImageDisplay::setPointCloudAlpha(float alpha)
{
  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_alpha_ = alpha;

  cloud_->setAlpha(alpha);

  propertyChanged(alpha_property_);
}

void DepthImageDisplay::setPointCloudStyle(int style)
{
  boost::mutex::scoped_lock lock(mutex_);

  ROS_ASSERT( style < StyleCount);

  pointcloud_style_ = style;

  rviz::PointCloud::RenderMode mode = rviz::PointCloud::RM_POINTS;
  if (style == Billboards)
  {
    mode = rviz::PointCloud::RM_BILLBOARDS;
  }
  else if (style == BillboardSpheres)
  {
    mode = rviz::PointCloud::RM_BILLBOARD_SPHERES;
  }
  else if (style == Boxes)
  {
    mode = rviz::PointCloud::RM_BOXES;
  }

  if (style == Points)
  {
    hideProperty(billboard_size_property_);
  }
  else
  {
    showProperty(billboard_size_property_);
  }

  cloud_->setRenderMode(mode);

  propertyChanged(style_property_);

  causeRender();
}

void DepthImageDisplay::setPointCloudBillboardSize(float size)
{
  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_billboard_size_ = size;

  cloud_->setDimensions(size, size, size);

  propertyChanged(billboard_size_property_);

  causeRender();
}

void DepthImageDisplay::clear()
{

  boost::mutex::scoped_lock lock(mutex_);

  depthMapView_.clear();
  cloud_->clear();

  setStatus(status_levels::Warn, "Image", "No Image received");

  if (render_panel_->getCamera())
  {
    render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
  }
}

void DepthImageDisplay::updateStatus()
{
  if (image_count_ == 0)
  {
    setStatus(status_levels::Warn, "Image", "No image received");
  }
  else
  {
    std::stringstream ss;
    ss << image_count_ << " images received";
    setStatus(status_levels::Ok, "Image", ss.str());
  }
}

void DepthImageDisplay::update(float wall_dt, float ros_dt)
{

  boost::mutex::scoped_lock lock(mutex_);

  updateStatus();

  try
  {
    if (depthmap_visible_)
    {
      depthMapView_.draw(render_panel_->width(), render_panel_->height());
      render_panel_->getRenderWindow()->update();
    }

    if (newCloud_ && pointcloud_visible_)
    {
      cloud_->clear();
      cloud_->addPoints(&new_points_.front(), new_points_.size());
      newCloud_ = false;
    }

  }
  catch (UnsupportedImageViewEncoding& e)
  {
    setStatus(status_levels::Error, "Image", e.what());
  }
}

void DepthImageDisplay::onTransportEnumOptions(V_string& choices)
{
  choices.push_back("raw");

  // Loop over all current ROS topic names
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it)
  {
    // If the beginning of this topic name is the same as depth_topic_,
    // and the whole string is not the same,
    // and the next character is /
    // and there are no further slashes from there to the end,
    // then consider this a possible transport topic.
    const ros::master::TopicInfo& ti = *it;
    const std::string& depth_topic_name = ti.name;
    if (depth_topic_name.find(depth_topic_) == 0 && depth_topic_name != depth_topic_
        && depth_topic_name[depth_topic_.size()] == '/'
        && depth_topic_name.find('/', depth_topic_.size() + 1) == std::string::npos)
    {
      std::string transport_type = depth_topic_name.substr(depth_topic_.size() + 1);
      std::string transport_plugin = "image_transport/" + transport_type;

      // If the transport type string found above is in the vector of
      // supported transport type plugins, add it to the list.
      if (std::find(transport_plugin_types_.begin(), transport_plugin_types_.end(), transport_plugin)
          != transport_plugin_types_.end())
      {
        choices.push_back(transport_type);
      }
    }
  }
}

void DepthImageDisplay::createProperties()
{
  // Depth map transport
  depth_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>(
      "Depth Map Topic", property_prefix_, boost::bind(&DepthImageDisplay::getDepthMapTopic, this),
      boost::bind(&DepthImageDisplay::setDepthMapTopic, this, _1), parent_category_, this);
  setPropertyHelpText(depth_topic_property_, "sensor_msgs::Image topic to subscribe to.");
  ROSTopicStringPropertyPtr depth_topic_prop = depth_topic_property_.lock();
  depth_topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Image>());

  depth_transport_property_ = property_manager_->createProperty<EditEnumProperty>(
      "Depth Map Transport Hint", property_prefix_, boost::bind(&DepthImageDisplay::getDepthMapTransport, this),
      boost::bind(&DepthImageDisplay::setDepthMapTransport, this, _1), parent_category_, this);
  EditEnumPropertyPtr ee_prop = depth_transport_property_.lock();
  ee_prop->setOptionCallback(boost::bind(&DepthImageDisplay::onTransportEnumOptions, this, _1));

  // Queue size
  queue_size_property_ = property_manager_->createProperty<IntProperty>(
      "Queue Size", property_prefix_, boost::bind(&DepthImageDisplay::getQueueSize, this),
      boost::bind(&DepthImageDisplay::setQueueSize, this, _1), parent_category_, this);
  setPropertyHelpText(
      queue_size_property_,
      "Advanced: set the size of the incoming Image message queue.  Increasing this is useful if your incoming TF data is delayed significantly from your Image data, but it can greatly increase memory usage if the messages are big.");

  // Max depth value
  maxDepth_property_ = property_manager_->createProperty<rviz::FloatProperty>(
      "Max. Depth Filter", property_prefix_, boost::bind(&DepthImageDisplay::getDepthFilter, this),
      boost::bind(&DepthImageDisplay::setDepthFilter, this, _1), parent_category_, this);
  setPropertyHelpText(maxDepth_property_, "Define a maximum depth");

  // Enable Dpeth map
  depthmap_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty>(
      "Depth Map Enabled", property_prefix_, boost::bind(&DepthImageDisplay::isDepthMapVisible, this),
      boost::bind(&DepthImageDisplay::setDepthMapVisible, this, _1), parent_category_, this);
  setPropertyHelpText(depthmap_enabled_property_, "Whether to display the 2D depth map.");

  // Show point cloud
  pointcloud_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty>(
      "Point Cloud Enabled", property_prefix_, boost::bind(&DepthImageDisplay::isPointCloudVisible, this),
      boost::bind(&DepthImageDisplay::setPointCloudVisible, this, _1), parent_category_, this);
  setPropertyHelpText(pointcloud_enabled_property_, "Whether to display a corresponding point cloud.");

  // pointcloud style
  style_property_ = property_manager_->createProperty<rviz::EnumProperty>(
      "Style", property_prefix_, boost::bind(&DepthImageDisplay::getPointCloudStyle, this),
      boost::bind(&DepthImageDisplay::setPointCloudStyle, this, _1), parent_category_, this);
  setPropertyHelpText(style_property_, "Rendering mode to use, in order of computational complexity.");
  rviz::EnumPropertyPtr enum_prop = style_property_.lock();
  enum_prop->addOption("Points", Points);
  enum_prop->addOption("Billboards", Billboards);
  enum_prop->addOption("Billboard Spheres", BillboardSpheres);
  enum_prop->addOption("Boxes", Boxes);

  billboard_size_property_ = property_manager_->createProperty<rviz::FloatProperty>(
      "Billboard Size", property_prefix_, boost::bind(&DepthImageDisplay::getPointCloudBillboardSize, this),
      boost::bind(&DepthImageDisplay::setPointCloudBillboardSize, this, _1), parent_category_, this);
  setPropertyHelpText(billboard_size_property_,
                      "Length, in meters, of the side of each billboard (or face if using the Boxes style).");
  rviz::FloatPropertyPtr float_prop = billboard_size_property_.lock();
  float_prop->setMin(0.0001);

  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>(
      "Alpha", property_prefix_, boost::bind(&DepthImageDisplay::getPointCloudAlpha, this),
      boost::bind(&DepthImageDisplay::setPointCloudAlpha, this, _1), parent_category_, this);
  setPropertyHelpText(
      alpha_property_,
      "Amount of transparency to apply to the points.  Note that this is experimental and does not always look correct.");

  coloredpointcloud_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty>(
      "Colored Point Cloud Enabled", property_prefix_,
      boost::bind(&DepthImageDisplay::isColoredPointCloudEnabled, this),
      boost::bind(&DepthImageDisplay::setColoredPointCloudEnabled, this, _1), parent_category_, this);
  setPropertyHelpText(coloredpointcloud_enabled_property_, "Whether to display colored point cloud.");

  // RGB transport
  rgb_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>(
      "RGB image Topic", property_prefix_, boost::bind(&DepthImageDisplay::getRGBTopic, this),
      boost::bind(&DepthImageDisplay::setRGBTopic, this, _1), parent_category_, this);
  setPropertyHelpText(rgb_topic_property_, "sensor_msgs::Image topic to subscribe to.");
  ROSTopicStringPropertyPtr rgb_topic_prop = rgb_topic_property_.lock();
  rgb_topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Image>());

  rgb_transport_property_ = property_manager_->createProperty<EditEnumProperty>(
      "RGB image Transport Hint", property_prefix_, boost::bind(&DepthImageDisplay::getRGBTransport, this),
      boost::bind(&DepthImageDisplay::setRGBTransport, this, _1), parent_category_, this);
  EditEnumPropertyPtr ee2_prop = rgb_transport_property_.lock();
  ee2_prop->setOptionCallback(boost::bind(&DepthImageDisplay::onTransportEnumOptions, this, _1));

  setPointCloudVisible(pointcloud_visible_);

}

void DepthImageDisplay::reset()
{
  Display::reset();

  clear();
}

void DepthImageDisplay::callbackDepthRGB(const sensor_msgs::ImageConstPtr& depth_msg,
                                         const sensor_msgs::ImageConstPtr& rgb_msg,
                                         const sensor_msgs::CameraInfoConstPtr& info_msg

                                         )
{
  boost::mutex::scoped_lock lock(mutex_);

  processMessages(depth_msg, rgb_msg, info_msg);

}

void DepthImageDisplay::callbackDepth(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg

                                      )
{
  boost::mutex::scoped_lock lock(mutex_);

  processMessages(depth_msg, sensor_msgs::ImageConstPtr(), info_msg);

}

void DepthImageDisplay::processMessages(const sensor_msgs::ImageConstPtr& depth_msg,
                                        const sensor_msgs::ImageConstPtr& rgb_msg,
                                        const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(depth_msg->encoding);
  int numChannels = enc::numChannels(depth_msg->encoding);

  depthMapView_.updateImage(depth_msg);
  ++image_count_;

  if (pointcloud_visible_)
  {

    // Update camera model
    cameraModel_->fromCameraInfo(info_msg);

    // Get transformation
    Ogre::Matrix4 transform;
    Ogre::Vector3 pos;
    Ogre::Quaternion orient;

    if (!vis_manager_->getFrameManager()->getTransform(info_msg->header, pos, orient))
    {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << info_msg->header.frame_id << "] to frame ["
          << vis_manager_->getFrameManager()->getFixedFrame() << "]";
      setStatus(status_levels::Error, "Message", ss.str());
      return;
    }

    transform = Ogre::Matrix4(orient);
    transform.setTrans(pos);

    ///////////////////////
    // Convert depth image to point cloud
    ////////////////////////////////////////

    cloud_->clear();

    // Use correct principal point from calibration
    float center_x = cameraModel_->cx();
    float center_y = cameraModel_->cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = 1.0; /////////!!!!!!!!!!!!!111 DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / cameraModel_->fx();
    float constant_y = unit_scaling / cameraModel_->fy();

    int width = depth_msg->width;
    int height = depth_msg->height;

    unsigned char* colorImgPtr = 0;

    if (coloredpointcloud_visible_ && rgb_msg) {
    // Check for bad inputs
        if (depth_msg->header.frame_id != rgb_msg->header.frame_id)
        {
          std::stringstream errorMsg;
          errorMsg << "Depth image frame id ["<<depth_msg->header.frame_id.c_str()<<
                                 "] doesn't match RGB image frame id ["<< rgb_msg->header.frame_id.c_str()<<"]";
          setStatus(status_levels::Error, "Message", errorMsg.str() );
          return;
        }

        if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
        {
          std::stringstream errorMsg;
          errorMsg << "Depth resolution ("<< (int)depth_msg->width<<"x"<<(int)depth_msg->height<<") "
                                  "does not match RGB resolution ("<< (int)rgb_msg->width<<"x"<<(int)rgb_msg->height<<")";
          setStatus(status_levels::Error, "Message", errorMsg.str() );
          return;
        }

        // OpenCV-ros bridge
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(rgb_msg, "rgba8");

        }
        catch (cv_bridge::Exception& e)
        {
          setStatus(status_levels::Error, "Message", e.what() );
          return;
        }
        catch (cv::Exception& e)
        {
          setStatus(status_levels::Error, "Message", e.what() );
          return;
        }

        colorImgPtr = (unsigned char*) &cv_ptr->image.data[0];
    }

    new_points_.clear();
    new_points_.reserve(height * width);

    Ogre::Vector3 newPoint;
    Ogre::Vector3 transPoint;
    rviz::PointCloud::Point rvizPoint;
    float color_r,color_g,color_b;

    // process float depth map
    if ((bitDepth == 32) && (numChannels == 1))
    {

      const float* img_ptr = (float*)&depth_msg->data[0];
      for (int v = 0; v < height; ++v)
      {
        for (int u = 0; u < width; ++u)
        {
          if (colorImgPtr) {
            color_r = (float)(*colorImgPtr++)/255.0f;
            color_g = (float)(*colorImgPtr++)/255.0f;
            color_b = (float)(*colorImgPtr++)/255.0f;
            colorImgPtr++; // alpha padding
          } else {
            color_r = color_g = color_b = 1.0f;
          }

          float depth = *img_ptr++;

          // Missing points denoted by NaNs
          if ((!std::isfinite(depth)) || (depth > maxDepth_))
          {
            continue;
          }

          // Fill in XYZ
          newPoint.x = (u - center_x) * depth * constant_x;
          newPoint.y = (v - center_y) * depth * constant_y;
          newPoint.z = depth;

          transPoint = transform * newPoint;

          if (validateFloats(transPoint))
          {
            rvizPoint.x = transPoint.x;
            rvizPoint.y = transPoint.y;
            rvizPoint.z = transPoint.z;

            rvizPoint.setColor(color_r, color_g, color_b);

            new_points_.push_back(rvizPoint);
          }
        }
      }
    } // Raw depth map
    else if ((bitDepth == 16) && (numChannels == 1))
    {
      unsigned short* img_ptr = (unsigned short*)&depth_msg->data[0];
      for (int v = 0; v < height; ++v)
      {
        for (int u = 0; u < width; ++u)
        {

          if (colorImgPtr) {
            color_r = (float)(*colorImgPtr++)/255.0f;
            color_g = (float)(*colorImgPtr++)/255.0f;
            color_b = (float)(*colorImgPtr++)/255.0f;
            colorImgPtr++; // alpha padding
          } else {
            color_r = color_g = color_b = 1.0f;
          }

          float depth = (float)(*img_ptr++) / 1000.0f; // mm to meter conversion

          // Missing points denoted by zeros
          if ((depth==0.0f) || (depth > maxDepth_))
          {
            continue;
          }
          // Fill in XYZ
          newPoint.x = (u - center_x) * depth * constant_x;
          newPoint.y = (v - center_y) * depth * constant_y;
          newPoint.z = depth;

          transPoint = transform * newPoint;

          if (validateFloats(transPoint))
          {
            rvizPoint.x = transPoint.x;
            rvizPoint.y = transPoint.y;
            rvizPoint.z = transPoint.z;

            rvizPoint.setColor(color_r, color_g, color_b);

            new_points_.push_back(rvizPoint);
          }
        }
      }

    }
    else
    {
      std::stringstream errorMsg;
      errorMsg << "Single-channel 32bit-floating point or 16bit raw depth images are required (input format is: "
          << depth_msg->encoding << ")";
      setStatus(status_levels::Error, "Message", errorMsg.str());
      return;
    }

    newCloud_ = true;

  }


  setStatus(status_levels::Ok, "Message", "OK");

}

void DepthImageDisplay::fixedFrameChanged()
{
  Display::reset();
}

} // namespace depth_image_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( depth_image_plugin, DepthImage, depth_image_plugin::DepthImageDisplay, rviz::Display)

