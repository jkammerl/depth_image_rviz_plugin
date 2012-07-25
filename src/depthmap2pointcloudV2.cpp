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

#include "depthmap2pointcloudV2.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

// opencv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>

#include "image_transport/camera_common.h"

#include <sstream>

using namespace rviz;
namespace enc = sensor_msgs::image_encodings;

namespace depth_image_plugin
{

Depth2PointCloudV2::Depth2PointCloudV2()
  : rviz::ImageDisplayBase()
  , depthmap_it_(update_nh_)
  , depthmap_sub_()
  , rgb_it_ (update_nh_)
  , rgb_sub_()
  , cameraInfo_sub_()
  , queue_size_(1)
  , pointCloudsEnabled_ (false)
  , coloredPointCloudsEnabled_(false)
  , cameraModel_(new image_geometry::PinholeCameraModel)
{


  rgb_topic_property_ = new RosTopicProperty("RGB Image Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                         "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateTopic() ));

  rgb_transport_property_ = new EnumProperty("RGB Transport Hint", "raw", "Preferred method of sending images.", this,
                                         SLOT( updateTopic() ));


  connect(rgb_transport_property_, SIGNAL( requestOptions( EnumProperty* )), this,
          SLOT( fillTransportOptionList( EnumProperty* )));

  pointcloud_common_ = new PointCloudCommon(this);

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue( pointcloud_common_->getCallbackQueue() );

}

void Depth2PointCloudV2::onInitialize()
{

  pointcloud_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  pointcloud_scene_node_->setVisible( true );

  pointcloud_common_->initialize(context_, pointcloud_scene_node_);
}

Depth2PointCloudV2::~Depth2PointCloudV2()
{

  unsubscribe();

  if (cameraModel_)
    delete (cameraModel_);

  if (pointcloud_common_)
    delete pointcloud_common_;

}

void Depth2PointCloudV2::onEnable()
{
  pointcloud_scene_node_->setVisible( true );
  subscribe();
}

void Depth2PointCloudV2::onDisable()
{
  pointcloud_scene_node_->setVisible( false );
  unsubscribe();

  clear();
}

void Depth2PointCloudV2::subscribe()
{

  try

  {
    syncDepthRGB_.reset(new SynchronizerDepthRGB(SyncPolicyDepthRGB(queue_size_)));

    depthmap_tf_filter_.reset();

    depthmap_sub_.reset(new image_transport::SubscriberFilter());
    rgb_sub_.reset(new image_transport::SubscriberFilter());
    cameraInfo_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

    std::string depthmap_topic = topic_property_->getTopicStd();
    std::string rgb_topic = rgb_topic_property_->getTopicStd();

    if (!depthmap_topic.empty()) {
      depthmap_sub_->subscribe(depthmap_it_, depthmap_topic, queue_size_,  image_transport::TransportHints(transport_property_->getStdString()));

      depthmap_tf_filter_.reset(
          new tf::MessageFilter<sensor_msgs::Image>(*depthmap_sub_, *context_->getTFClient(), fixed_frame_.toStdString(), queue_size_, update_nh_));


      std::string info_topic = image_transport::getCameraInfoTopic(depthmap_topic);
      cameraInfo_sub_->subscribe(update_nh_, info_topic, queue_size_);
      cameraInfo_sub_->registerCallback(boost::bind(&Depth2PointCloudV2::caminfoCallback, this, _1));

      if (!rgb_topic.empty()) {
        rgb_sub_->subscribe(rgb_it_, rgb_topic, queue_size_,  image_transport::TransportHints(rgb_transport_property_->getStdString()));

        syncDepthRGB_->connectInput(*depthmap_tf_filter_, *rgb_sub_);
        syncDepthRGB_->registerCallback(boost::bind(&Depth2PointCloudV2::processMessage, this, _1, _2));
        coloredPointCloudsEnabled_ = true;
      } else
      {
        depthmap_tf_filter_->registerCallback(boost::bind(&Depth2PointCloudV2::processMessage, this, _1));
        coloredPointCloudsEnabled_ = false;
      }

    }
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }

}

void Depth2PointCloudV2::caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg )
{
  boost::mutex::scoped_lock lock(camInfo_mutex_);
  camInfo_ = msg;
 }

void Depth2PointCloudV2::unsubscribe()
{
  clear();

  boost::mutex::scoped_lock lock(mutex_);

  try

  {
    syncDepthRGB_.reset(new SynchronizerDepthRGB(SyncPolicyDepthRGB(queue_size_)));

    depthmap_tf_filter_.reset();

    depthmap_sub_.reset();
    rgb_sub_.reset();
    cameraInfo_sub_.reset();

    coloredPointCloudsEnabled_ = false;
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }

}

void Depth2PointCloudV2::updateQueueSize()
{
  ImageDisplayBase::updateTopic();
}


void Depth2PointCloudV2::clear()
{

  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_common_->reset();

  setStatus(StatusProperty::Warn, "Image", "No Image received");
}


void Depth2PointCloudV2::update(float wall_dt, float ros_dt)
{

  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_common_->update(wall_dt, ros_dt);

}


void Depth2PointCloudV2::reset()
{
  ImageDisplayBase::reset();

  clear();
}

void Depth2PointCloudV2::processMessage(const sensor_msgs::ImageConstPtr& depth_msg)
{

  processMessage(depth_msg, sensor_msgs::ImageConstPtr());

}

void Depth2PointCloudV2::processMessage(const sensor_msgs::ImageConstPtr& depth_msg,
                                        const sensor_msgs::ImageConstPtr& rgb_msg)
{
   ++messages_received_;
   setStatus(StatusProperty::Ok, "Image", QString::number(messages_received_) + " depth maps received");

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(depth_msg->encoding);
  int numChannels = enc::numChannels(depth_msg->encoding);

  // Update camera model
  {
    boost::mutex::scoped_lock lock(camInfo_mutex_);
    if (camInfo_)
      cameraModel_->fromCameraInfo(camInfo_);
  }

  // Convert depth map to point cloud

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

  if (rgb_msg)
  {

    if (depth_msg->header.frame_id != rgb_msg->header.frame_id)
    {
      std::stringstream errorMsg;
      errorMsg << "Depth image frame id [" << depth_msg->header.frame_id.c_str()
          << "] doesn't match RGB image frame id [" << rgb_msg->header.frame_id.c_str() << "]";
      setStatus(StatusProperty::Error, "Message", QString("Format Error: ") + errorMsg.str().c_str());
      return;
    }

    if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
    {
      std::stringstream errorMsg;
      errorMsg << "Depth resolution (" << (int)depth_msg->width << "x" << (int)depth_msg->height << ") "
          "does not match RGB resolution (" << (int)rgb_msg->width << "x" << (int)rgb_msg->height << ")";
      setStatus(StatusProperty::Error, "Message", QString("Format Error: ") + errorMsg.str().c_str());
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
      setStatus(StatusProperty::Error, "Message", QString("OpenCV-ROS bridge: ") + e.what());
      return;
    }
    catch (cv::Exception& e)
    {
      setStatus(StatusProperty::Error, "Message", QString("OpenCV: ") + e.what());
      return;
    }

    colorImgPtr = (unsigned char*)&cv_ptr->image.data[0];
  }

  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2);
  cloudMsg->header = depth_msg->header;

  if (rgb_msg)
  {
    cloudMsg->fields.resize(4);
    cloudMsg->fields[0].name = "x";
    cloudMsg->fields[1].name = "y";
    cloudMsg->fields[2].name = "z";
    cloudMsg->fields[3].name = "rgb";
  }
  else
  {
    cloudMsg->fields.resize(3);
    cloudMsg->fields[0].name = "x";
    cloudMsg->fields[1].name = "y";
    cloudMsg->fields[2].name = "z";
  }

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < cloudMsg->fields.size(); ++d, offset += 4)
  {
    cloudMsg->fields[d].offset = offset;
    cloudMsg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    cloudMsg->fields[d].count = 1;
  }

  cloudMsg->point_step = offset;

  cloudMsg->data.resize(height * width * cloudMsg->point_step);
  cloudMsg->is_bigendian = false;
  cloudMsg->is_dense = false;

  float* cloudDataPtr = reinterpret_cast<float*>(&cloudMsg->data[0]);

  size_t pointCount_ = 0;

  Ogre::Vector3 newPoint;
  Ogre::Vector3 transPoint;

  float color_r, color_g, color_b;

  // process float depth map
  if ((bitDepth == 32) && (numChannels == 1))
  {

    const float* img_ptr = (float*)&depth_msg->data[0];
    for (int v = 0; v < height; ++v)
    {
      for (int u = 0; u < width; ++u)
      {
        if (colorImgPtr)
        {
          color_r = (float)(*colorImgPtr++) / 255.0f;
          color_g = (float)(*colorImgPtr++) / 255.0f;
          color_b = (float)(*colorImgPtr++) / 255.0f;
          colorImgPtr++; // alpha padding
        }

        float depth = *img_ptr++;

        // Missing points denoted by NaNs
        if (!std::isfinite(depth))
        {
          continue;
        }

        // Fill in XYZ
        newPoint.x = (u - center_x) * depth * constant_x;
        newPoint.y = (v - center_y) * depth * constant_y;
        newPoint.z = depth;

        if (validateFloats(newPoint))
        {
          *cloudDataPtr = newPoint.x;
          ++cloudDataPtr;
          *cloudDataPtr = newPoint.y;
          ++cloudDataPtr;
          *cloudDataPtr = newPoint.z;
          ++cloudDataPtr;
        }

        ++pointCount_;

        if (colorImgPtr)
        {
          uint32_t rgb = ((uint32_t)color_r << 16 | (uint32_t)color_g << 8 | (uint32_t)color_b);
          *cloudDataPtr = *reinterpret_cast<float*>(&rgb);
          ++cloudDataPtr;
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

        if (colorImgPtr)
        {
          color_r = (float)(*colorImgPtr++) / 255.0f;
          color_g = (float)(*colorImgPtr++) / 255.0f;
          color_b = (float)(*colorImgPtr++) / 255.0f;
          colorImgPtr++; // alpha padding
        }

        float depth = (float)(*img_ptr++) / 1000.0f; // mm to meter conversion

        // Missing points denoted by zeros
        if (depth == 0.0f)
        {
          continue;
        }
        // Fill in XYZ
        newPoint.x = (u - center_x) * depth * constant_x;
        newPoint.y = (v - center_y) * depth * constant_y;
        newPoint.z = depth;

        if (validateFloats(newPoint))
        {
          *cloudDataPtr = newPoint.x;
          ++cloudDataPtr;
          *cloudDataPtr = newPoint.y;
          ++cloudDataPtr;
          *cloudDataPtr = newPoint.z;
          ++cloudDataPtr;
        }

        if (colorImgPtr)
        {
          uint32_t rgb = ((uint32_t)color_r << 16 | (uint32_t)color_g << 8 | (uint32_t)color_b);
          *cloudDataPtr = *reinterpret_cast<float*>(&rgb);
          ++cloudDataPtr;
        }
      }
    }

  }
  else
  {
    std::stringstream errorMsg;
    errorMsg << "Single-channel 32bit-floating point or 16bit raw depth images are required (input format is: "
        << depth_msg->encoding << ")";
    setStatus(StatusProperty::Error, "Message", QString("Format Error") + errorMsg.str().c_str());
    return;
  }

  cloudMsg->width = pointCount_;
  cloudMsg->height = 1;
  cloudMsg->data.resize(cloudMsg->height * cloudMsg->width * cloudMsg->point_step);
  cloudMsg->row_step = cloudMsg->point_step * cloudMsg->width;

  pointcloud_common_->addMessage(cloudMsg);

}

void Depth2PointCloudV2::fixedFrameChanged()
{
  Display::reset();
}

} // namespace depth_image_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( depth_image_plugin, depth2pointcloudV2, depth_image_plugin::Depth2PointCloudV2, rviz::Display)

