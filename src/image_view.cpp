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


#include <boost/algorithm/string/erase.hpp>
#include <boost/foreach.hpp>

#include <OGRE/OgreTextureManager.h>

#include <sensor_msgs/image_encodings.h>

#include <sstream>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>

#include "image_view.h"


namespace depth_image_plugin
{

ImageView::ImageView() :
  new_image_(false)
, width_(0)
, height_(0)
, minDepth_(0.0f)
, maxDepth_(10.0f)
{
  empty_image_.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  static uint32_t count = 0;
  std::stringstream ss;
  ss << "ImageView" << count++;
  texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, empty_image_, Ogre::TEX_TYPE_2D, 0);
}


ImageView::~ImageView()
{
  current_image_.reset();
  delete screen_rect_;
}

void ImageView::initialize(Ogre::SceneNode* image_scene_node) {

   {
     static int count = 0;
     std::stringstream ss;
     ss << "DepthImageDisplayObject" << count++;

     screen_rect_ = new Ogre::Rectangle2D(true);
     screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
     screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

     ss << "Material";
     material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
     material_->setSceneBlending( Ogre::SBT_REPLACE );
     material_->setDepthWriteEnabled(false);
     material_->setReceiveShadows(false);
     material_->setDepthCheckEnabled(false);

     material_->getTechnique(0)->setLightingEnabled(false);
     Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
     tu->setTextureName(getTexture()->getName());
     tu->setTextureFiltering( Ogre::TFO_NONE );

     material_->setCullingMode(Ogre::CULL_NONE);
     Ogre::AxisAlignedBox aabInf;
     aabInf.setInfinite();
     screen_rect_->setBoundingBox(aabInf);
     screen_rect_->setMaterial(material_->getName());
     image_scene_node->attachObject(screen_rect_);
   }
}


void ImageView::clear()
{
  boost::mutex::scoped_lock lock(mutex_);

  texture_->unload();
  texture_->loadImage(empty_image_);

  new_image_ = false;
  current_image_.reset();
}

bool ImageView::draw(float win_width, float win_height)
{
  sensor_msgs::Image::ConstPtr image;
  bool new_image = false;
  {
    boost::mutex::scoped_lock lock(mutex_);

    image = current_image_;
    new_image = new_image_;
  }

  if (!image || !new_image)
  {
    return false;
  }

  new_image_ = false;

  if (image->data.empty())
  {
    return false;
  }

  Ogre::PixelFormat format = Ogre::PF_R8G8B8;
  Ogre::Image ogre_image;
  std::vector<uint8_t> buffer;
  void* data_ptr = (void*)&image->data[0];
  uint32_t data_size = image->data.size();
  if (image->encoding == sensor_msgs::image_encodings::RGB8)
  {
    format = Ogre::PF_BYTE_RGB;
  }
  else if (image->encoding == sensor_msgs::image_encodings::RGBA8)
  {
    format = Ogre::PF_BYTE_RGBA;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC4 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_8SC4 ||
           image->encoding == sensor_msgs::image_encodings::BGRA8)
  {
    format = Ogre::PF_BYTE_BGRA;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_8SC3 ||
           image->encoding == sensor_msgs::image_encodings::BGR8)
  {
    format = Ogre::PF_BYTE_BGR;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
           image->encoding == sensor_msgs::image_encodings::MONO8)
  {
    format = Ogre::PF_BYTE_L;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
           image->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
           image->encoding == sensor_msgs::image_encodings::MONO16)
  {
    format = Ogre::PF_SHORT_L;

    // Rescale floating point image to 0<=x<=1 scale
    size_t i;
    unsigned short* img_ptr;
    uint32_t img_size = image->data.size() / sizeof(unsigned short);

    // Rescale floating-point image
    unsigned short minDepth = minDepth_;
    unsigned short maxDepth = maxDepth_*1000.0f; // mm to meter conversion
    unsigned short dynamic_range = std::max(0, maxDepth - minDepth);
    unsigned short dynamic_range_scale = std::numeric_limits<unsigned short>::max()/dynamic_range;

    if( dynamic_range )
    {
      img_ptr = (unsigned short*) &image->data[0];
      for( i = 0; i < img_size; ++i )
      {
        *img_ptr = std::min(maxDepth, std::max(minDepth, *img_ptr));

        *img_ptr -= minDepth;
        *img_ptr *= dynamic_range_scale;
        img_ptr++;
      }
    }
  }
  else if (image->encoding.find("bayer") == 0)
  {
    format = Ogre::PF_BYTE_L;
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    // Ogre encoding
    format = Ogre::PF_FLOAT32_R;

    // Rescale floating point image to 0<=x<=1 scale
    size_t i;
    float* img_ptr;
    uint32_t img_size = image->data.size() / sizeof(float);

    // Rescale floating-point image
    float dynamic_range = std::max(0.0f, maxDepth_ - minDepth_);

    if( dynamic_range > 0.0f )
    {
      img_ptr = (float*) &image->data[0];
      for( i = 0; i < img_size; ++i )
      {
        *img_ptr = std::min(maxDepth_, std::max(minDepth_, *img_ptr));

        *img_ptr -= minDepth_;
        *img_ptr /= dynamic_range;
        img_ptr++;
      }
    }
  }
  else
  {
    throw UnsupportedImageViewEncoding(image->encoding);
  }

  float img_width, img_height;
  img_width = width_ = image->width;
  img_height = height_ = image->height;

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(data_ptr, data_size));

  ogre_image.loadRawData(pixel_stream, width_, height_, 1, format, 1, 0);

  texture_->unload();
  texture_->loadImage(ogre_image);

  //make sure the aspect ratio of the image is preserved
  if ( img_width != 0 && img_height != 0 && win_width !=0 && win_height != 0 )
  {
    float img_aspect = img_width / img_height;
    float win_aspect = win_width / win_height;

    if ( img_aspect > win_aspect )
    {
      screen_rect_->setCorners(-1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect, false);
    }
    else
    {
      screen_rect_->setCorners(-1.0f * img_aspect/win_aspect, 1.0f, 1.0f * img_aspect/win_aspect, -1.0f, false);
    }
  }


  return true;
}

void ImageView::updateImage(const sensor_msgs::Image::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  current_image_ = msg;
  new_image_ = true;

}

} // end of namespace rviz
