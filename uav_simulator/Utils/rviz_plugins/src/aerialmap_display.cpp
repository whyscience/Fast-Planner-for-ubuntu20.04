/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <boost/bind.hpp>

#include <iomanip>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"

#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

#include "aerialmap_display.h"

namespace rviz_common {
using properties::BoolProperty;
using properties::FloatProperty;

AerialMapDisplay::AerialMapDisplay()
    : Display(),
      manual_object_(NULL)
    //! @bug cannot compile @gcc-5 or later, material_(0)
    ,
      loaded_(false),
      resolution_(0.0f),
      width_(0),
      height_(0),
      position_(Ogre::Vector3::ZERO),
      orientation_(Ogre::Quaternion::IDENTITY),
      new_map_(false) {
  topic_property_ = new RosTopicProperty(
      "Topic", "", QString::fromStdString(
          rclcpp::message_traits::datatype<nav_msgs::msg::OccupancyGrid>()),
      "nav_msgs::msg::OccupancyGrid topic to subscribe to.", this,
      SLOT(updateTopic()));

  alpha_property_ = new FloatProperty(
      "Alpha", 0.7, "Amount of transparency to apply to the map.", this,
      SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  draw_under_property_ =
      new Property("Draw Behind", false,
                   "Rendering option, controls whether or not the map is always"
                   " drawn behind everything else.",
                   this, SLOT(updateDrawUnder()));

  resolution_property_ = new FloatProperty(
      "Resolution", 0, "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  width_property_ = new IntProperty(
      "Width", 0, "Width of the map, in meters. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new IntProperty(
      "Height", 0, "Height of the map, in meters. (not editable)", this);
  height_property_->setReadOnly(true);

  position_property_ = new VectorProperty(
      "Position", Ogre::Vector3::ZERO,
      "Position of the bottom left corner of the map, in meters. (not editable)",
      this);
  position_property_->setReadOnly(true);

  orientation_property_ =
      new QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                             "Orientation of the map. (not editable)", this);
  orientation_property_->setReadOnly(true);
}

AerialMapDisplay::~AerialMapDisplay() {
  unsubscribe();
  clear();
}

void
AerialMapDisplay::onInitialize() {
  static int count = 0;
  std::stringstream ss;
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  ss << "AerialMapObjectMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);

  updateAlpha();
}

void
AerialMapDisplay::onEnable() {
  subscribe();
}

void
AerialMapDisplay::onDisable() {
  unsubscribe();
  clear();
}

void
AerialMapDisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }

  if (!topic_property_->getTopic().isEmpty()) {
    try {
      //todo eric
      //map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1, &AerialMapDisplay::incomingAerialMap, this);
      setStatus(properties::StatusProperty::Ok, "Topic", "OK");
    }
    catch (rclcpp::Exception &e) {
      setStatus(properties::StatusProperty::Error, "Topic",
                QString("Error subscribing: ") + e.what());
    }
  }
}

void
AerialMapDisplay::unsubscribe() {
  //map_sub_.shutdown();//todo eric
}

void
AerialMapDisplay::updateAlpha() {
  float alpha = alpha_property_->getFloat();

  Ogre::Pass *pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState *tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0) {
    tex_unit = pass->getTextureUnitState(0);
  } else {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                              Ogre::LBS_CURRENT, alpha);

  if (alpha < 0.9998) {
    material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->setDepthWriteEnabled(false);
  } else {
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(!draw_under_property_->getValue().toBool());
  }
}

void
AerialMapDisplay::updateDrawUnder() {
  bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= 0.9998) {
    material_->setDepthWriteEnabled(!draw_under);
  }

  if (manual_object_) {
    if (draw_under) {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
    } else {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
    }
  }
}

void
AerialMapDisplay::updateTopic() {
  unsubscribe();
  subscribe();
  clear();
}

void
AerialMapDisplay::clear() {
  setStatus(properties::StatusProperty::Warn, "Message", "No map received");

  if (!loaded_) {
    return;
  }

  scene_manager_->destroyManualObject(manual_object_);
  manual_object_ = NULL;

  std::string tex_name = texture_->getName();
  texture_.setNull();
  Ogre::TextureManager::getSingleton().unload(tex_name);

  loaded_ = false;
}
/*
bool validateFloats(const nav_msgs::msg::OccupancyGrid& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg->info.resolution );
  valid = valid && validateFloats( msg->info.origin );
  return valid;
}
*/
void
AerialMapDisplay::update(float wall_dt, float ros_dt) {
  {
    boost::mutex::scoped_lock lock(mutex_);

    current_map_ = updated_map_;
  }

  if (!current_map_ || !new_map_) {
    return;
  }

  if (current_map_->data.empty()) {
    return;
  }

  new_map_ = false;
  /*
    if( !validateFloats( *current_map_ ))
    {
      setStatus( properties::StatusProperty::Error, "Map", "Message contained invalid
    floating point values (nans or infs)" );
      return;
    }
  */
  if (current_map_->info.width * current_map_->info.height == 0) {
    std::stringstream ss;
    ss << "AerialMap is zero-sized (" << current_map_->info.width << "x"
       << current_map_->info.height << ")";
    setStatus(properties::StatusProperty::Error, "AerialMap",
              QString::fromStdString(ss.str()));
    return;
  }

  clear();

  setStatus(properties::StatusProperty::Ok, "Message", "AerialMap received");

  /*ROS_DEBUG*/printf("Received a %d X %d map @ %.3f m/pix\n", current_map_->info.width,
            current_map_->info.height, current_map_->info.resolution);

  float resolution = current_map_->info.resolution;

  int width = current_map_->info.width;
  int height = current_map_->info.height;

  Ogre::Vector3 position(current_map_->info.origin.position.x,
                         current_map_->info.origin.position.y,
                         current_map_->info.origin.position.z);
  Ogre::Quaternion orientation(current_map_->info.origin.orientation.w,
                               current_map_->info.origin.orientation.x,
                               current_map_->info.origin.orientation.y,
                               current_map_->info.origin.orientation.z);
  frame_ = current_map_->header.frame_id;
  if (frame_.empty()) {
    frame_ = "/map";
  }

  // Expand it to be RGB data
  unsigned int pixels_size = width * height * 3;
  unsigned char *pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  bool map_status_set = false;
  unsigned int num_pixels_to_copy = pixels_size;
  if (pixels_size != current_map_->data.size()) {
    std::stringstream ss;
    ss << "Data size doesn't match width*height: width = " << width
       << ", height = " << height
       << ", data size = " << current_map_->data.size();
    setStatus(properties::StatusProperty::Error, "AerialMap",
              QString::fromStdString(ss.str()));
    map_status_set = true;

    // Keep going, but don't read past the end of the data.
    if (current_map_->data.size() < pixels_size) {
      num_pixels_to_copy = current_map_->data.size();
    }
  }

  // TODO: a fragment shader could do this on the video card, and
  // would allow a non-grayscale color to mark the out-of-range
  // values.
  for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy;
       pixel_index++) {
    /*
        unsigned char val;
        int8_t data = current_map_->data[ pixel_index ];
        if(data > 0)
          val = 0;
        else if(data < 0)
          val = 255;
        else
          val = 127;
        pixels[ pixel_index ] = val;
    */
    pixels[pixel_index] = current_map_->data[pixel_index];
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "AerialMapTexture" << tex_count++;
  try {
    texture_ = Ogre::TextureManager::getSingleton().loadRawData(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        pixel_stream, width, height, Ogre::PF_R8G8B8, Ogre::TEX_TYPE_2D, 0);

    if (!map_status_set) {
      setStatus(properties::StatusProperty::Ok, "AerialMap", "AerialMap OK");
    }
  }
  catch (Ogre::RenderingAPIException &) {
    Ogre::Image image;
    pixel_stream->seek(0);
    float fwidth = width;
    float fheight = height;
    if (width > height) {
      float aspect = fheight / fwidth;
      fwidth = 2048;
      fheight = fwidth * aspect;
    } else {
      float aspect = fwidth / fheight;
      fheight = 2048;
      fwidth = fheight * aspect;
    }

    {
      std::stringstream ss;
      ss << "AerialMap is larger than your graphics card supports.  "
            "Downsampled from ["
         << width << "x" << height << "] to [" << fwidth << "x" << fheight
         << "]";
      setStatus(properties::StatusProperty::Ok, "AerialMap",
                QString::fromStdString(ss.str()));
    }

    /*RCLCPP_WARN(nh_->get_logger(),*/printf( "Failed to create full-size map texture, likely because your "
                                     "graphics card does not support textures of size > 2048.  "
                                     "Downsampling to [%d x %d]...",
                (int) fwidth, (int) fheight);
    // RCLCPP_INFO(nh_->get_logger(), "Stream size [%d], width [%f], height [%f], w * h [%f]",
    // pixel_stream->size(), width, height, width * height);
    image.loadRawData(pixel_stream, width, height, Ogre::PF_R8G8B8);
    image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
    ss << "Downsampled";
    texture_ = Ogre::TextureManager::getSingleton().loadImage(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
  }

  delete[] pixels;

  Ogre::Pass *pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState *tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0) {
    tex_unit = pass->getTextureUnitState(0);
  } else {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "AerialMapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject(ss2.str());
  scene_node_->attachObject(manual_object_);

  manual_object_->begin(material_->getName(),
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, resolution * height, 0.0f);
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(resolution * width, 0.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }
  }
  manual_object_->end();

  if (draw_under_property_->getValue().toBool()) {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  resolution_property_->setValue(resolution);
  width_property_->setValue(width);
  height_property_->setValue(height);
  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);

  transformAerialMap();

  loaded_ = true;

  context_->queueRender();
}

void
AerialMapDisplay::incomingAerialMap(
    nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  updated_map_ = msg;
  boost::mutex::scoped_lock lock(mutex_);
  new_map_ = true;
}

void
AerialMapDisplay::transformAerialMap() {
  if (!current_map_) {
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(
      frame_, rclcpp::Time(), current_map_->info.origin, position, orientation)) {
    /*ROS_DEBUG*/printf("Error transforming map '%s' from frame '%s' to frame '%s'",
              qPrintable(getName()), frame_.c_str(), qPrintable(fixed_frame_));

    setStatus(properties::StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame_) +
                  "] to [" + fixed_frame_ + "]");
  } else {
    setStatus(properties::StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void
AerialMapDisplay::fixedFrameChanged() {
  transformAerialMap();
}

void
AerialMapDisplay::reset() {
  Display::reset();

  clear();
  // Force resubscription so that the map will be re-sent
  updateTopic();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_common::AerialMapDisplay, rviz_common::Display)
