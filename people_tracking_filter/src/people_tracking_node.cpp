/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <people_tracking_filter/people_tracking_node.h>
#include <people_tracking_filter/rgb.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/tracker_particle.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <chrono>
#include <list>
#include <people_msgs/msg/position_measurement.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

static const double sequencer_delay = 0.8;  // TODO(noidea): this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker = 1000;
static const double tracker_init_dist = 4.0;

namespace estimation
{
// constructor
PeopleTrackingNode::PeopleTrackingNode() : rclcpp::Node("people_tracker"), tracker_counter_(0)
{
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  robot_state_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

  // initialize
  meas_cloud_.points = std::vector<geometry_msgs::msg::Point32>(1);
  meas_cloud_.points[0].x = 0;
  meas_cloud_.points[0].y = 0;
  meas_cloud_.points[0].z = 0;

  // // get parameters
  this->declare_parameter("fixed_frame", "default");
  this->declare_parameter("freq", 1.0);
  this->declare_parameter("start_distance_min", 0.0);
  this->declare_parameter("reliability_threshold", 1.0);
  this->declare_parameter("sys_sigma_pos_x", 0.0);
  this->declare_parameter("sys_sigma_pos_y", 0.0);
  this->declare_parameter("sys_sigma_pos_z", 0.0);
  this->declare_parameter("sys_sigma_vel_x", 0.0);
  this->declare_parameter("sys_sigma_vel_y", 0.0);
  this->declare_parameter("sys_sigma_vel_z", 0.0);
  this->declare_parameter("follow_one_person", false);
  this->get_parameter("fixed_frame", fixed_frame_);
  this->get_parameter("freq", freq_);
  this->get_parameter("start_distance_min", start_distance_min_);
  this->get_parameter("reliability_threshold", reliability_threshold_);
  this->get_parameter("sys_sigma_pos_x", sys_sigma_.pos_[0]);
  this->get_parameter("sys_sigma_pos_y", sys_sigma_.pos_[1]);
  this->get_parameter("sys_sigma_pos_z", sys_sigma_.pos_[2]);
  this->get_parameter("sys_sigma_vel_x", sys_sigma_.vel_[0]);
  this->get_parameter("sys_sigma_vel_y", sys_sigma_.vel_[1]);
  this->get_parameter("sys_sigma_vel_z", sys_sigma_.vel_[2]);
  this->get_parameter("follow_one_person", follow_one_person_);

  // advertise filter output
  this->create_publisher<people_msgs::msg::PositionMeasurement>("people_tracker_filter", 10);

  // advertise visualization
  this->create_publisher<sensor_msgs::msg::PointCloud>("people_tracker_filter_visualization", 10);
  this->create_publisher<sensor_msgs::msg::PointCloud>(
    "people_tracker_measurements_visualization", 10);

  // // register message sequencer
  people_meas_sub_ = this->create_subscription<people_msgs::msg::PositionMeasurement>(
    "people_tracker_measurements", 10,
    [this](const people_msgs::msg::PositionMeasurement::SharedPtr msg) { callbackRcv(msg); });
}

// destructor
PeopleTrackingNode::~PeopleTrackingNode()
{
  // delete sequencer
  delete message_sequencer_;

  // delete all trackers
  for (std::list<Tracker *>::iterator it = trackers_.begin(); it != trackers_.end(); it++)
    delete *it;
};

// callback for messages
void PeopleTrackingNode::callbackRcv(const people_msgs::msg::PositionMeasurement::SharedPtr message)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Tracking node got a people position measurement (%f,%f,%f)",
    message->pos.x, message->pos.y, message->pos.z);
  // get measurement in fixed frame
  // tf2::Stamped<tf2::Vector3> meas_rel, meas;
  // meas_rel.setData(
  //   tf2::Vector3(message->pos.x, message->pos.y, message->pos.z));
  // meas_rel.stamp_ = tf2::TimePoint(
  //   std::chrono::seconds(message->header.stamp.sec) +
  //   std::chrono::nanoseconds(message->header.stamp.nanosec));
  geometry_msgs::msg::PointStamped meas_rel, meas;
  meas_rel.point = message->pos;
  meas_rel.header = message->header;
  buffer_->transform(meas_rel, meas, fixed_frame_);
  // robot_state_.transformPoint(fixed_frame_, meas_rel, meas);

  // get measurement covariance
  BFL::SymmetricMatrix cov(3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++) cov(i + 1, j + 1) = message->covariance[3 * i + j];

  // ----- LOCKED ------
  boost::mutex::scoped_lock lock(filter_mutex_);

  // update tracker if matching tracker found
  for (std::list<Tracker *>::iterator it = trackers_.begin(); it != trackers_.end(); it++)
    if ((*it)->getName() == message->object_id) {
      (*it)->updatePrediction(message->header.stamp.sec + message->header.stamp.nanosec * 1e-9);
      (*it)->updateCorrection(tf2::Vector3(meas.point.x, meas.point.y, meas.point.z), cov);
    }
  // check if reliable message with no name should be a new tracker
  if (message->object_id == "" && message->reliability > reliability_threshold_) {
    double closest_tracker_dist = start_distance_min_;
    BFL::StatePosVel est;
    for (std::list<Tracker *>::iterator it = trackers_.begin(); it != trackers_.end(); it++) {
      (*it)->getEstimate(est);
      double dst = sqrt(pow(est.pos_[0] - meas.point.x, 2) + pow(est.pos_[1] - meas.point.y, 2));
      if (dst < closest_tracker_dist) closest_tracker_dist = dst;
    }
    // initialize a new tracker
    if (follow_one_person_) std::cout << "Following one person" << std::endl;
    if (
      message->initialization == 1 &&
      ((!follow_one_person_ && (closest_tracker_dist >= start_distance_min_)) ||
       (follow_one_person_ && trackers_.empty()))) {
      // if (closest_tracker_dist >= start_distance_min_ || message->initialization == 1){
      // if (message->initialization == 1 && trackers_.empty()){
      RCLCPP_INFO(this->get_logger(), "Passed crazy conditional.");
      tf2::Vector3 pt(meas.point.x, meas.point.y, meas.point.z);
      // tf2::Stamped<tf2::Vector3> loc(
      //   pt, tf2::TimePoint(std::chrono::seconds(message->header.stamp.sec) + std::chrono::nanoseconds(message->header.stamp.nanosec)),
      //   message->header.frame_id);
      geometry_msgs::msg::PointStamped loc;
      loc.point = meas.point;
      loc.header = message->header;
      buffer_->transform(loc, loc, "base_link");
      // robot_state_.transformPoint("base_link", loc, loc);
      float cur_dist;
      if ((cur_dist = pow(loc.point.x, 2.0) + pow(loc.point.y, 2.0)) < tracker_init_dist) {
        std::cout << "starting new tracker" << std::endl;
        std::stringstream tracker_name;
        BFL::StatePosVel prior_sigma(
          tf2::Vector3(sqrt(cov(1, 1)), sqrt(cov(2, 2)), sqrt(cov(3, 3))),
          tf2::Vector3(0.0000001, 0.0000001, 0.0000001));
        tracker_name << "person " << tracker_counter_++;
        Tracker * new_tracker = new TrackerKalman(tracker_name.str(), sys_sigma_);
        // Tracker* new_tracker = new TrackerParticle(tracker_name.str(), num_particles_tracker, sys_sigma_);
        new_tracker->initialize(
          tf2::Vector3(meas.point.x, meas.point.y, meas.point.z), prior_sigma,
          message->header.stamp.sec + message->header.stamp.nanosec * 1e-9);
        trackers_.push_back(new_tracker);
        RCLCPP_INFO(this->get_logger(), "Initialized new tracker %s", tracker_name.str().c_str());
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Found a person, but he/she is not close enough to start following. "
          "Person is %f away, and must be less than %f away.",
          cur_dist, tracker_init_dist);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed crazy conditional.");
    }
  }
  lock.unlock();
  // ------ LOCKED ------

  // visualize measurement
  meas_cloud_.points[0].x = meas.point.x;
  meas_cloud_.points[0].y = meas.point.y;
  meas_cloud_.points[0].z = meas.point.z;
  meas_cloud_.header.frame_id = meas.header.frame_id;
  people_tracker_vis_pub_->publish(meas_cloud_);
}

// callback for dropped messages
void PeopleTrackingNode::callbackDrop(
  const people_msgs::msg::PositionMeasurement::SharedPtr message)
{
  RCLCPP_INFO(
    this->get_logger(), "DROPPED PACKAGE for %s from %s with delay %f !!!!!!!!!!!",
    message->object_id.c_str(), message->name.c_str(),
    (this->get_clock()->now() - message->header.stamp).seconds());
}

// filter loop
void PeopleTrackingNode::spin()
{
  RCLCPP_INFO(this->get_logger(), "People tracking manager started.");

  while (rclcpp::ok()) {
    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);

    // visualization variables
    std::vector<geometry_msgs::msg::Point32> filter_visualize(trackers_.size());
    std::vector<float> weights(trackers_.size());
    sensor_msgs::msg::ChannelFloat32 channel;

    // loop over trackers
    unsigned int i = 0;
    auto itr = trackers_.begin();
    while (itr != trackers_.end()) {
      // update prediction up to delayed time
      (*itr)->updatePrediction(this->get_clock()->now().seconds() - sequencer_delay);

      // publish filter result
      people_msgs::msg::PositionMeasurement est_pos;
      (*itr)->getEstimate(est_pos);
      est_pos.header.frame_id = fixed_frame_;

      RCLCPP_DEBUG(this->get_logger(), "Publishing people tracker filter.");
      people_filter_pub_->publish(est_pos);

      // visualize filter result
      filter_visualize[i].x = est_pos.pos.x;
      filter_visualize[i].y = est_pos.pos.y;
      filter_visualize[i].z = est_pos.pos.z;

      // calculate weight
      {
        int quality = static_cast<int>(trunc((*itr)->getQuality() * 999.0));
        int index = std::min(998, 999 - std::max(1, quality));
        weights[i] = *(float *)&(rgb[index]);  // NOLINT(readability/casting)
      }

      // remove trackers that have zero quality
      RCLCPP_INFO(
        this->get_logger(), "Quality of tracker %s = %f", (*itr)->getName().c_str(),
        (*itr)->getQuality());
      if ((*itr)->getQuality() <= 0) {
        RCLCPP_INFO(this->get_logger(), "Removing tracker %s", (*itr)->getName().c_str());
        delete *itr;
        trackers_.erase(itr++);
      } else
        itr++;
      i++;
    }
    lock.unlock();
    // ------ LOCKED ------

    // visualize all trackers
    channel.name = "rgb";
    channel.values = weights;
    sensor_msgs::msg::PointCloud people_cloud;
    people_cloud.channels.push_back(channel);
    people_cloud.header.frame_id = fixed_frame_;
    people_cloud.points = filter_visualize;
    people_filter_vis_pub_->publish(people_cloud);

    // sleep
    usleep(1e6 / freq_);

    rclcpp::spin_some(this->get_node_base_interface());
  }
};
}  // namespace estimation

// ----------
// -- MAIN --
// ----------
int main(int argc, char ** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  // create tracker node
  auto node = std::make_shared<estimation::PeopleTrackingNode>();

  // wait for filter to finish
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
