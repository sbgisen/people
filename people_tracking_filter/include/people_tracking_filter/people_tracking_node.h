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

#ifndef PEOPLE_TRACKING_FILTER_PEOPLE_TRACKING_NODE_H
#define PEOPLE_TRACKING_FILTER_PEOPLE_TRACKING_NODE_H

#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <string>
#include <list>
#include <boost/thread/mutex.hpp>

// ros stuff
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// people tracking stuff
#include <people_tracking_filter/tracker.h>
#include <people_tracking_filter/detector_particle.h>
#include <people_tracking_filter/gaussian_vector.h>

// messages
#include <sensor_msgs/msg/point_cloud.hpp>
#include <people_msgs/msg/position_measurement.hpp>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>

// log files
#include <fstream>


namespace estimation
{

class PeopleTrackingNode : public rclcpp::Node
{
public:
  /// constructor
  explicit PeopleTrackingNode();

  /// destructor
  virtual ~PeopleTrackingNode();

  /// callback for messages
  void callbackRcv(people_msgs::msg::PositionMeasurement::SharedPtr message);

  /// callback for dropped messages
  void callbackDrop(people_msgs::msg::PositionMeasurement::SharedPtr message);

  /// tracker loop
  void spin();

private:
  rclcpp::Publisher<people_msgs::msg::PositionMeasurement>::SharedPtr people_filter_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr people_filter_vis_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr people_tracker_vis_pub_;

  rclcpp::Subscription<people_msgs::msg::PositionMeasurement>::SharedPtr people_meas_sub_;

  /// message sequencer
  message_filters::TimeSequencer<people_msgs::msg::PositionMeasurement>*  message_sequencer_;

  /// trackers
  std::list<Tracker*> trackers_;

  // tf listener
  std::shared_ptr<tf2_ros::TransformListener> robot_state_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;

  unsigned int tracker_counter_;
  double freq_, start_distance_min_, reliability_threshold_;
  BFL::StatePosVel sys_sigma_;
  std::string fixed_frame_;
  boost::mutex filter_mutex_;

  sensor_msgs::msg::PointCloud  meas_cloud_;
  unsigned int meas_visualize_counter_;

  // Track only one person who the robot will follow.
  bool follow_one_person_;
};  // class
}  // namespace estimation
#endif  // PEOPLE_TRACKING_FILTER_PEOPLE_TRACKING_NODE_H
