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
#include <bfl/pdf/pdf.h>
#include <leg_detector/calc_leg_features.h>
#include <leg_detector/laser_processor.h>
#include <message_filters/subscriber.h>
#include <opencv2/core/core_c.h>
#include <people_tracking_filter/rgb.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/tracker_kalman.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <memory>
#include <opencv2/ml.hpp>
#include <people_msgs/msg/position_measurement.hpp>
#include <people_msgs/msg/position_measurement_array.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <set>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "people_msgs/msg/position_measurement_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

static double no_observation_timeout_s = 0.5;
static double max_second_leg_age_s     = 2.0;
static double max_track_jump_m         = 1.0;
static double max_meas_jump_m          = 0.75;  // 1.0
static double leg_pair_separation_m    = 1.0;
static std::string fixed_frame         = "odom_combined";

static double kal_p = 4, kal_q = .002, kal_r = 10;
static bool use_filter = true;


class SavedFeature
{
public:
  static int nextid;
  tf2_ros::Buffer& tf_buffer_;

  BFL::StatePosVel sys_sigma_;
  estimation::TrackerKalman filter_;

  std::string id_;
  std::string object_id;
  rclcpp::Time time_;
  rclcpp::Time meas_time_;

  double reliability, p;

  geometry_msgs::msg::PointStamped position_;
  SavedFeature* other;
  float dist_to_person_;

  // one leg tracker
  SavedFeature(geometry_msgs::msg::PointStamped loc, tf2_ros::Buffer & buffer)
  : tf_buffer_(buffer),
    sys_sigma_(tf2::Vector3(0.05, 0.05, 0.05), tf2::Vector3(1.0, 1.0, 1.0)),
    filter_("tracker_name", sys_sigma_),
    reliability(-1.),
    p(4)
  {
    char id[100];
    snprintf(id, sizeof(id), "legtrack%d", nextid++);
    id_ = std::string(id);

    object_id = "";
    time_ = loc.header.stamp;
    meas_time_ = loc.header.stamp;
    other = NULL;

    try
    {
      loc.header.stamp = rclcpp::Time();
      tf_buffer_.transform(loc, loc, fixed_frame);
      // tfl_.transformPoint(fixed_frame, loc, loc);
    }
    catch (...)
    {
      // RCLCPP_WARN("TF exception spot 6.");
    }
    geometry_msgs::msg::TransformStamped pose;
    pose.header = loc.header;
    pose.transform.rotation.w = 1.0;
    pose.child_frame_id = id_;
    tf_buffer_.setTransform(pose, id_);
    // tf2::Stamped<tf2::Transform> pose(tf2::Pose(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, loc.frame_id_, id_);
    // tfl_.setTransform(pose);

    BFL::StatePosVel prior_sigma(tf2::Vector3(0.1, 0.1, 0.1), tf2::Vector3(0.0000001, 0.0000001, 0.0000001));
    BFL::StatePosVel mu(tf2::Vector3(loc.point.x, loc.point.y, loc.point.z), tf2::Vector3(0, 0 ,0));
    filter_.initialize(mu, prior_sigma, time_.seconds());

    BFL::StatePosVel est;
    filter_.getEstimate(est);

    updatePosition();
  }

  void propagate(rclcpp::Time time)
  {
    time_ = time;

    filter_.updatePrediction(time.seconds());

    updatePosition();
  }

  void update(geometry_msgs::msg::PointStamped loc, double probability)
  {
    if (rclcpp::Time(loc.header.stamp).seconds() <= meas_time_.seconds()) {
      loc.header.stamp = meas_time_ + rclcpp::Duration::from_seconds(0.0001);
    }
    geometry_msgs::msg::TransformStamped pose;
    pose.header = loc.header;
    pose.transform.rotation.w = 1.0;
    pose.child_frame_id = id_;
    tf_buffer_.setTransform(pose, id_);
    // tfl_.setTransform(pose);

    meas_time_ = loc.header.stamp;
    time_ = meas_time_;

    MatrixWrapper::SymmetricMatrix cov(3);
    cov = 0.0;
    cov(1, 1) = 0.0025;
    cov(2, 2) = 0.0025;
    cov(3, 3) = 0.0025;

    filter_.updateCorrection(tf2::Vector3(loc.point.x, loc.point.y, loc.point.z), cov);

    updatePosition();

    if (reliability < 0 || !use_filter)
    {
      reliability = probability;
      p = kal_p;
    }
    else
    {
      p += kal_q;
      double k = p / (p + kal_r);
      reliability += k * (probability - reliability);
      p *= (1 - k);
    }
  }

  double getLifetime()
  {
    return filter_.getLifetime();
  }

  double getReliability()
  {
    return reliability;
  }

private:
  void updatePosition()
  {
    BFL::StatePosVel est;
    filter_.getEstimate(est);

    position_.point.x = est.pos_[0];
    position_.point.y = est.pos_[1];
    position_.point.z = est.pos_[2];
    position_.header.stamp = time_;
    position_.header.frame_id = fixed_frame;
    double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));
    // reliability = fmax(reliability, nreliability);
  }
};

int SavedFeature::nextid = 0;




class MatchedFeature
{
public:
  laser_processor::SampleSet* candidate_;
  SavedFeature* closest_;
  float distance_;
  double probability_;

  MatchedFeature(laser_processor::SampleSet* candidate, SavedFeature* closest, float distance, double probability)
    : candidate_(candidate)
    , closest_(closest)
    , distance_(distance)
    , probability_(probability)
  {}

  inline bool operator< (const MatchedFeature& b) const
  {
    return (distance_ <  b.distance_);
  }
};

int g_argc;
char** g_argv;




// actual legdetector node
class LegDetector:public rclcpp::Node
{
public:

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tfl_;

  laser_processor::ScanMask mask_;

  int mask_count_;

  // cv::ml::RTrees forest;
  cv::Ptr<cv::ml::RTrees> forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  std::list<SavedFeature*> saved_features_;
  std::mutex saved_mutex_;

  int feature_id_;

  bool use_seeds_;
  bool publish_legs_, publish_people_, publish_leg_markers_, publish_people_markers_;
  int next_p_id_;
  double leg_reliability_limit_;
  int min_points_per_group;

  std::shared_ptr<rclcpp::Publisher<people_msgs::msg::PositionMeasurementArray>> people_measurements_pub_;
  std::shared_ptr<rclcpp::Publisher<people_msgs::msg::PositionMeasurementArray>> leg_measurements_pub_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> markers_pub_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  rclcpp::Subscription<people_msgs::msg::PositionMeasurement>::SharedPtr people_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  // message_filters::Subscriber<people_msgs::msg::PositionMeasurement> people_sub_;
  // message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
  // tf2_ros::MessageFilter<people_msgs::msg::PositionMeasurement> people_notifier_;
  // tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> laser_notifier_;

  explicit LegDetector(int argc, char * argv[])
  : rclcpp::Node("leg_detector"),
    mask_count_(0),
    feat_count_(0),
    next_p_id_(0),
    tf_buffer_(this->get_clock()),
    tfl_(tf_buffer_)
  {
    if (argc > 1) {
      forest = cv::ml::RTrees::create();
      cv::String feature_file = cv::String(argv[1]);
      forest = cv::ml::StatModel::load<cv::ml::RTrees>(feature_file);
      feat_count_ = forest->getVarCount();
      RCLCPP_INFO(this->get_logger(), "Loaded forest with %d features: %s\n", feat_count_, argv[1]);
    } else {
      RCLCPP_INFO(
        this->get_logger(), "Please provide a trained random forests classifier as an input.\n");
      rclcpp::shutdown();
    }

    this->declare_parameter("use_seeds", !true);
    this->declare_parameter("connected_thresh", 0.06);
    this->declare_parameter("min_points_per_group", 5);
    this->declare_parameter("leg_reliability_limit", 0.7);
    this->declare_parameter("publish_legs", true);
    this->declare_parameter("publish_people", true);
    this->declare_parameter("publish_leg_markers", true);
    this->declare_parameter("publish_people_markers", true);
    this->declare_parameter("no_observation_timeout", 0.5);
    this->declare_parameter("max_second_leg_age", 2.0);
    this->declare_parameter("max_track_jump", 1.0);
    this->declare_parameter("max_meas_jump", 0.75);
    this->declare_parameter("leg_pair_separation", 1.0);
    this->declare_parameter("fixed_frame", "odom_combined");
    this->declare_parameter("kalman_p", 4.0);
    this->declare_parameter("kalman_q", 0.002);
    this->declare_parameter("kalman_r", 10.0);
    this->declare_parameter("kalman_on", true);
    this->get_parameter("use_seeds", use_seeds_);
    this->get_parameter("connected_thresh", connected_thresh_);
    this->get_parameter("min_points_per_group", min_points_per_group);
    this->get_parameter("leg_reliability_limit", leg_reliability_limit_);
    this->get_parameter("publish_legs", publish_legs_);
    this->get_parameter("publish_people", publish_people_);
    this->get_parameter("publish_leg_markers", publish_leg_markers_);
    this->get_parameter("publish_people_markers", publish_people_markers_);
    this->get_parameter("no_observation_timeout", no_observation_timeout_s);
    this->get_parameter("max_second_leg_age", max_second_leg_age_s);
    this->get_parameter("max_track_jump", max_track_jump_m);
    this->get_parameter("max_meas_jump", max_meas_jump_m);
    this->get_parameter("leg_pair_separation", leg_pair_separation_m);
    this->get_parameter("fixed_frame", fixed_frame);
    this->get_parameter("kalman_p", kal_p);
    this->get_parameter("kalman_q", kal_q);
    this->get_parameter("kalman_r", kal_r);
    this->get_parameter("kalman_on", use_filter);

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    cb_handle_ = param_subscriber_->add_parameter_callback("connected_thresh", [this](const rclcpp::Parameter & p) {
      connected_thresh_ = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("min_points_per_group", [this](const rclcpp::Parameter & p) {
      min_points_per_group = p.as_int();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("leg_reliability_limit", [this](const rclcpp::Parameter & p) {
      leg_reliability_limit_ = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("publish_legs", [this](const rclcpp::Parameter & p) {
      publish_legs_ = p.as_bool();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("publish_people", [this](const rclcpp::Parameter & p) {
      publish_people_ = p.as_bool();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("publish_leg_markers", [this](const rclcpp::Parameter & p) {
      publish_leg_markers_ = p.as_bool();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("publish_people_markers", [this](const rclcpp::Parameter & p) {
      publish_people_markers_ = p.as_bool();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("no_observation_timeout", [this](const rclcpp::Parameter & p) {
      no_observation_timeout_s = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("max_second_leg_age", [this](const rclcpp::Parameter & p) {
      max_second_leg_age_s = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("max_track_jump", [this](const rclcpp::Parameter & p) {
      max_track_jump_m = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("max_meas_jump", [this](const rclcpp::Parameter & p) {
      max_meas_jump_m = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("leg_pair_separation", [this](const rclcpp::Parameter & p) {
      leg_pair_separation_m = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("fixed_frame", [this](const rclcpp::Parameter & p) {
      fixed_frame = p.as_string().c_str();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("kalman_p", [this](const rclcpp::Parameter & p) {
      kal_p = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("kalman_q", [this](const rclcpp::Parameter & p) {
      kal_q = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("kalman_r", [this](const rclcpp::Parameter & p) {
      kal_r = p.as_double();
    });
    cb_handle_ = param_subscriber_->add_parameter_callback("kalman_on", [this](const rclcpp::Parameter & p) {
      use_filter = p.as_bool();
    });

    // advertise topics
    leg_measurements_pub_ = this->create_publisher<people_msgs::msg::PositionMeasurementArray>("leg_tracker_measurements", 10);
    people_measurements_pub_ = this->create_publisher<people_msgs::msg::PositionMeasurementArray>("people_tracker_measurements", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);

    if (use_seeds_)
    {
      people_sub_ = this->create_subscription<people_msgs::msg::PositionMeasurement>("people", 10, std::bind(&LegDetector::peopleCallback, this, std::placeholders::_1));
      // people_notifier_.registerCallback(std::bind(&LegDetector::peopleCallback, this, std::placeholders::_1));
      // people_notifier_.setTolerance(rclcpp::Duration(0, 1e7));
    }
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LegDetector::laserCallback, this, std::placeholders::_1));
    // laser_notifier_.registerCallback(
    //     std::bind(&LegDetector::laserCallback, this, std::placeholders::_1));
    // laser_notifier_.setTolerance(rclcpp::Duration(0, 1e7));

    feature_id_ = 0;
  }


  ~LegDetector()
  {
  }

  // void configure(leg_detector::LegDetectorConfig &config, uint32_t level)
  // {
  //   connected_thresh_       = config.connection_threshold;
  //   min_points_per_group    = config.min_points_per_group;
  //   leg_reliability_limit_  = config.leg_reliability_limit;
  //   publish_legs_           = config.publish_legs;
  //   publish_people_         = config.publish_people;
  //   publish_leg_markers_    = config.publish_leg_markers;
  //   publish_people_markers_ = config.publish_people_markers;

  //   no_observation_timeout_s = config.no_observation_timeout;
  //   max_second_leg_age_s     = config.max_second_leg_age;
  //   max_track_jump_m         = config.max_track_jump;
  //   max_meas_jump_m          = config.max_meas_jump;
  //   leg_pair_separation_m    = config.leg_pair_separation;
  //   if (std::string(fixed_frame).compare(config.fixed_frame) != 0)
  //   {
  //     fixed_frame              = config.fixed_frame.c_str();
  //     laser_notifier_.setTargetFrame(fixed_frame);
  //     people_notifier_.setTargetFrame(fixed_frame);
  //   }

  //   kal_p                    = config.kalman_p;
  //   kal_q                    = config.kalman_q;
  //   kal_r                    = config.kalman_r;
  //   use_filter               = config.kalman_on == 1;
  // }

  double distance(std::list<SavedFeature*>::iterator it1,  std::list<SavedFeature*>::iterator it2)
  {
    geometry_msgs::msg::PointStamped one = (*it1)->position_, two = (*it2)->position_;
    double dx = one.point.x - two.point.x, dy = one.point.y - two.point.y, dz = one.point.z - two.point.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }

  // Find the tracker that is closest to this person message
  // If a tracker was already assigned to a person,
  // keep this assignment when the distance between them is not too large.
  void peopleCallback(const people_msgs::msg::PositionMeasurement::SharedPtr people_meas)
  {
    // If there are no legs, return.
    if (saved_features_.empty())
      return;

    geometry_msgs::msg::PointStamped person_loc;
    person_loc.point = people_meas->pos;
    person_loc.point.z = 0.0;  // Ignore the height of the person measurement.
    person_loc.header = people_meas->header;

    // Holder for all transformed pts.
    geometry_msgs::msg::PointStamped dest_loc;
    dest_loc.header = people_meas->header;
    dest_loc.point = people_meas->pos;

    std::scoped_lock lock(saved_mutex_);

    std::list<SavedFeature*>::iterator closest = saved_features_.end();
    std::list<SavedFeature*>::iterator closest1 = saved_features_.end();
    std::list<SavedFeature*>::iterator closest2 = saved_features_.end();
    float closest_dist = max_meas_jump_m;
    float closest_pair_dist = 2 * max_meas_jump_m;

    std::list<SavedFeature*>::iterator begin = saved_features_.begin();
    std::list<SavedFeature*>::iterator end = saved_features_.end();
    std::list<SavedFeature*>::iterator it1, it2;

    // If there's a pair of legs with the right label and within the max dist, return
    // If there's one leg with the right label and within the max dist,
    //   find a partner for it from the unlabeled legs whose tracks are reasonably new.
    //   If no partners exist, label just the one leg.
    // If there are no legs with the right label and within the max dist,
    //   find a pair of unlabeled legs and assign them the label.
    // If all of the above cases fail,
    //   find a new unlabeled leg and assign the label.

    // For each tracker, get the distance to this person.
    for (it1 = begin; it1 != end; ++it1)
    {
      try
      {
        std::chrono::nanoseconds time(rclcpp::Time(people_meas->header.stamp).nanoseconds());
        tf_buffer_.transform(person_loc, dest_loc, (*it1)->id_,
                                            tf2::TimePoint(time), fixed_frame);
        // tfl_.transformPoint((*it1)->id_, people_meas->header.stamp,
        //                     person_loc, fixed_frame, dest_loc);
        RCLCPP_INFO(this->get_logger(), "Succesful leg transformation at spot 7");
      }
      catch (...)
      {
        RCLCPP_WARN(this->get_logger(), "TF exception spot 7.");
      }
      (*it1)->dist_to_person_ = std::hypot(dest_loc.point.x, dest_loc.point.y);
    }

    // Try to find one or two trackers with the same label and within the max distance of the person.
    std::cout << "Looking for two legs" << std::endl;
    it2 = end;
    for (it1 = begin; it1 != end; ++it1)
    {
      // If this leg belongs to the person...
      if ((*it1)->object_id == people_meas->object_id)
      {
        // and their distance is close enough...
        if ((*it1)->dist_to_person_ < max_meas_jump_m)
        {
          // if this is the first leg we've found, assign it to it2. Otherwise, leave it assigned to it1 and break.
          if (it2 == end)
            it2 = it1;
          else
            break;
        }
        // Otherwise, remove the tracker's label, it doesn't belong to this person.
        else
        {
          // the two trackers moved apart. This should not happen.
          (*it1)->object_id = "";
        }
      }
    }
    // If we found two legs with the right label and within the max distance, all is good, return.
    if (it1 != end && it2 != end)
    {
      std::cout << "Found matching pair. The second distance was " << (*it1)->dist_to_person_ << std::endl;
      return;
    }



    // If we only found one close leg with the right label, let's try to find a second leg that
    //   * doesn't yet have a label  (=valid precondition),
    //   * is within the max distance,
    //   * is less than max_second_leg_age_s old.
    std::cout << "Looking for one leg plus one new leg" << std::endl;
    float dist_between_legs, closest_dist_between_legs;
    if (it2 != end)
    {
      closest_dist = max_meas_jump_m;
      closest = saved_features_.end();

      for (it1 = begin; it1 != end; ++it1)
      {
        // Skip this leg track if:
        // - you're already using it.
        // - it already has an id.
        // - it's too old. Old unassigned trackers are unlikely to be the second leg in a pair.
        // - it's too far away from the person.
        if ((it1 == it2) || ((*it1)->object_id != "") || ((*it1)->getLifetime() > max_second_leg_age_s) ||
            ((*it1)->dist_to_person_ >= closest_dist))
          continue;

        // Get the distance between the two legs
        try
        {
          tf_buffer_.transform((*it1)->position_, dest_loc,fixed_frame);
          // tfl_.transformPoint((*it1)->id_, (*it2)->position_.stamp_, (*it2)->position_, fixed_frame, dest_loc);
        }
        catch (...)
        {
          RCLCPP_WARN(this->get_logger(), "TF exception getting distance between legs.");
        }
        dist_between_legs = std::hypot(dest_loc.point.x, dest_loc.point.y);

        // If this is the closest dist (and within range), and the legs are close together and unlabeled, mark it.
        if (dist_between_legs < leg_pair_separation_m)
        {
          closest = it1;
          closest_dist = (*it1)->dist_to_person_;
          closest_dist_between_legs = dist_between_legs;
        }
      }
      // If we found a close, unlabeled leg, set it's label.
      if (closest != end)
      {
        std::cout << "Replaced one leg with a distance of " << closest_dist
                  << " and a distance between the legs of " << closest_dist_between_legs << std::endl;
        (*closest)->object_id = people_meas->object_id;
      }
      else
      {
        std::cout << "Returned one matched leg only" << std::endl;
      }

      // Regardless of whether we found a second leg, return.
      return;
    }

    std::cout << "Looking for a pair of new legs" << std::endl;
    // If we didn't find any legs with this person's label,
    // try to find two unlabeled legs that are close together and close to the tracker.
    it1 = saved_features_.begin();
    it2 = saved_features_.begin();
    closest = saved_features_.end();
    closest1 = saved_features_.end();
    closest2 = saved_features_.end();
    closest_dist = max_meas_jump_m;
    closest_pair_dist = 2 * max_meas_jump_m;
    for (; it1 != end; ++it1)
    {
      // Only look at trackers without ids and that are not too far away.
      if ((*it1)->object_id != "" || (*it1)->dist_to_person_ >= max_meas_jump_m)
        continue;

      // Keep the single closest leg around in case none of the pairs work out.
      if ((*it1)->dist_to_person_ < closest_dist)
      {
        closest_dist = (*it1)->dist_to_person_;
        closest = it1;
      }

      // Find a second leg.
      it2 = it1;
      it2++;
      for (; it2 != end; ++it2)
      {
        // Only look at trackers without ids and that are not too far away.
        if ((*it2)->object_id != "" || (*it2)->dist_to_person_ >= max_meas_jump_m)
          continue;

        // Get the distance between the two legs
        try {
          std::chrono::nanoseconds time(rclcpp::Time(people_meas->header.stamp).nanoseconds());
          tf_buffer_.transform((*it2)->position_,dest_loc, (*it1)->id_, tf2::TimePoint(time), fixed_frame);
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "TF exception getting distance between legs in spot 2.");
        }
        dist_between_legs = std::hypot(dest_loc.point.x, dest_loc.point.y);

        // Ensure that this pair of legs is the closest pair to the tracker,
        // and that the distance between the legs isn't too large.
        if ((*it1)->dist_to_person_ + (*it2)->dist_to_person_ < closest_pair_dist &&
            dist_between_legs < leg_pair_separation_m)
        {
          closest_pair_dist = (*it1)->dist_to_person_ + (*it2)->dist_to_person_;
          closest1 = it1;
          closest2 = it2;
          closest_dist_between_legs = dist_between_legs;
        }
      }
    }
    // Found a pair of legs.
    if (closest1 != end && closest2 != end)
    {
      (*closest1)->object_id = people_meas->object_id;
      (*closest2)->object_id = people_meas->object_id;
      std::cout << "Found a completely new pair with total distance " << closest_pair_dist
                << " and a distance between the legs of " << closest_dist_between_legs << std::endl;
      return;
    }

    std::cout << "Looking for just one leg" << std::endl;
    // No pair worked, try for just one leg.
    if (closest != end)
    {
      (*closest)->object_id = people_meas->object_id;
      std::cout << "Returned one new leg only" << std::endl;
      return;
    }

    std::cout << "Nothing matched" << std::endl;
  }

  void pairLegs()
  {
    // Deal With legs that already have ids
    std::list<SavedFeature*>::iterator begin = saved_features_.begin();
    std::list<SavedFeature*>::iterator end = saved_features_.end();
    std::list<SavedFeature*>::iterator leg1, leg2, best, it;

    for (leg1 = begin; leg1 != end; ++leg1)
    {
      // If this leg has no id, skip
      if ((*leg1)->object_id == "")
        continue;

      leg2 = end;
      best = end;
      double closest_dist = leg_pair_separation_m;
      for (it = begin; it != end; ++it)
      {
        if (it == leg1) continue;

        if ((*it)->object_id == (*leg1)->object_id)
        {
          leg2 = it;
          break;
        }

        if ((*it)->object_id != "")
          continue;

        double d = distance(it, leg1);
        if (((*it)->getLifetime() <= max_second_leg_age_s)
            && (d < closest_dist))
        {
          closest_dist = d;
          best = it;
        }
      }

      if (leg2 != end)
      {
        double dist_between_legs = distance(leg1, leg2);
        if (dist_between_legs > leg_pair_separation_m)
        {
          (*leg1)->object_id = "";
          (*leg1)->other = NULL;
          (*leg2)->object_id = "";
          (*leg2)->other = NULL;
        }
        else
        {
          (*leg1)->other = *leg2;
          (*leg2)->other = *leg1;
        }
      }
      else if (best != end)
      {
        (*best)->object_id = (*leg1)->object_id;
        (*leg1)->other = *best;
        (*best)->other = *leg1;
      }
    }

    // Attempt to pair up legs with no id
    for (;;)
    {
      std::list<SavedFeature*>::iterator best1 = end, best2 = end;
      double closest_dist = leg_pair_separation_m;

      for (leg1 = begin; leg1 != end; ++leg1)
      {
        // If this leg has an id or low reliability, skip
        if ((*leg1)->object_id != ""
            || (*leg1)->getReliability() < leg_reliability_limit_)
          continue;

        for (leg2 = begin; leg2 != end; ++leg2)
        {
          if (((*leg2)->object_id != "")
              || ((*leg2)->getReliability() < leg_reliability_limit_)
              || (leg1 == leg2)) continue;
          double d = distance(leg1, leg2);
          if (d < closest_dist)
          {
            best1 = leg1;
            best2 = leg2;
          }
        }
      }

      if (best1 != end)
      {
        char id[100];
        snprintf(id, sizeof(id), "Person%d", next_p_id_++);
        (*best1)->object_id = std::string(id);
        (*best2)->object_id = std::string(id);
        (*best1)->other = *best2;
        (*best2)->other = *best1;
      }
      else
      {
        break;
      }
    }
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    laser_processor::ScanProcessor processor(*scan, mask_);

    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);

    cv::Mat tmp_mat = cv::Mat(1, feat_count_, CV_32FC1);

    // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
    rclcpp::Time purge = scan->header.stamp + rclcpp::Duration::from_seconds(-no_observation_timeout_s);
    std::list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end())
    {
      if ((*sf_iter)->meas_time_ < purge)
      {
        if ((*sf_iter)->other)
          (*sf_iter)->other->other = NULL;
        delete *sf_iter;
        saved_features_.erase(sf_iter++);
      }
      else
        ++sf_iter;
    }


    // System update of trackers, and copy updated ones in propagate list
    std::list<SavedFeature*> propagated;
    for (std::list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++)
    {
      (*sf_iter)->propagate(scan->header.stamp);
      propagated.push_back(*sf_iter);
    }


    // Detection step: build up the set of "candidate" clusters
    // For each candidate, find the closest tracker (within threshold) and add to the match list
    // If no tracker is found, start a new one
    std::multiset<MatchedFeature> matches;
    for (std::list<laser_processor::SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      std::vector<float> f = calcLegFeatures(*i, *scan);

      memcpy(tmp_mat.data, f.data(), f.size()*sizeof(float));

      float probability = 0.5 + 0.5 *
                          static_cast<float>(forest->predict(tmp_mat, cv::noArray(), cv::ml::RTrees::PREDICT_SUM)) /
                          static_cast<float>(forest->getRoots().size());

      geometry_msgs::msg::PointStamped loc;
      loc.header = scan->header;
      loc.point.x = (*i)->center()[0];
      loc.point.y = (*i)->center()[1];
      loc.point.z = (*i)->center()[2];
      try {
        loc.header.stamp = rclcpp::Time();
        tf_buffer_.transform(loc, loc, fixed_frame);
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "TF exception spot 3.");
      }

      std::list<SavedFeature*>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_m;

      for (std::list<SavedFeature*>::iterator pf_iter = propagated.begin();
           pf_iter != propagated.end();
           pf_iter++)
      {
        // find the closest distance between candidate and trackers
        float dist = std::hypot(loc.point.x - (*pf_iter)->position_.point.x,
                                loc.point.y - (*pf_iter)->position_.point.y);
        // float dist = loc.distance((*pf_iter)->position_);
        if (dist < closest_dist)
        {
          closest = pf_iter;
          closest_dist = dist;
        }
      }
      // Nothing close to it, start a new track
      if (closest == propagated.end())
      {
        std::list<SavedFeature*>::iterator new_saved =
          saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tf_buffer_));
      }
      // Add the candidate, the tracker and the distance to a match list
      else
        matches.insert(MatchedFeature(*i, *closest, closest_dist, probability));
    }

    // loop through _sorted_ matches list
    // find the match with the shortest distance for each tracker
    while (matches.size() > 0)
    {
      std::multiset<MatchedFeature>::iterator matched_iter = matches.begin();
      bool found = false;
      std::list<SavedFeature*>::iterator pf_iter = propagated.begin();
      while (pf_iter != propagated.end())
      {
        // update the tracker with this candidate
        if (matched_iter->closest_ == *pf_iter)
        {
          // Transform candidate to fixed frame
          geometry_msgs::msg::PointStamped loc;
          loc.header = scan->header;
          loc.point.x = matched_iter->candidate_->center()[0];
          loc.point.y = matched_iter->candidate_->center()[1];
          loc.point.z = matched_iter->candidate_->center()[2];
          // (matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
          try
          {
            loc.header.stamp = rclcpp::Time();
            tf_buffer_.transform(loc, loc, fixed_frame);
            // tfl_.transformPoint(fixed_frame, loc, loc);
          }
          catch (...)
          {
            RCLCPP_WARN(this->get_logger(), "TF exception spot 4.");
            // ROS_WARN("TF exception spot 4.");
          }

          // Update the tracker with the candidate location
          matched_iter->closest_->update(loc, matched_iter->probability_);

          // remove this match and
          matches.erase(matched_iter);
          propagated.erase(pf_iter++);
          found = true;
          break;
        }
        // still looking for the tracker to update
        else
        {
          pf_iter++;
        }
      }

      // didn't find tracker to update, because it was deleted above
      // try to assign the candidate to another tracker
      if (!found)
      {
        geometry_msgs::msg::PointStamped loc;
        loc.header = scan->header;
        loc.point.x = matched_iter->candidate_->center()[0];
        loc.point.y = matched_iter->candidate_->center()[1];
        loc.point.z = matched_iter->candidate_->center()[2];
        // (matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
        try
        {
          loc.header.stamp = rclcpp::Time();
          tf_buffer_.transform(loc, loc, fixed_frame);
          // tfl_.transformPoint(fixed_frame, loc, loc);
        }
        catch (...)
        {
          RCLCPP_WARN(this->get_logger(), "TF exception spot 5.");
          // ROS_WARN("TF exception spot 5.");
        }

        std::list<SavedFeature*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;

        for (std::list<SavedFeature*>::iterator remain_iter = propagated.begin();
             remain_iter != propagated.end();
             remain_iter++)
        {
          float dist = std::hypot(loc.point.x - (*remain_iter)->position_.point.x,
                                  loc.point.y - (*remain_iter)->position_.point.y);
          // float dist = loc.distance((*remain_iter)->position_);
          if (dist < closest_dist)
          {
            closest = remain_iter;
            closest_dist = dist;
          }
        }

        // no tracker is within a threshold of this candidate
        // so create a new tracker for this candidate
        if (closest == propagated.end())
          std::list<SavedFeature*>::iterator new_saved =
            saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tf_buffer_));
        else
          matches.insert(MatchedFeature(matched_iter->candidate_, *closest, closest_dist, matched_iter->probability_));
        matches.erase(matched_iter);
      }
    }

    if (!use_seeds_)
      pairLegs();

    // Publish Data!
    int i = 0;
    std::vector<people_msgs::msg::PositionMeasurement> people;
    std::vector<people_msgs::msg::PositionMeasurement> legs;
    std::vector<visualization_msgs::msg::Marker> markers;

    for (std::list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++, i++)
    {
      // reliability
      double reliability = (*sf_iter)->getReliability();

      if ((*sf_iter)->getReliability() > leg_reliability_limit_
          && publish_legs_)
      {
        people_msgs::msg::PositionMeasurement pos;
        pos.header.stamp = scan->header.stamp;
        pos.header.frame_id = fixed_frame;
        pos.name = "leg_detector";
        pos.object_id = (*sf_iter)->id_;
        pos.pos = (*sf_iter)->position_.point;
        pos.reliability = reliability;
        pos.covariance[0] = pow(0.3 / reliability, 2.0);
        pos.covariance[1] = 0.0;
        pos.covariance[2] = 0.0;
        pos.covariance[3] = 0.0;
        pos.covariance[4] = pow(0.3 / reliability, 2.0);
        pos.covariance[5] = 0.0;
        pos.covariance[6] = 0.0;
        pos.covariance[7] = 0.0;
        pos.covariance[8] = 10000.0;
        pos.initialization = 0;
        legs.push_back(pos);
      }

      if (publish_leg_markers_)
      {
        visualization_msgs::msg::Marker m;
        m.header.stamp = (*sf_iter)->time_;
        m.header.frame_id = fixed_frame;
        m.ns = "LEGS";
        m.id = i;
        m.type = m.SPHERE;
        m.pose.position = (*sf_iter)->position_.point;

        m.scale.x = .1;
        m.scale.y = .1;
        m.scale.z = .1;
        m.color.a = 1;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        if ((*sf_iter)->object_id != "")
        {
          m.color.r = 1;
        }
        else
        {
          m.color.b = (*sf_iter)->getReliability();
        }

        markers.push_back(m);
      }

      if (publish_people_ || publish_people_markers_)
      {
        SavedFeature* other = (*sf_iter)->other;
        if (other != NULL && other < (*sf_iter))
        {
          double dx = ((*sf_iter)->position_.point.x + other->position_.point.x) / 2,
                 dy = ((*sf_iter)->position_.point.y + other->position_.point.y) / 2,
                 dz = ((*sf_iter)->position_.point.z + other->position_.point.z) / 2;

          if (publish_people_)
          {
            reliability = reliability * other->reliability;
            people_msgs::msg::PositionMeasurement pos;
            pos.header.stamp = (*sf_iter)->time_;
            pos.header.frame_id = fixed_frame;
            pos.name = (*sf_iter)->object_id;;
            pos.object_id = (*sf_iter)->id_ + "|" + other->id_;
            pos.pos.x = dx;
            pos.pos.y = dy;
            pos.pos.z = dz;
            pos.reliability = reliability;
            pos.covariance[0] = pow(0.3 / reliability, 2.0);
            pos.covariance[0] = std::isinf(pos.covariance[0]) ? 10000.0 : pos.covariance[0];
            pos.covariance[1] = 0.0;
            pos.covariance[2] = 0.0;
            pos.covariance[3] = 0.0;
            pos.covariance[4] = pow(0.3 / reliability, 2.0);
            pos.covariance[4] = std::isinf(pos.covariance[0]) ? 10000.0 : pos.covariance[0];
            pos.covariance[5] = 0.0;
            pos.covariance[6] = 0.0;
            pos.covariance[7] = 0.0;
            pos.covariance[8] = 10000.0;
            pos.initialization = 0;
            people.push_back(pos);
          }

          if (publish_people_markers_)
          {
            visualization_msgs::msg::Marker m;
            m.header.stamp = (*sf_iter)->time_;
            m.header.frame_id = fixed_frame;
            m.ns = "PEOPLE";
            m.id = i;
            m.type = m.SPHERE;
            m.pose.position.x = dx;
            m.pose.position.y = dy;
            m.pose.position.z = dz;
            m.scale.x = .2;
            m.scale.y = .2;
            m.scale.z = .2;
            m.color.a = 1;
            m.color.g = 1;
            m.lifetime = rclcpp::Duration::from_seconds(0.5);

            markers.push_back(m);
          }
        }
      }
    }

    people_msgs::msg::PositionMeasurementArray array;
    array.header.stamp = this->get_clock()->now();
    if (publish_legs_)
    {
      array.people = legs;
      leg_measurements_pub_->publish(array);
    }
    if (publish_people_)
    {
      array.people = people;
      people_measurements_pub_->publish(array);
    }
    if (publish_leg_markers_ || publish_people_markers_)
    {
      visualization_msgs::msg::MarkerArray msg;
      msg.markers = markers;
      markers_pub_->publish(msg);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LegDetector>(argc, argv);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
