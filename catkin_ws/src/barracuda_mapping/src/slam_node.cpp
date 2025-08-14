#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <string>
#include <vector>
#include <deque>
#include <memory>
#include <XmlRpcValue.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <cmath>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "barracuda_mapping/CheckCollision.h"

class SlamNode {
public:
  SlamNode()
    : tf_listener_(tf_buffer_), map_cloud_(new pcl::PointCloud<pcl::PointXYZ>()), pose_idx_(0) {
    ros::NodeHandle pnh("~");
    std::vector<std::string> topics;
    pnh.param<std::string>("map_frame", map_frame_, std::string("map"));
    pnh.param<std::string>("base_frame", base_frame_, std::string("base_link"));
    pnh.param<std::string>("odometry_topic", odom_topic_, std::string("slam/odometry"));
    // NDT odometry configuration
    pnh.param("ndt_enabled", ndt_enabled_, true);
    pnh.param<std::string>("ndt_odometry_input_topic", ndt_odom_input_topic_, std::string(""));
    pnh.param("ndt_resolution", ndt_resolution_, 0.5);
    pnh.param("ndt_step_size", ndt_step_size_, 0.1);
    pnh.param("ndt_epsilon", ndt_epsilon_, 1e-6);
    pnh.param("ndt_max_iterations", ndt_max_iter_, 40);
    pnh.param("ndt_use_tf_init", ndt_use_tf_init_, true);
    // NDT keyframe and outlier filtering
    pnh.param("ndt_keyframe_enabled", ndt_keyframe_enabled_, true);
    pnh.param("ndt_keyframe_trans_thresh", ndt_kf_trans_thresh_, 0.2);
    pnh.param("ndt_keyframe_rot_thresh_deg", ndt_kf_rot_thresh_deg_, 5.0);
    pnh.param("ndt_outlier_enabled", ndt_outlier_enabled_, true);
    pnh.param<std::string>("ndt_outlier_method", ndt_outlier_method_, std::string("sor"));
    pnh.param("ndt_sor_mean_k", ndt_sor_mean_k_, 20);
    pnh.param("ndt_sor_stddev_mul", ndt_sor_stddev_mul_, 1.0);
    pnh.param("ndt_ror_radius", ndt_ror_radius_, 0.5);
    pnh.param("ndt_ror_min_neighbors", ndt_ror_min_neighbors_, 2);

    // NDT submap (target aggregation) to mitigate scan-to-scan drift
    pnh.param("ndt_submap_enabled", ndt_submap_enabled_, true);
    pnh.param("ndt_submap_max_keyframes", ndt_submap_max_keyframes_, 5);
    pnh.param("ndt_submap_voxel_leaf", ndt_submap_voxel_leaf_, 0.2);
    double resolution;
    pnh.param("octomap_resolution", resolution, 0.1);
    octree_ = std::make_unique<octomap::OcTree>(resolution);

    // Downsampling parameters (voxel grid)
    pnh.param("downsample_enabled", downsample_enabled_, true);
    pnh.param("downsample_leaf_size", downsample_leaf_size_, resolution);
    // Parse pointcloud topic(s) robustly: support list or single string.
    // Prefer ~pointcloud_topics (YAML list or string). Fallback to ~pointcloud_topic.
    XmlRpc::XmlRpcValue pc_param;
    if (pnh.getParam("pointcloud_topics", pc_param)) {
      if (pc_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < pc_param.size(); ++i) {
          if (pc_param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            topics.emplace_back(static_cast<std::string>(pc_param[i]));
          } else {
            ROS_WARN("~pointcloud_topics[%d] is not a string; ignoring.", i);
          }
        }
      } else if (pc_param.getType() == XmlRpc::XmlRpcValue::TypeString) {
        topics.emplace_back(static_cast<std::string>(pc_param));
      } else {
        ROS_WARN("~pointcloud_topics has unexpected type; expected list or string. Falling back.");
      }
    }
    if (topics.empty()) {
      std::string single_topic;
      if (pnh.getParam("pointcloud_topic", single_topic) && !single_topic.empty()) {
        topics.push_back(single_topic);
      }
    }
    if (topics.empty()) {
      ROS_WARN("No valid pointcloud topic provided; defaulting to '/points'.");
      topics.push_back("/points");
    }
    ROS_INFO_STREAM("Configured pointcloud topics (" << topics.size() << "):");
    for (const auto &t : topics) ROS_INFO_STREAM("  - " << t);
    for (const auto &t : topics) {
      cloud_subs_.push_back(
          nh_.subscribe<sensor_msgs::PointCloud2>(
              t, 1, boost::bind(&SlamNode::cloudCallback, this, _1, t)));
      ROS_INFO_STREAM("Subscribing to pointcloud topic: " << t);
    }
    pnh.param("publish_tf", publish_tf_, false);
    if (ndt_enabled_) {
      if (ndt_odom_input_topic_.empty() && !topics.empty()) {
        ndt_odom_input_topic_ = topics.front();
        ROS_WARN_STREAM("~ndt_odometry_input_topic not set; defaulting to first topic: "
                        << ndt_odom_input_topic_);
      }
      ROS_INFO_STREAM("NDT odometry enabled. Input topic='" << ndt_odom_input_topic_
                      << "' res=" << ndt_resolution_ << " step=" << ndt_step_size_
                      << " eps=" << ndt_epsilon_ << " max_iter=" << ndt_max_iter_
                      << " use_tf_init=" << (ndt_use_tf_init_ ? "true" : "false") << ".");
      ndt_pose_.setIdentity();
      keyframe_pose_.setIdentity();
      ndt_pose_initialized_ = false;
      submap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
      submap_keyframes_.clear();
    }
    collision_srv_ = nh_.advertiseService("check_collision", &SlamNode::checkCollision, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    octomap_full_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    octomap_binary_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);

    ROS_INFO_STREAM("SlamNode parameters: map_frame='" << map_frame_
                    << "' base_frame='" << base_frame_
                    << "' odometry_topic='" << odom_topic_
                    << "' downsample_enabled=" << (downsample_enabled_ ? "true" : "false")
                    << " downsample_leaf_size=" << downsample_leaf_size_ << ".");

    auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01,0.01,0.01,0.01,0.01,0.01).finished());
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', pose_idx_), gtsam::Pose3(), noise));
    initial_.insert(gtsam::Symbol('x', pose_idx_), gtsam::Pose3());
    isam_.update(graph_, initial_);
    graph_.resize(0);
    initial_.clear();
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const std::string &topic) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Received cloud: frame='" << msg->header.frame_id
                                  << "' stamp=" << msg->header.stamp.toSec()
                                  << " size=" << msg->width * msg->height);
    geometry_msgs::TransformStamped tf;
    // Try to get transform at message time; fall back to latest if unavailable
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
      ROS_INFO_STREAM_THROTTLE(1.0, "TF lookup OK at stamp from '" << msg->header.frame_id
                                        << "' to '" << map_frame_ << "'.");
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(2.0, "TF at stamp unavailable (%s). Falling back to latest.", ex.what());
      try {
        tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
        ROS_INFO_STREAM_THROTTLE(1.0, "TF fallback OK (latest) from '" << msg->header.frame_id
                                          << "' to '" << map_frame_ << "'.");
      } catch (tf2::TransformException &ex2) {
        ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s", ex2.what());
        return;
      }
    }
    // Prepare two versions of the cloud:
    // 1) raw (sensor frame) for NDT odometry
    // 2) map-frame for OctoMap insertion (from NDT pose if available, else TF)
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);
    pcl::PointCloud<pcl::PointXYZ> map_frame_cloud;
    if (ndt_enabled_ && ndt_pose_initialized_) {
      pcl::transformPointCloud(raw_cloud, map_frame_cloud, ndt_pose_);
    } else {
      sensor_msgs::PointCloud2 transformed;
      tf2::doTransform(*msg, transformed, tf);
      pcl::fromROSMsg(transformed, map_frame_cloud);
    }

    // Downsample via voxel grid if enabled
    pcl::PointCloud<pcl::PointXYZ> processed_map;
    if (downsample_enabled_) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(static_cast<float>(downsample_leaf_size_),
                     static_cast<float>(downsample_leaf_size_),
                     static_cast<float>(downsample_leaf_size_));
      vg.setInputCloud(map_frame_cloud.makeShared());
      vg.filter(processed_map);
    } else {
      processed_map = std::move(map_frame_cloud);
    }

    std::size_t before = map_cloud_->size();
    map_cloud_->reserve(map_cloud_->size() + processed_map.size());
    *map_cloud_ += processed_map;
    ROS_INFO_STREAM_THROTTLE(1.0, "Added " << processed_map.size() << " points to map ("
                                      << before << " -> " << map_cloud_->size() << ").");

    // Update Octomap
    octomap::Pointcloud octo_cloud;
    octo_cloud.reserve(processed_map.size());
    for (const auto &pt : processed_map.points) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
        octo_cloud.push_back(pt.x, pt.y, pt.z);
    }
    octomap::point3d origin(0.f, 0.f, 0.f);
    if (ndt_enabled_ && ndt_pose_initialized_) {
      origin = octomap::point3d(ndt_pose_(0,3), ndt_pose_(1,3), ndt_pose_(2,3));
    } else {
      origin = octomap::point3d(tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z);
    }
    octree_->insertPointCloud(octo_cloud, origin);
    octree_->updateInnerOccupancy();

    // Publish OctoMap for visualization (full and binary)
    octomap_msgs::Octomap full_msg;
    if (octomap_msgs::fullMapToMsg(*octree_, full_msg)) {
      full_msg.header.frame_id = map_frame_;
      full_msg.header.stamp = msg->header.stamp;
      octomap_full_pub_.publish(full_msg);
    }
    octomap_msgs::Octomap bin_msg;
    if (octomap_msgs::binaryMapToMsg(*octree_, bin_msg)) {
      bin_msg.header.frame_id = map_frame_;
      bin_msg.header.stamp = msg->header.stamp;
      octomap_binary_pub_.publish(bin_msg);
    }

    // NDT odometry (if enabled and this topic is selected)
    if (ndt_enabled_ && topic == ndt_odom_input_topic_) {
      ndt_sensor_frame_ = msg->header.frame_id;
      // Filter raw cloud for NDT input (outliers + downsample)
      pcl::PointCloud<pcl::PointXYZ>::Ptr src_filt = filterForNDT(raw_cloud);

      if (!ndt_pose_initialized_) {
        // Initialize pose and keyframe
        ndt_pose_.setIdentity();
        keyframe_pose_.setIdentity();
        last_keyframe_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*src_filt));
        ndt_pose_initialized_ = true;
        publishOdomFromPose(msg->header.stamp);
      } else {
        // Choose registration target: submap (preferred), else keyframe, else last cloud
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr target;
        if (ndt_submap_enabled_ && submap_cloud_ && !submap_cloud_->empty()) {
          target = submap_cloud_;
        } else if (last_keyframe_cloud_) {
          target = last_keyframe_cloud_;
        } else {
          target = last_odom_cloud_;
        }
        if (!target) {
          // Fallback in case keyframe is missing
          last_keyframe_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*src_filt));
          publishOdomFromPose(msg->header.stamp);
        } else {
          pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
          ndt.setTransformationEpsilon(static_cast<float>(ndt_epsilon_));
          ndt.setStepSize(static_cast<float>(ndt_step_size_));
          ndt.setResolution(static_cast<float>(ndt_resolution_));
          ndt.setMaximumIterations(ndt_max_iter_);
          ndt.setInputTarget(target);
          ndt.setInputSource(src_filt);

          Eigen::Matrix4f init = Eigen::Matrix4f::Identity();
          if (ndt_use_tf_init_) {
            // Use TF delta between keyframe and current as initial guess
            try {
              geometry_msgs::TransformStamped tf_k;
              tf_k = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
              Eigen::Matrix4f curr_tf = tf2::transformToEigen(tf_k).matrix().cast<float>();
              // initial guess relative to keyframe
              init = keyframe_pose_.inverse() * curr_tf;
            } catch (tf2::TransformException &ex) {
              // keep identity
              ROS_WARN_THROTTLE(2.0, "TF init for NDT failed: %s", ex.what());
            }
          }

          pcl::PointCloud<pcl::PointXYZ> aligned;
          ndt.align(aligned, init);
          if (!ndt.hasConverged()) {
            ROS_WARN_THROTTLE(1.0, "NDT did not converge; keeping previous pose.");
          } else {
            Eigen::Matrix4f kf_T_curr = ndt.getFinalTransformation(); // current in keyframe frame
            ndt_pose_ = keyframe_pose_ * kf_T_curr;                   // current in map frame

            // Update last processed cloud
            last_odom_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*src_filt));

            // Keyframe decision
            if (ndt_keyframe_enabled_) {
              Eigen::Matrix4f kf_T_est = keyframe_pose_.inverse() * ndt_pose_;
              Eigen::Vector3f t = kf_T_est.block<3,1>(0,3);
              float trans = t.norm();
              Eigen::Matrix3f R = kf_T_est.block<3,3>(0,0);
              float angle = Eigen::AngleAxisf(R).angle() * 180.0f / static_cast<float>(M_PI);
              if (trans > ndt_kf_trans_thresh_ || angle > ndt_kf_rot_thresh_deg_) {
                last_keyframe_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*src_filt));
                keyframe_pose_ = ndt_pose_;
                // Update submap with new keyframe points in map frame
                if (ndt_submap_enabled_) {
                  addKeyframeToSubmap(*src_filt, ndt_pose_);
                }
                ROS_INFO_STREAM_THROTTLE(1.0, "New NDT keyframe: trans=" << trans << " m, rot=" << angle << " deg");
              }
            }

            // Publish odometry/TF from NDT pose
            publishOdomFromPose(msg->header.stamp);
          }
        }
      }
    }

    // If NDT is disabled or this is not the odom topic, fall back to TF-based odom
    if (!ndt_enabled_) {
      ++pose_idx_;
      // Convert TF to Eigen Isometry and then to GTSAM Pose3 via 4x4 matrix
      Eigen::Isometry3d T_eigen = tf2::transformToEigen(tf);
      gtsam::Pose3 current(T_eigen.matrix());
      gtsam::Pose3 previous = isam_.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', pose_idx_-1));
      gtsam::Pose3 between = previous.between(current);
      auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1,0.1,0.1,0.1,0.1,0.1).finished());
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', pose_idx_-1), gtsam::Symbol('x', pose_idx_), between, noise));
      initial_.insert(gtsam::Symbol('x', pose_idx_), current);
      isam_.update(graph_, initial_);
      graph_.resize(0);
      initial_.clear();

      gtsam::Pose3 estimate = isam_.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', pose_idx_));
      Eigen::Matrix4d m = estimate.matrix();
      Eigen::Isometry3d iso(m);
      geometry_msgs::Pose pose_msg = tf2::toMsg(iso);

      // Log current pose
      Eigen::Vector3d t = iso.translation();
      Eigen::Quaterniond q(iso.rotation());
      ROS_INFO_STREAM_THROTTLE(1.0, "Pose idx=" << pose_idx_
                                << " t=[" << t.x() << ", " << t.y() << ", " << t.z() << "]"
                                << " q=[" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]");

      nav_msgs::Odometry odom;
      odom.header.stamp = msg->header.stamp;
      odom.header.frame_id = map_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose = pose_msg;
      odom_pub_.publish(odom);
      ROS_INFO_STREAM_THROTTLE(1.0, "Published odometry (TF-based fallback) on '" << odom_topic_ << "' at "
                                        << odom.header.stamp.toSec());
    }

  }

  bool checkCollision(barracuda_mapping::CheckCollision::Request &req,
                      barracuda_mapping::CheckCollision::Response &res) {
    Eigen::Vector3d center(req.center.x, req.center.y, req.center.z);
    ROS_INFO_STREAM_THROTTLE(1.0, "check_collision called: center=[" << center.transpose()
                                      << "] r=" << req.radius << ". Octomap size="
                                      << octree_->getNumLeafNodes());

    const double r2 = req.radius * req.radius;
    const octomap::point3d min(center.x() - req.radius,
                               center.y() - req.radius,
                               center.z() - req.radius);
    const octomap::point3d max(center.x() + req.radius,
                               center.y() + req.radius,
                               center.z() + req.radius);
    for (octomap::OcTree::leaf_bbx_iterator it = octree_->begin_leafs_bbx(min, max);
         it != octree_->end_leafs_bbx(); ++it) {
      if (it->getOccupancy() >= octree_->getOccupancyThres()) {
        // getCoordinate() returns by value; do not bind to a reference
        const octomap::point3d p = it.getCoordinate();
        const double dx = p.x() - center.x();
        const double dy = p.y() - center.y();
        const double dz = p.z() - center.z();
        if (dx*dx + dy*dy + dz*dz <= r2) {
          res.collision = true;
          ROS_INFO_THROTTLE(1.0, "Collision detected.");
          return true;
        }
      }
    }
    res.collision = false;
    ROS_INFO_THROTTLE(1.0, "No collision.");
    return true;
  }

  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> cloud_subs_;
  ros::ServiceServer collision_srv_;
  ros::Publisher odom_pub_;
  ros::Publisher octomap_full_pub_;
  ros::Publisher octomap_binary_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  std::unique_ptr<octomap::OcTree> octree_;
  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  size_t pose_idx_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  // Downsampling configuration
  bool downsample_enabled_;
  double downsample_leaf_size_;
  // NDT odometry state/config
  bool ndt_enabled_;
  std::string ndt_odom_input_topic_;
  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_epsilon_;
  int ndt_max_iter_;
  bool ndt_use_tf_init_;
  bool ndt_pose_initialized_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_odom_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_keyframe_cloud_;
  Eigen::Matrix4f ndt_pose_;
  Eigen::Matrix4f keyframe_pose_;
  std::string ndt_sensor_frame_;
  // Keyframe config
  bool ndt_keyframe_enabled_;
  double ndt_kf_trans_thresh_;
  double ndt_kf_rot_thresh_deg_;
  // Outlier filtering config
  bool ndt_outlier_enabled_;
  std::string ndt_outlier_method_;
  int ndt_sor_mean_k_;
  double ndt_sor_stddev_mul_;
  double ndt_ror_radius_;
  int ndt_ror_min_neighbors_;
  // Submap config/state
  bool ndt_submap_enabled_;
  int ndt_submap_max_keyframes_;
  double ndt_submap_voxel_leaf_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr submap_cloud_;
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> submap_keyframes_;

  // TF publish
  bool publish_tf_;

  void publishOdomFromPose(const ros::Time& stamp) {
    // Map->Sensor from NDT
    Eigen::Matrix4d m_map_sensor = ndt_pose_.cast<double>();
    Eigen::Isometry3d T_map_sensor(m_map_sensor);

    // Sensor->Base from TF (static preferred)
    Eigen::Isometry3d T_sensor_base = Eigen::Isometry3d::Identity();
    if (!ndt_sensor_frame_.empty() && ndt_sensor_frame_ != base_frame_) {
      try {
        // Prefer latest for static transform; fallback if stamped not available
        geometry_msgs::TransformStamped tf_sb = tf_buffer_.lookupTransform(ndt_sensor_frame_, base_frame_, ros::Time(0));
        T_sensor_base = tf2::transformToEigen(tf_sb);
      } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(2.0, "Sensor->base TF missing (%s); assuming identity.", ex.what());
      }
    }

    Eigen::Isometry3d T_map_base = T_map_sensor * T_sensor_base;
    geometry_msgs::Pose pose_msg = tf2::toMsg(T_map_base);
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose_msg;
    odom_pub_.publish(odom);
    ROS_INFO_STREAM_THROTTLE(1.0, "Published NDT odometry on '" << odom_topic_ << "' at "
                                      << odom.header.stamp.toSec());

    (void)publish_tf_; // intentionally unused; TF is handled by EKF nodes
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterForNDT(const pcl::PointCloud<pcl::PointXYZ>& in) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(in));
    if (ndt_outlier_enabled_) {
      if (ndt_outlier_method_ == "ror") {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setRadiusSearch(static_cast<float>(ndt_ror_radius_));
        ror.setMinNeighborsInRadius(ndt_ror_min_neighbors_);
        ror.setInputCloud(cloud);
        pcl::PointCloud<pcl::PointXYZ> temp; ror.filter(temp);
        *cloud = std::move(temp);
      } else {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setMeanK(ndt_sor_mean_k_);
        sor.setStddevMulThresh(static_cast<float>(ndt_sor_stddev_mul_));
        sor.setInputCloud(cloud);
        pcl::PointCloud<pcl::PointXYZ> temp; sor.filter(temp);
        *cloud = std::move(temp);
      }
    }
    if (downsample_enabled_) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(static_cast<float>(downsample_leaf_size_),
                     static_cast<float>(downsample_leaf_size_),
                     static_cast<float>(downsample_leaf_size_));
      vg.setInputCloud(cloud);
      pcl::PointCloud<pcl::PointXYZ> temp; vg.filter(temp);
      *cloud = std::move(temp);
    }
    return cloud;
  }

  void addKeyframeToSubmap(const pcl::PointCloud<pcl::PointXYZ>& keyframe_sensor_cloud,
                           const Eigen::Matrix4f& T_map_sensor) {
    if (!submap_cloud_) submap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    // Transform to map frame
    pcl::PointCloud<pcl::PointXYZ> kf_map;
    pcl::transformPointCloud(keyframe_sensor_cloud, kf_map, T_map_sensor);
    // Voxel downsample before adding
    pcl::PointCloud<pcl::PointXYZ>::Ptr kf_ds(new pcl::PointCloud<pcl::PointXYZ>());
    if (ndt_submap_voxel_leaf_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(static_cast<float>(ndt_submap_voxel_leaf_),
                     static_cast<float>(ndt_submap_voxel_leaf_),
                     static_cast<float>(ndt_submap_voxel_leaf_));
      vg.setInputCloud(kf_map.makeShared());
      vg.filter(*kf_ds);
    } else {
      *kf_ds = kf_map;
    }
    // Push to deque and rebuild submap cloud
    submap_keyframes_.push_back(kf_ds);
    while (static_cast<int>(submap_keyframes_.size()) > ndt_submap_max_keyframes_) {
      submap_keyframes_.pop_front();
    }
    rebuildSubmap();
  }

  void rebuildSubmap() {
    if (!submap_cloud_) submap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    submap_cloud_->clear();
    for (const auto& kf : submap_keyframes_) {
      *submap_cloud_ += *kf;
    }
    // Optional final voxel downsample to keep size reasonable
    if (ndt_submap_voxel_leaf_ > 0.0 && !submap_cloud_->empty()) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(static_cast<float>(ndt_submap_voxel_leaf_),
                     static_cast<float>(ndt_submap_voxel_leaf_),
                     static_cast<float>(ndt_submap_voxel_leaf_));
      vg.setInputCloud(submap_cloud_);
      pcl::PointCloud<pcl::PointXYZ> temp; vg.filter(temp);
      *submap_cloud_ = std::move(temp);
    }
    ROS_INFO_STREAM_THROTTLE(1.0, "Submap updated: points=" << submap_cloud_->size()
                                  << " keyframes=" << submap_keyframes_.size());
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gtsam_slam_node");
  SlamNode node;
  ros::spin();
  return 0;
}
