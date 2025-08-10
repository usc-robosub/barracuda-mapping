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
#include <XmlRpcValue.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

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
      cloud_subs_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(t, 1, boost::bind(&SlamNode::cloudCallback, this, _1)));
      ROS_INFO_STREAM("Subscribing to pointcloud topic: " << t);
    }
    collision_srv_ = nh_.advertiseService("check_collision", &SlamNode::checkCollision, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

    ROS_INFO_STREAM("SlamNode parameters: map_frame='" << map_frame_
                    << "' base_frame='" << base_frame_
                    << "' odometry_topic='" << odom_topic_ << "'.");

    auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01,0.01,0.01,0.01,0.01,0.01).finished());
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', pose_idx_), gtsam::Pose3(), noise));
    initial_.insert(gtsam::Symbol('x', pose_idx_), gtsam::Pose3());
    isam_.update(graph_, initial_);
    graph_.resize(0);
    initial_.clear();
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
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
    sensor_msgs::PointCloud2 transformed;
    tf2::doTransform(*msg, transformed, tf);

    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(transformed, tmp);
    std::size_t before = map_cloud_->size();
    *map_cloud_ += tmp;
    ROS_INFO_STREAM_THROTTLE(1.0, "Added " << tmp.size() << " points to map ("
                                      << before << " -> " << map_cloud_->size() << ").");

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
    ROS_INFO_STREAM_THROTTLE(1.0, "Published odometry on '" << odom_topic_ << "' at "
                                      << odom.header.stamp.toSec());

  }

  bool checkCollision(barracuda_mapping::CheckCollision::Request &req,
                      barracuda_mapping::CheckCollision::Response &res) {
    Eigen::Vector3d center(req.center.x, req.center.y, req.center.z);
    ROS_INFO_STREAM_THROTTLE(1.0, "check_collision called: center=[" << center.transpose()
                                      << "] r=" << req.radius << ". Map size=" << map_cloud_->size());
    for (const auto &pt : map_cloud_->points) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      if ((p - center).norm() <= req.radius) {
        res.collision = true;
        ROS_INFO_THROTTLE(1.0, "Collision detected.");
        return true;
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
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  size_t pose_idx_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_topic_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gtsam_slam_node");
  SlamNode node;
  ros::spin();
  return 0;
}
