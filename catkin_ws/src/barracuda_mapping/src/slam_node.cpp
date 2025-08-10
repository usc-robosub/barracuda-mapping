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
    pnh.getParam("pointcloud_topics", topics);
    if (topics.empty()) {
      topics.push_back("/points");
    }
    for (const auto &t : topics) {
      cloud_subs_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(t, 1, boost::bind(&SlamNode::cloudCallback, this, _1)));
    }
    collision_srv_ = nh_.advertiseService("check_collision", &SlamNode::checkCollision, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("slam/odometry", 10);

    auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01,0.01,0.01,0.01,0.01,0.01).finished());
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', pose_idx_), gtsam::Pose3(), noise));
    initial_.insert(gtsam::Symbol('x', pose_idx_), gtsam::Pose3());
    isam_.update(graph_, initial_);
    graph_.resize(0);
    initial_.clear();
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    sensor_msgs::PointCloud2 transformed;
    tf2::doTransform(*msg, transformed, tf);

    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(transformed, tmp);
    *map_cloud_ += tmp;

    ++pose_idx_;
    gtsam::Pose3 current(tf2::transformToEigen(tf));
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

    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose_msg;
    odom_pub_.publish(odom);

  }

  bool checkCollision(barracuda_mapping::CheckCollision::Request &req,
                      barracuda_mapping::CheckCollision::Response &res) {
    Eigen::Vector3d center(req.center.x, req.center.y, req.center.z);
    for (const auto &pt : map_cloud_->points) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      if ((p - center).norm() <= req.radius) {
        res.collision = true;
        return true;
      }
    }
    res.collision = false;
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
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gtsam_slam_node");
  SlamNode node;
  ros::spin();
  return 0;
}

