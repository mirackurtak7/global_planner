#include "nav2_straightline_planner/straight_line_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  // Polygon topic'ine abone ol
  polygon_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/polygon", rclcpp::QoS(10),
    std::bind(&StraightLine::polygonCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "StraightLine planner configured.");
}

void StraightLine::cleanup()
{
  polygon_regions_.clear();
  RCLCPP_INFO(node_->get_logger(), "StraightLine planner cleaned up.");
}

void StraightLine::activate()
{
  RCLCPP_INFO(node_->get_logger(), "StraightLine planner activated.");
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "StraightLine planner deactivated.");
}

void StraightLine::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(polygon_mutex_);
  polygon_regions_.clear();

  PolygonRegion region;
  for (const auto &point : msg->polygon.points) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    region.vertices.push_back(p);
  }
  region.orientation = msg->polygon.points[0].z;  // Z eksenini yönelim olarak kullan
  region.additional_cost = 10.0; // Örnek maliyet
  polygon_regions_.push_back(region);

  RCLCPP_INFO(node_->get_logger(), "Polygon region received.");
}

bool StraightLine::isPointInsidePolygon(const geometry_msgs::msg::Point &point, const PolygonRegion &polygon)
{
  int n = polygon.vertices.size();
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    if ((polygon.vertices[i].y > point.y) != (polygon.vertices[j].y > point.y) &&
        (point.x < (polygon.vertices[j].x - polygon.vertices[i].x) *
                   (point.y - polygon.vertices[i].y) /
                   (polygon.vertices[j].y - polygon.vertices[i].y) +
                   polygon.vertices[i].x)) {
      inside = !inside;
    }
  }
  return inside;
}

double StraightLine::calculateAngleDifference(double path_angle, double polygon_orientation)
{
  double diff = std::fmod(std::abs(path_angle - polygon_orientation), 360.0);
  return (diff > 180.0) ? 360.0 - diff : diff;
}

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.frame_id = global_frame_;
  global_path.header.stamp = node_->now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal pose is not in the global frame.");
    return global_path;
  }

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double path_angle = std::atan2(dy, dx) * 180.0 / M_PI;

  // Çokgenleri kontrol et ve maliyeti güncelle
  std::lock_guard<std::mutex> lock(polygon_mutex_);
  for (const auto &region : polygon_regions_) {
    if (isPointInsidePolygon(goal.pose.position, region)) {
      double angle_diff = calculateAngleDifference(path_angle, region.orientation);
      if (angle_diff < 90.0) {  // Yumuşak geçiş için eşik
        RCLCPP_INFO(node_->get_logger(), "Path passes through polygon region.");
      }
    }
  }

  geometry_msgs::msg::PoseStamped pose = start;
  pose.pose.orientation.w = 1.0;
  global_path.poses.push_back(pose);

  pose = goal;
  pose.pose.orientation.w = 1.0;
  global_path.poses.push_back(pose);

  return global_path;
}

}  // namespace nav2_straightline_planner

PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)

