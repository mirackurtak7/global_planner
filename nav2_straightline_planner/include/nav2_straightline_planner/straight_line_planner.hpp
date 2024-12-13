#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_straightline_planner
{

// Çokgen bölgeleri temsil eden yapı
struct PolygonRegion
{
  std::vector<geometry_msgs::msg::Point> vertices; // Çokgenin köşe noktaları
  double orientation;                              // Çokgenin yönelimi
  double additional_cost;                          // Çokgenin ek maliyeti
};

class StraightLine : public nav2_core::GlobalPlanner
{
public:
  StraightLine() = default;
  ~StraightLine() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) override;

private:
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  bool isPointInsidePolygon(const geometry_msgs::msg::Point &point, const PolygonRegion &polygon);
  double calculateAngleDifference(double path_angle, double polygon_orientation);

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
  std::vector<PolygonRegion> polygon_regions_; // Tüm çokgen bölgeleri saklar
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;
  std::string global_frame_;
  std::mutex polygon_mutex_;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::string name_;
};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

