
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class CostmapFiller : public rclcpp::Node
{
public:
  CostmapFiller() : Node("costmap_filler")
  {
    // Create a subscriber for the point cloud data
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud",
      10,
      std::bind(&CostmapFiller::cloud_callback, this, std::placeholders::_1));
    
    // Create a publisher for the occupancy grid
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "occupancy_grid", 10);
  }
  
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    // Convert the point cloud data to an occupancy grid
    // ...
    
    // Publish the occupancy grid
    grid_pub_->publish(grid);
  }
  
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapFiller>());
  rclcpp::shutdown();
  return 0;
}

