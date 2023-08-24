#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class Pcd2Pcl : public rclcpp::Node
{
public:
    Pcd2Pcl();
    void loop();
    void setparam();
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub;
    sensor_msgs::msg::PointCloud2::SharedPtr map_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pcd;
    std::string map_path,frame_id;
    double loop_rate;
    bool pub_flag;
    void load_pcd_map(std::string path ,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd);
    void convert_pcd_to_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd, sensor_msgs::msg::PointCloud2::SharedPtr cloud_pcl);
    // void setparam();


};
