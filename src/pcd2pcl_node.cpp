#include "pcd_convert_pcl_ros2/pcd2pcl.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

Pcd2Pcl::Pcd2Pcl() : Node("pcd_to_pcl_node")
{
    void load_pcd_map(std::string path ,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd);
    void convert_pcd_to_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd);
    void setparam();
    map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud",10);
    pub_flag = true;
    down_sampling_flag=true;
}
void Pcd2Pcl::setparam()
{
    this->declare_parameter("map_path","map_path.pcd");
    this->declare_parameter("frame_id","map");
    this->declare_parameter("use_downsampling",true);
    this->declare_parameter("voxel_leafsize",1.0);

    this->get_parameter("map_path",map_path);
    this->get_parameter("frame_id",frame_id);
    this->get_parameter("use_downsampling",down_sampling_flag);
    this->get_parameter("voxel_leafsize",leaf_size);
}
void Pcd2Pcl::load_pcd_map(std::string path ,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd)
{
    RCLCPP_INFO(get_logger(),"load map_cloud from:%s",path.c_str());
    if(pcl::io::loadPCDFile(path, *cloud_pcd))
    {
        RCLCPP_INFO(get_logger(),"Success load map_cloud from:%s",path.c_str());
    }

    else
    {
        RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", path.c_str());
    }
}
void Pcd2Pcl::convert_pcd_to_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd,sensor_msgs::msg::PointCloud2::SharedPtr cloud_pcl)
{
    // sensor_msgs::msg::PointCloud2::SharedPtr pcl_cloud;
    pcl::toROSMsg(*cloud_pcd,*cloud_pcl);
    rclcpp::Time current_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    // msg->header.stamp = ros_clock.now();
    cloud_pcl->header.stamp = current_stamp;
    cloud_pcl->header.frame_id = frame_id;
}
void Pcd2Pcl::loop()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr down_cloud_pcd(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::msg::PointCloud2::SharedPtr map_cloud_ptr(new sensor_msgs::msg::PointCloud2);
    // if(pub_flag)
    // {
        RCLCPP_INFO(get_logger(),"ready publish pcd->pcl");
        // load_pcd_map(map_path, map_pcd);
        // convert_pcd_to_pcl(map_pcd,map_cloud);
        pcl::io::loadPCDFile(map_path,*cloud_pcd);
        RCLCPP_INFO(get_logger(), "Initial point cloud data points: %lu", cloud_pcd->points.size());
        if(down_sampling_flag)
        {
            pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
            voxelGrid.setInputCloud(cloud_pcd);
            voxelGrid.setLeafSize(leaf_size,leaf_size,leaf_size);
            voxelGrid.filter(*down_cloud_pcd);
            RCLCPP_INFO(get_logger(), "down sampling point cloud data points: %lu", down_cloud_pcd->points.size());
            pcl::toROSMsg(*down_cloud_pcd,*map_cloud_ptr);
            rclcpp::Time current_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            // msg->header.stamp = ros_clock.now();
            map_cloud_ptr->header.stamp = current_stamp;
            map_cloud_ptr->header.frame_id = frame_id;
            map_cloud_pub->publish(*map_cloud_ptr);
        }
        else
        {
            // convert_pcd_to_pcl(map_pcd,map_cloud);
            pcl::toROSMsg(*cloud_pcd,*map_cloud_ptr);
            rclcpp::Time current_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            // msg->header.stamp = ros_clock.now();
            map_cloud_ptr->header.stamp = current_stamp;
            map_cloud_ptr->header.frame_id = frame_id;
            map_cloud_pub->publish(*map_cloud_ptr);
        }
        pub_flag = false;
    // }
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pcd2Pcl>();

  node->setparam();
  rclcpp::Rate loop_rate(0.1);
  //
  while (rclcpp::ok())
  {
    node->loop();
    rclcpp::spin_some(node);
    loop_rate.sleep();

  }
  rclcpp::shutdown();
  return 0;
}