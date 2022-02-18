#include "nav2_costmap_2d/ray_tracer.hpp"

namespace nav2_costmap_2d
{
RayTracer::RayTracer()
{
}

void RayTracer::initialize(
    rclcpp::Node::SharedPtr & parent_node,
    std::string camera_info_topic_name,
    std::string aligned_pc2_topic_name,
    bool use_pointcloud,
    tf2_ros::Buffer * tf2_buffer,
    std::string global_frame,
    std::string sensor_frame,
    tf2::Duration tf_tolerance,
    double max_trace_distance
)
{
    parent_node_ = parent_node;
    global_frame_ = global_frame;
    sensor_frame_ = sensor_frame; 
    tf2_buffer_ = tf2_buffer;
    camera_info_topic_name_ = camera_info_topic_name;
    aligned_pc2_topic_name_ = aligned_pc2_topic_name;
    tf_tolerance_ = tf_tolerance;
    use_pointcloud_ = use_pointcloud;
    max_trace_distance_ = max_trace_distance;
    if(use_pointcloud_)
    {
        aligned_pc2_sub_ = parent_node_->create_subscription<sensor_msgs::msg::PointCloud2>(aligned_pc2_topic_name_, rclcpp::SensorDataQoS(), std::bind(&RayTracer::pointCloudCb, this, std::placeholders::_1));
        RCLCPP_INFO(parent_node_->get_logger(), "Subscribed to %s", aligned_pc2_topic_name_.c_str());
    }
    else
    {
        camera_info_sub_ = parent_node_->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_name_, rclcpp::SensorDataQoS(), std::bind(&RayTracer::cameraInfoCb, this, std::placeholders::_1));
        RCLCPP_INFO(parent_node_->get_logger(), "Subscribed to %s", camera_info_topic_name_.c_str());
    }
}

std::vector<geometry_msgs::msg::Point> RayTracer::rayTracePixels(std::vector<cv::Point>& pixels)
{   
    std::vector<geometry_msgs::msg::Point> ans = std::vector<geometry_msgs::msg::Point>();
    if(!ready_to_raytrace_)
    {
        RCLCPP_WARN(parent_node_->get_logger(), "No messages received yet on the topic");
        return ans;
    }
    try
    {
        geometry_msgs::msg::TransformStamped sensor_to_global = tf2_buffer_->lookupTransform(sensor_frame_, global_frame_, rclcpp::Time(0));
        Eigen::Isometry3d eigen_transform = tf2::transformToEigen(sensor_to_global.transform);
        for(auto& pixel : pixels)
        {
            cv::Point3d cv_ray = camera_model_.projectPixelTo3dRay(pixel);
            geometry_msgs::msg::Point p = intersectWithGroundPlane(eigen_transform, cv_ray);
            ans.push_back(p);
        }
        return ans;
    }
    catch(tf2::TransformException & ex)
    {
        RCLCPP_ERROR(
        parent_node_->get_logger(),
        "TF Exception that should never happen for sensor frame: %s, global_frame: %s, %s",
        sensor_frame_.c_str(),
        global_frame_.c_str(), ex.what());
        return ans;
    }
}

geometry_msgs::msg::Point RayTracer::intersectWithGroundPlane(Eigen::Isometry3d camera_to_ground_tf, cv::Point3d cv_ray)
{
    Eigen::Vector3d eigen_ray({cv_ray.x, cv_ray.y, cv_ray.z});
    Eigen::Vector3d camera_origin = camera_to_ground_tf.translation();
    Eigen::Vector3d ray_end = camera_to_ground_tf*eigen_ray;
    Eigen::Vector3d ground_plane_origin({0.0,  0.0, 0.0});
    Eigen::Vector3d ground_plane_normal({0.0, 0.0, 1.0});
    Eigen::Vector3d u = ray_end - camera_origin;
    Eigen::Vector3d w = camera_origin - ground_plane_origin;
    double D = ground_plane_normal.dot(u);
    double N = -ground_plane_normal.dot(w);
    if(abs(D) < 0.0000001){
        return geometry_msgs::msg::Point();
    }
    double sI = N / D;
    Eigen::Vector3d intersection = camera_origin + sI * u;
    return tf2::toMsg(intersection);
}

cv::Point2d RayTracer::worldToImage(geometry_msgs::msg::PointStamped& point){
    if (point.header.frame_id != sensor_frame_)
    {
        RCLCPP_INFO(
        parent_node_->get_logger(), "Point has to be on the %s frame but is in frame %s",
        sensor_frame_.c_str(), point.header.frame_id.c_str());
        return cv::Point2d();
    }
    cv::Point3d cv_point{point.point.x, point.point.y, point.point.z};
    cv::Point2d pixel = camera_model_.project3dToPixel(cv_point);
    return pixel;
}

void RayTracer::pointCloudCb(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pointcloud_msg_ = *msg;
    ready_to_raytrace_ = true;
}

void RayTracer::cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg) 
{
    RCLCPP_INFO(parent_node_->get_logger(), "Received camera info message. Ready to raytrace");
    camera_model_.fromCameraInfo(*msg);
    camera_info_sub_.reset();
    ready_to_raytrace_ = true;
}




}  // namespace nav2_costmap_2d