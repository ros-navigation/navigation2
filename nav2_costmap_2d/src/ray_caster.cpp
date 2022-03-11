#include "nav2_costmap_2d/ray_caster.hpp"

namespace nav2_costmap_2d
{
RayCaster::RayCaster()
{
}

void RayCaster::initialize(
    rclcpp::Node::SharedPtr & parent_node,
    std::string camera_info_topic_name,
    std::string aligned_pc2_topic_name,
    bool use_pointcloud,
    tf2_ros::Buffer * tf2_buffer,
    std::string global_frame,
    tf2::Duration tf_tolerance,
    double max_trace_distance
)
{
    parent_node_ = parent_node;
    global_frame_ = global_frame;
    
    tf2_buffer_ = tf2_buffer;
    camera_info_topic_name_ = camera_info_topic_name;
    aligned_pc2_topic_name_ = aligned_pc2_topic_name;
    tf_tolerance_ = tf_tolerance;
    use_pointcloud_ = use_pointcloud;
    max_trace_distance_ = max_trace_distance;
    if(use_pointcloud_)
    {
        aligned_pc2_sub_ = parent_node_->create_subscription<sensor_msgs::msg::PointCloud2>(aligned_pc2_topic_name_, rclcpp::SensorDataQoS(), std::bind(&RayCaster::pointCloudCb, this, std::placeholders::_1));
        RCLCPP_INFO(parent_node_->get_logger(), "Subscribed to %s", aligned_pc2_topic_name_.c_str());
    }
    else
    {
        camera_info_sub_ = parent_node_->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_name_, rclcpp::SensorDataQoS(), std::bind(&RayCaster::cameraInfoCb, this, std::placeholders::_1));
        RCLCPP_INFO(parent_node_->get_logger(), "Subscribed to %s", camera_info_topic_name_.c_str());
    }
}

bool RayCaster::worldToImage(geometry_msgs::msg::PointStamped& point, cv::Point2d& pixel){
    if (!tf2_buffer_->canTransform(
        point.header.frame_id, sensor_frame_, tf2_ros::fromMsg(point.header.stamp)))
    {
        RCLCPP_ERROR(
        parent_node_->get_logger(), "Ray caster can't transform from %s to %s",
        sensor_frame_.c_str(), point.header.frame_id.c_str());
        return false;
    }
    geometry_msgs::msg::PointStamped transformed_point;
    tf2_buffer_->transform(point, transformed_point, sensor_frame_, tf_tolerance_);
    cv::Point3d cv_point{transformed_point.point.x, transformed_point.point.y, transformed_point.point.z};
    pixel = camera_model_.project3dToPixel(cv_point);
    return true;
}

bool RayCaster::imageToGroundPlane(cv::Point2d& pixel, geometry_msgs::msg::PointStamped& point)
{
    // (void) sensor_to_world_tf;
    if (!tf2_buffer_->canTransform(
        global_frame_, sensor_frame_, tf2_ros::fromMsg(point.header.stamp)))
    {
        RCLCPP_ERROR_THROTTLE(
        parent_node_->get_logger(), *parent_node_->get_clock(), 10000, "Ray caster can't transform from %s to %s",
        sensor_frame_.c_str(), point.header.frame_id.c_str());
        return false;
    }
    cv::Point3d cv_ray = camera_model_.projectPixelTo3dRay(pixel);
    geometry_msgs::msg::PointStamped ray_end_sensor_frame = rayToPointStamped(cv_ray, sensor_frame_);
    geometry_msgs::msg::PointStamped ray_end_world_frame;
    geometry_msgs::msg::PointStamped camera_position_world_frame;
    tf2_buffer_->transform(ray_end_sensor_frame, ray_end_world_frame, global_frame_, tf_tolerance_);
    tf2_buffer_->transform(camera_origin_, camera_position_world_frame, global_frame_, tf_tolerance_);
    if(ray_end_world_frame.point.z > camera_position_world_frame.point.z)
    {
        RCLCPP_DEBUG(parent_node_->get_logger(),"Point falls behind the camera, will not raycast");
        return false;
    }
    double ray_scale_factor = -camera_position_world_frame.point.z/(ray_end_world_frame.point.z - camera_position_world_frame.point.z);
    geometry_msgs::msg::PointStamped intersection;
    intersection.header.frame_id = global_frame_;
    intersection.point.x = camera_position_world_frame.point.x + ray_scale_factor*(ray_end_world_frame.point.x - camera_position_world_frame.point.x);
    intersection.point.y = camera_position_world_frame.point.y + ray_scale_factor*(ray_end_world_frame.point.y - camera_position_world_frame.point.y);
    intersection.point.z = camera_position_world_frame.point.z + ray_scale_factor*(ray_end_world_frame.point.z - camera_position_world_frame.point.z);
    geometry_msgs::msg::PointStamped intersection_camera_frame = tf2_buffer_->transform(intersection, sensor_frame_);
    if(intersection_camera_frame.point.z > max_trace_distance_)
    {
        RCLCPP_DEBUG(parent_node_->get_logger(),"Point falls further than the max trace distance, will not raycast");
        return false;
    }
    point = intersection;
    return true;
}

bool RayCaster::imageToGroundPlaneLookup(cv::Point2d& pixel, geometry_msgs::msg::PointStamped& point, geometry_msgs::msg::TransformStamped sensor_to_world_tf)
{
    if(!ready_to_raytrace_)
    {
        RCLCPP_WARN_THROTTLE(parent_node_->get_logger(), *parent_node_->get_clock(), 10000, "Not ready to raytrace");
        return false;
    }
    if (!tf2_buffer_->canTransform(
        global_frame_, sensor_frame_, tf2_ros::fromMsg(point.header.stamp)))
    {
        RCLCPP_ERROR_THROTTLE(
        parent_node_->get_logger(), *parent_node_->get_clock(), 10000, "Ray caster can't transform from %s to %s",
        sensor_frame_.c_str(), point.header.frame_id.c_str());
        return false;
    }
    geometry_msgs::msg::PointStamped point_sensor_frame = caster_lookup_table_.at(pixel.y*img_width_+pixel.x);
    if(point_sensor_frame.point.z > max_trace_distance_)
    {
        RCLCPP_DEBUG(parent_node_->get_logger(),"Point falls further than the max trace distance, will not raycast");
        return false;
    }
    if(point_sensor_frame.point.z < 0.0)
    {
        RCLCPP_DEBUG(parent_node_->get_logger(),"Point falls falls behind the camera, will not raycast");
        return false;
    }
    geometry_msgs::msg::PointStamped point_world_frame;
    tf2::doTransform(
      point_sensor_frame, point_world_frame, sensor_to_world_tf);
    // tf2_buffer_->transform(point_sensor_frame, point_world_frame, global_frame_, tf_tolerance_);
    point = point_world_frame;
    return true;
}

geometry_msgs::msg::PointStamped RayCaster::rayToPointStamped(cv::Point3d& ray_end, std::string& frame_id)
{
    geometry_msgs::msg::PointStamped point;
    point.point.x = ray_end.x;
    point.point.y = ray_end.y;
    point.point.z = ray_end.z;
    point.header.frame_id = frame_id;
    return point;
}

void RayCaster::pointCloudCb(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pointcloud_msg_ = *msg;
    ready_to_raytrace_ = true;
}

void RayCaster::cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg) 
{
    RCLCPP_INFO(parent_node_->get_logger(), "Troll");
    if(msg->header.frame_id.empty())
    {
        RCLCPP_ERROR(parent_node_->get_logger(), "Frame id of camera info message is empty. Cannot raycast");
        return;
    }
    camera_model_.fromCameraInfo(*msg);
    sensor_frame_ = msg->header.frame_id; 
    camera_origin_.header.frame_id = sensor_frame_;
    img_width_ = msg->width;
    img_height_ = msg->height;
    if(!computeLookupTable(msg->header.stamp))
    {
        RCLCPP_WARN(parent_node_->get_logger(), "Could not compute lookup table");
        return;
    }
    camera_info_sub_.reset();
    ready_to_raytrace_ = true;
    RCLCPP_INFO(parent_node_->get_logger(), "Received camera info message. Ready to raycast");
}


bool RayCaster::computeLookupTable(builtin_interfaces::msg::Time & time_msg)
{
    if (!tf2_buffer_->canTransform(
        global_frame_, sensor_frame_, tf2_ros::fromMsg(time_msg)))
    {
        RCLCPP_ERROR_THROTTLE(
        parent_node_->get_logger(), *parent_node_->get_clock(), 10000, "Ray caster can't transform from %s to %s",
        sensor_frame_.c_str(), global_frame_.c_str());
        return false;
    }
    caster_lookup_table_.clear();
    caster_lookup_table_.reserve(img_width_*img_height_);
    for(size_t v = 0; v < img_height_; v++)
    {
        for(size_t u = 0; u < img_width_; u++)
        {
            cv::Point2d pixel;
            pixel.x = u;
            pixel.y = v;
            cv::Point3d cv_ray = camera_model_.projectPixelTo3dRay(pixel);
            geometry_msgs::msg::PointStamped ray_end_sensor_frame = rayToPointStamped(cv_ray, sensor_frame_);
            geometry_msgs::msg::PointStamped ray_end_world_frame;
            geometry_msgs::msg::PointStamped camera_position_world_frame;
            tf2_buffer_->transform(ray_end_sensor_frame, ray_end_world_frame, global_frame_, tf_tolerance_);
            tf2_buffer_->transform(camera_origin_, camera_position_world_frame, global_frame_, tf_tolerance_);
            double ray_scale_factor = -camera_position_world_frame.point.z/(ray_end_world_frame.point.z - camera_position_world_frame.point.z);
            geometry_msgs::msg::PointStamped intersection;
            intersection.header.frame_id = global_frame_;
            intersection.point.x = camera_position_world_frame.point.x + ray_scale_factor*(ray_end_world_frame.point.x - camera_position_world_frame.point.x);
            intersection.point.y = camera_position_world_frame.point.y + ray_scale_factor*(ray_end_world_frame.point.y - camera_position_world_frame.point.y);
            intersection.point.z = camera_position_world_frame.point.z + ray_scale_factor*(ray_end_world_frame.point.z - camera_position_world_frame.point.z);
            geometry_msgs::msg::PointStamped intersection_camera_frame = tf2_buffer_->transform(intersection, sensor_frame_);
            caster_lookup_table_.emplace_back(intersection_camera_frame);
        }
    }
    RCLCPP_INFO(parent_node_->get_logger(), "Lookup table computed successfully");
    return true;
}



}  // namespace nav2_costmap_2d