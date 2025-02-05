#pragma once

#include <cmath>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/link.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/visual.pb.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <shape_msgs/msg/detail/solid_primitive__struct.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>

#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/scene.pb.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ament_index_cpp/get_package_share_directory.hpp>


class RVizGazeboBridge
{
public:
    RVizGazeboBridge(const std::string& robot_name);
    void load_from_rviz_to_gazebo();
    void load_from_gazebo_to_rviz();

private:
    int spawn_xacro(const std::string& command, double x, double y, double z, double qx, double qy, double qz, double qw, const std::string& name); //x y z (position to spawn) qx qy qz qw (orientation to spawn - quaternion)
    void gazebo_spawn_box(const std::string& name, double x_dim, double y_dim, double z_dim, double x, double y, double z, double qx, double qy, double qz, double qw); //x y z dimensions (size), x y z (position to spawn) qx qy qz qw (orientation to spawn - quaternion)
    void gazebo_spawn_cylinder(const std::string& name, double radius, double height, double x, double y, double z, double qx, double qy, double qz, double qw);
    void gazebo_spawn_sphere(const std::string& name, double radius, double x, double y, double z, double qx, double qy, double qz, double qw);
    void init();
    std::string parse_filename_to_url(std::string filename); 

    const std::string package_share_path = ament_index_cpp::get_package_share_directory("rviz_gazebo_bridge");
    const std::string robot_name;
    std::string world_name;
    bool is_initialized = false;

};