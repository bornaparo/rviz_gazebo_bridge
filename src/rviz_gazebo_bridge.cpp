#include "rviz_gazebo_bridge/rviz_gazebo_bridge.h"
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <rclcpp/logging.hpp>
#include <stdexcept>

static rclcpp::Logger LOGGER = rclcpp::get_logger("rviz_gazebo_bridge");

RVizGazeboBridge::RVizGazeboBridge(const std::string& robot_name) : robot_name(robot_name)
{
    std::string service_name = "/gazebo/worlds";
    ignition::transport::Node transport_node;
    ignition::msgs::Empty req;
    ignition::msgs::StringMsg_V res;
    bool request_result = false;
    unsigned int timeout = 5000; //ms
    bool success = transport_node.Request(service_name, req, timeout, res, request_result); 
    // RCLCPP_INFO(LOGGER, "[MOJE] RVizGazeboBridge(), res.data(0): %s", res.data(0).c_str());
    // RCLCPP_INFO(LOGGER, "[MOJE] RVizGazeboBridge(), success: %d", success);
    // RCLCPP_INFO(LOGGER, "[MOJE] RVizGazeboBridge(), request_result: %d", request_result);
    if (!success || !request_result || !res.data().size())
    {
        RCLCPP_ERROR(LOGGER, "failed to call service: %s, probably ign gazebo isn't running", service_name.c_str());
        throw std::runtime_error("failed getting ign gazebo world name, probably ign gazebo isn't running");
    }
    else
    {
        this->world_name = res.data(0);
        RCLCPP_INFO(LOGGER, "world_name is: %s", this->world_name.c_str());
    }
};


void RVizGazeboBridge::gazebo_spawn_sphere(const std::string& name, double radius, double x, double y, double z, double qx, double qy, double qz, double qw)
{
    std::string file_path = this->package_share_path + "/models/sphere.urdf.xacro"; //uzet package-share od models
    std::ostringstream command;
    command << "xacro " << file_path << " radius:=" << radius << " -o /tmp/tmp.urdf";
    RCLCPP_INFO_STREAM(LOGGER, "command: " + command.str());
    int result = this->spawn_xacro(command.str(), x, y, z, qx, qy, qz, qw, name);
    if (result == 1)
    {
        RCLCPP_INFO_STREAM(LOGGER, "Sphere spawned successfully!");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Error spawning sphere!");
    }
}

void RVizGazeboBridge::gazebo_spawn_cylinder(const std::string& name, double radius, double height, double x, double y, double z, double qx, double qy, double qz, double qw)
{
    std::string file_path = this->package_share_path + "/models/cylinder.urdf.xacro";
    std::ostringstream command;
    command << "xacro " << file_path << " radius:=" << radius << " height:=" << height << " -o /tmp/tmp.urdf"; 
    RCLCPP_INFO_STREAM(LOGGER, "command: " + command.str());

    int result = this->spawn_xacro(command.str(), x, y, z, qx, qy, qz, qw, name);
    if (result == 1)
    {
        RCLCPP_INFO_STREAM(LOGGER, "Cylinder spawned successfully!");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Error spawning cylinder!");
    }
}

void RVizGazeboBridge::gazebo_spawn_box(const std::string& name, double x_dim=1, double y_dim=1, double z_dim=1, double x=0, double y=0, double z=2, double qx=0, double qy=0, double qz=0, double qw=1)
{
    std::string file_path = this->package_share_path + "/models/box.urdf.xacro"; 
    std::ostringstream command; 
    command << "xacro " << file_path << " box_size:='" << x_dim << " " << y_dim << " " << z_dim << "' -o /tmp/tmp.urdf"; 
    RCLCPP_INFO_STREAM(LOGGER, "command: " + command.str());

    int result = this->spawn_xacro(command.str(), x, y, z, qx, qy, qz, qw, name);
    if (result == 1)
    {
        RCLCPP_INFO_STREAM(LOGGER, "Box spawned successfully!");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Error spawning box!");
    }

}

int RVizGazeboBridge::spawn_xacro(const std::string& command, double x, double y, double z, double qx=0, double qy=0, double qz=0, double qw=1, const std::string& name="model")
{
    int result = std::system(command.c_str());

    if (result == 0) {
        RCLCPP_INFO_STREAM(LOGGER, "Successfully generated URDF from XACRO and outputed it to /tmp/tmp.urdf.");
    } else {
        RCLCPP_ERROR(LOGGER, "Failed to generate URDF from XACRO.");
        return -1;
    }
    
    std::ifstream file("/tmp/tmp.urdf"); //otvara taj filepath
    if (!file.is_open())
    {
        RCLCPP_ERROR(LOGGER, "Error opening file '/tmp/tmp.urdf'");
        return -1;
    }
    std::ostringstream urdf_content;
    urdf_content << file.rdbuf(); //reads whole file
    if (file.bad())
    {
        RCLCPP_ERROR(LOGGER, "Error reading the file '/tmp/tmp.urdf'");
        return -1;
    }
    file.close();

    std::string service_name = "/world/" + this->world_name + "/create";
    ignition::transport::Node transport_node;
    ignition::msgs::EntityFactory req;
    
    req.set_sdf(urdf_content.str()); //radi also for urdf, not just sdf
    req.set_name(name.c_str()); //set unique name
    ignition::msgs::Pose* pose = req.mutable_pose(); 
    pose->mutable_position()->set_x(x); 
    pose->mutable_position()->set_y(y); 
    pose->mutable_position()->set_z(z);
    pose->mutable_orientation()->set_x(qx);
    pose->mutable_orientation()->set_y(qy);
    pose->mutable_orientation()->set_z(qz);
    pose->mutable_orientation()->set_w(qw);

    bool request_result = false;
    ignition::msgs::Boolean res;
    unsigned int timeout = 5000; //ms
    bool success = transport_node.Request(service_name, req, timeout, res, request_result);

    if (!success || !request_result || !res.data())
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to spawn " + name);
        throw std::runtime_error("Failed to spawn " + name + ", stopping execution.");
    }

    return 1;
}


void RVizGazeboBridge::load_from_rviz_to_gazebo() //takes all collision objects from motion planning scene and spawns them in gazebo
{
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto &collision_objects = psi.getObjects();

    // Iterate through the objects and print their IDs
    for (const auto &pair : collision_objects)
    {
        RCLCPP_INFO_STREAM(LOGGER, "Collision Object ID: " << pair.first);
        const auto &collision_object = pair.second;
        
        switch(collision_object.primitives[0].type) 
        {
            case shape_msgs::msg::SolidPrimitive::BOX:
            {
                RCLCPP_INFO_STREAM(LOGGER, "  collision object type is: box");
                double x_dim = collision_object.primitives[0].dimensions[0];
                double y_dim = collision_object.primitives[0].dimensions[1];
                double z_dim = collision_object.primitives[0].dimensions[2];
                double x = collision_object.pose.position.x;
                double y = collision_object.pose.position.y;
                double z = collision_object.pose.position.z;
                double qx = collision_object.pose.orientation.x;
                double qy = collision_object.pose.orientation.y;
                double qz = collision_object.pose.orientation.z;
                double qw = collision_object.pose.orientation.w;

                this->gazebo_spawn_box(pair.first, x_dim, y_dim, z_dim, x, y, z, qx, qy, qz, qw);
                break;

            }
            case shape_msgs::msg::SolidPrimitive::SPHERE:
                {
                RCLCPP_INFO_STREAM(LOGGER, "  collision object type is: sphere");
                double radius = collision_object.primitives[0].dimensions[0];
                double x = collision_object.pose.position.x;
                double y = collision_object.pose.position.y;
                double z = collision_object.pose.position.z;
                double qx = collision_object.pose.orientation.x;
                double qy = collision_object.pose.orientation.y;
                double qz = collision_object.pose.orientation.z;
                double qw = collision_object.pose.orientation.w;

                this->gazebo_spawn_sphere(pair.first, radius, x, y, z, qx, qy, qz, qw);
                break;
                }
            case shape_msgs::msg::SolidPrimitive::CYLINDER:
            {
                RCLCPP_INFO_STREAM(LOGGER, "  collision object type is: cylinder");
                double radius = collision_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
                double height = collision_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
                double x = collision_object.pose.position.x;
                double y = collision_object.pose.position.y;
                double z = collision_object.pose.position.z;
                double qx = collision_object.pose.orientation.x;
                double qy = collision_object.pose.orientation.y;
                double qz = collision_object.pose.orientation.z;
                double qw = collision_object.pose.orientation.w;

                this->gazebo_spawn_cylinder(pair.first, radius, height, x, y, z, qx, qy, qz, qw);
                break;
            }
            case shape_msgs::msg::SolidPrimitive::CONE:
                RCLCPP_INFO_STREAM(LOGGER, "  collision object type is: CONE");
                RCLCPP_WARN_STREAM(LOGGER, "  loading CONE from rviz to gazebo is not yet implemented");
                break;
            case shape_msgs::msg::SolidPrimitive::PRISM:
                RCLCPP_INFO_STREAM(LOGGER, "  collision object type is: PRISM");
                RCLCPP_WARN_STREAM(LOGGER, "  loading PRISM from rviz to gazebo is not yet implemented");
                break;
            default:
                RCLCPP_INFO_STREAM(LOGGER, "  non common collision object type");
                RCLCPP_WARN_STREAM(LOGGER, "  loading non common collision object from rviz to gazebo is not yet implemented");
                break;
        }
    }
}


std::string RVizGazeboBridge::parse_filename_to_url(std::string filename)
{
    std::string to_replace = " ";
    std::string replacement = "%20";
    size_t pos = 0;
    while ((pos = filename.find(to_replace, pos)) != std::string::npos) //replace space with %20
    {
        filename.replace(pos, to_replace.length(), replacement);
        pos += replacement.length(); // Move past the replacement
    }

    if(filename.find("package://") != std::string::npos) { 
        return filename;
    }

    if(filename.find("file://") != std::string::npos) { 
        return filename;
    }

    if(filename.find("http://") != std::string::npos) { 
        return filename;
    }
    
    if(filename.find("https://") != std::string::npos) { 
        return filename;
    }

    return "file://" + filename;
}

void RVizGazeboBridge::load_from_gazebo_to_rviz()
{
    RCLCPP_INFO_STREAM(LOGGER, "Loading models from gazebo to moveit planning scene");

    ignition::transport::Node transport_node;
    ignition::msgs::Scene response;
    ignition::msgs::Empty request;
    bool result;
    const std::string service_name = "/world/" + this->world_name + "/scene/info";
    const unsigned int timeout = 5000;
    bool success = transport_node.Request(service_name, request, timeout, response, result);

    if (!success || !result)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to call the service " << service_name);
        throw std::runtime_error("Failed to call the service" + service_name + " stopping execution.");
    }

    const auto visual_pose_in_world_frame = [](const ignition::msgs::Model& model, const ignition::msgs::Link& link, const ignition::msgs::Visual& visual)
    {
        const auto geometry_from_ign = [](const ignition::msgs::Pose& ign_pose)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = ign_pose.position().x();
            pose.position.y = ign_pose.position().y();
            pose.position.z = ign_pose.position().z();
            pose.orientation.x = ign_pose.orientation().x();
            pose.orientation.y = ign_pose.orientation().y();
            pose.orientation.z = ign_pose.orientation().z();
            pose.orientation.w = ign_pose.orientation().w();
            
            return pose;
        };
        tf2::Transform model_tf, link_tf, visual_tf;
        tf2::fromMsg(geometry_from_ign(model.pose()), model_tf); //pose, tf
        tf2::fromMsg(geometry_from_ign(link.pose()), link_tf); //pose, tf
        tf2::fromMsg(geometry_from_ign(visual.pose()), visual_tf); //pose, tf
        tf2::Transform world_to_visual = model_tf * link_tf * visual_tf;
        
        geometry_msgs::msg::Pose result_pose;
        const auto &tmp = tf2::toMsg(world_to_visual);

        result_pose.position.x = tmp.translation.x;
        result_pose.position.y = tmp.translation.y;
        result_pose.position.z = tmp.translation.z;
        result_pose.orientation = tmp.rotation;
        return result_pose;
    };
    
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    for (int i = 0; i < response.model_size(); i++) //loop through models
    {
        const auto &model = response.model(i); 
        if (model.name() == this->robot_name || model.name() == "ground_plane") continue; 
        RCLCPP_INFO_STREAM(LOGGER, "model_name: " << model.name()); 
        
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        collision_object.id = model.name() + "_" + std::to_string(i);

        for (int j = 0; j < model.link_size(); j++) //loop through links
        {
            const auto &link = model.link(j);
            RCLCPP_INFO_STREAM(LOGGER, "  link name: " << link.name()); //debug
            
            //loop through link visuals
            for (int k = 0; k < link.visual_size(); k++)
            {
                const auto &visual = link.visual(k);
                const auto &type = visual.geometry().type();
                if (type == ignition::msgs::Geometry_Type::Geometry_Type_MESH) {
                    const std::string mesh_url = this->parse_filename_to_url(visual.geometry().mesh().filename());
                    RCLCPP_INFO_STREAM(LOGGER, "    mesh visual filename: " << mesh_url);
                    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_url);
                    if (!mesh) { 
                        RCLCPP_ERROR_STREAM(LOGGER, "Failed to load mesh from URL: " << mesh_url);
                        throw std::runtime_error("Failed to load mesh from URL: " + mesh_url);
                    }
                    
                    // Convert the shapes::Mesh to shape_msgs::Mesh
                    shape_msgs::msg::Mesh mesh_msg;
                    shapes::ShapeMsg mesh_msg_wrapper;
                    shapes::constructMsgFromShape(mesh, mesh_msg_wrapper);
                    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_wrapper);

                    collision_object.meshes.push_back(mesh_msg);
                    collision_object.mesh_poses.push_back(visual_pose_in_world_frame(model, link, visual));

                    delete mesh;
                } else { //common types
                    shape_msgs::msg::SolidPrimitive::_type_type coll_obj_type;
                    shape_msgs::msg::SolidPrimitive::_dimensions_type coll_obj_dimensions;
                    switch (type)
                    {
                        case ignition::msgs::Geometry_Type::Geometry_Type_BOX:
                            coll_obj_type = shape_msgs::msg::SolidPrimitive::BOX;
                            coll_obj_dimensions = {visual.geometry().box().size().x(), visual.geometry().box().size().y(), visual.geometry().box().size().z()};
                            RCLCPP_INFO(LOGGER, "    collision object primitives type: box");
                            break;
                        case ignition::msgs::Geometry_Type::Geometry_Type_SPHERE:
                            coll_obj_type = shape_msgs::msg::SolidPrimitive::SPHERE;
                            coll_obj_dimensions = {visual.geometry().sphere().radius()};
                            RCLCPP_INFO(LOGGER, "    collision object primitives type: sphere");
                            break;
                        case ignition::msgs::Geometry_Type::Geometry_Type_CYLINDER:
                            coll_obj_type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                            coll_obj_dimensions.resize(2);
                            coll_obj_dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = visual.geometry().cylinder().radius();
                            coll_obj_dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = visual.geometry().cylinder().length();
                            RCLCPP_INFO(LOGGER, "    collision object primitives type: cylinder");
                            break;
                        default:
                            RCLCPP_ERROR(LOGGER, "not implemented for ignition::msgs::Geometry_Type::%d", type);
                            throw std::runtime_error("not implemented for ignition::msgs::Geometry_Type::" + std::to_string(type));
                    }
                    
                    shape_msgs::msg::SolidPrimitive primitive;
                    primitive.type = coll_obj_type;
                    primitive.dimensions = coll_obj_dimensions;
                    collision_object.primitives.push_back(primitive);
                    collision_object.primitive_poses.push_back(visual_pose_in_world_frame(model, link, visual));
                }
            }
        }

        collision_objects.push_back(collision_object); //one model is one collision object with all its meshes and primitives
    }

    moveit::planning_interface::PlanningSceneInterface psi;
    for (const auto& coll_obj : collision_objects)
    {
        psi.applyCollisionObject(coll_obj);
        RCLCPP_INFO_STREAM(LOGGER, "applied collision obstacle: " << coll_obj.id);
    }
    RCLCPP_INFO_STREAM(LOGGER, "applied collision obstacles");

}
