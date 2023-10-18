/*
This node implements a service to add or remove objects to the planning scene or to fully
clear it. This service request is used to deal with the planning scene for motion planning
*/


// STL
#include <string>

// ROS Library
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/SolidPrimitive.h>

// User-defined service
#include "assignment_2/PlanScene_AddObj.h"
#include "assignment_2/PlanScene_ClearScene.h"
#include "assignment_2/PlanScene_RemoveObj.h"
#include "assignment_2/Transform.h"


class PlanningScene{

    public:

    PlanningScene(){
        add_srv = nh.advertiseService("/add_collision_obj", &PlanningScene::addCollisionObject, this);
        remove_srv = nh.advertiseService("/remove_collision_obj", &PlanningScene::removeCollisionObject, this);
        clear_srv = nh.advertiseService("/clear_scene", &PlanningScene::clearScene, this);
        transform_service = nh.serviceClient<assignment_2::Transform>("/transform_poses");
    }

    private:

    bool addCollisionObject(assignment_2::PlanScene_AddObj::Request &req, assignment_2::PlanScene_AddObj::Response &res){
        // Retrieve the request data
        std::string id = std::to_string(req.id);
        geometry_msgs::PoseStamped pose_stamped = req.pose;
        shape_msgs::SolidPrimitive shape = req.shape;

        // If needed, transform the pose into frame FRAME_ID
        if(pose_stamped.header.frame_id != FRAME_ID){
            assignment_2::Transform tf_srv;
            tf_srv.request.input.push_back(pose_stamped);
            tf_srv.request.target_frame;
            transform_service.call(tf_srv);
            pose_stamped = tf_srv.response.output[0];
        }

        geometry_msgs::Pose pose = pose_stamped.pose;
        
        // Define the Collisgion Object
        moveit_msgs::CollisionObject object;
        object.id = id;        
        object.header.frame_id = FRAME_ID;
        object.primitive_poses.push_back(pose);
        object.primitives.push_back(shape);
        
        // Clear the Planning scene
        planning_scene_interface.removeCollisionObjects(ids);
        
        // Add the Collision Object
        ids.push_back(id);
        collision_objects.push_back(object);

        // Recreate the Planning Scene
        planning_scene_interface.addCollisionObjects(collision_objects);

        res.success = true;

        ROS_INFO("Added object id[%s] as the collision objects to the planning scene.", id.c_str());

        return true;
    }
    
    bool removeCollisionObject(assignment_2::PlanScene_RemoveObj::Request &req, assignment_2::PlanScene_RemoveObj::Response &res){
        // Retrieve the request data
        std::string id = std::to_string(req.id);
        
        // Look for the requested object
        int index = 0;
        while(index < ids.size()){
            if(id.compare(ids[index]) == 0){
                break;
            }
            index++;
        }
        
        // Return if the object hasn't been found
        if(index  == ids.size()){
            ROS_INFO("Object id[%s] not found.", id.c_str());
            return false;
        }

        // Clear the Planning Scene
        planning_scene_interface.removeCollisionObjects(ids);

        // Set the header of the response pose
        res.pose.header = collision_objects[0].header;

        // Retrieve the object to delete
        geometry_msgs::Pose pose = collision_objects[0].primitive_poses[0];
        shape_msgs::SolidPrimitive shape = collision_objects[0].primitives[0];

        // Delete the object from the list of collision objects
        ids.erase(ids.begin() + index);
        collision_objects.erase(collision_objects.begin() + index);

        // Recreate the planning scene
        planning_scene_interface.addCollisionObjects(collision_objects);

        // Send the response
        res.pose.pose = pose;
        res.shape = shape;

        ROS_INFO("Removed object id[%s] from the planning scene.", id.c_str());
        
        return true;
    }

    bool clearScene(assignment_2::PlanScene_ClearScene::Request &req, assignment_2::PlanScene_ClearScene::Response &res){
        // Get the ids to send them as response
        int count = 0;
        for(int i = 0; i  < ids.size(); i++){
            res.ids.push_back(ids[i]);
            moveit_msgs::CollisionObject obj = collision_objects[i];
            res.pose = obj.primitive_poses;
            res.shape = obj.primitives;
            count++;
        }

        // Clear the scene
        planning_scene_interface.removeCollisionObjects(ids);
        
        // Clear the collision objects
        for(;count > 0; count--){
            ids.pop_back();
            collision_objects.pop_back();
        }

        // Update the planning scene
        planning_scene_interface.addCollisionObjects(collision_objects);

        ROS_INFO("Cleared the planning scene");

        return true;
    }
    
    ros::NodeHandle nh;    
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::vector<std::string> ids;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    ros::ServiceServer add_srv, remove_srv, clear_srv;
    ros::ServiceClient transform_service;

    const std::string FRAME_ID = "base_footprint";

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_scene");

    PlanningScene scene;
    ROS_INFO("Planning Scene Service READY");
    ros::spin();

    return 0;
}