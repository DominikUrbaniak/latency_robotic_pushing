#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include <math.h>

#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <custom_interfaces/srv/set_obj_id.hpp>
#include <custom_interfaces/msg/pose_array.hpp>

//#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


using namespace std::chrono_literals;
using namespace std::placeholders;

bool gazebo_model_id_set = false;

class ObjectInfo {
private:
    std::string objPath;
    unsigned int id;
    std::string frame_id;
    std::string name;
    geometry_msgs::msg::Pose pose;
    double r;
    double g;
    double b;
    double scale_x;
    double scale_y;
    double scale_z;

    bool use_embedded_materials;
public:
    inline void setObjPath(std::string p) {objPath=p;}
    inline std::string getObjPath() {return objPath;}
    inline unsigned int getTagID() {return id;}
    inline unsigned int getGazeboModelID() {return id;}
    inline std::string get_frame_id() {return frame_id;}
    inline std::string getname() {return name;}
    inline double getx() {return pose.position.x;}
    inline double gety() {return pose.position.y;}
    inline double getz() {return pose.position.z;}
    inline double getqx() {return pose.orientation.x;}
    inline double getqy() {return pose.orientation.y;}
    inline double getqz() {return pose.orientation.z;}
    inline double getqw() {return pose.orientation.w;}
    inline bool get_use_embeded_materials() {return use_embedded_materials;}
    inline double getr() {return r;}
    inline double getg() {return g;}
    inline double getb() {return b;}
    inline double getscale_x() {return scale_x;}
    inline double getscale_y() {return scale_y;}
    inline double getscale_z() {return scale_z;}

    inline geometry_msgs::msg::Pose getPose() {return pose;}

    inline void setTagID(unsigned int i) {id=i;}
    inline void setGazeboModelID(unsigned int i) {id=i;}
    inline void set_frame_id(std::string s) {frame_id=s;}
    inline void setname(std::string s) {name=s;}
    inline void setPose(geometry_msgs::msg::Pose g) {pose=g;}
    inline void setx(double v) {pose.position.x=v;}
    inline void sety(double v) {pose.position.y=v;}
    inline void setz(double v) {pose.position.z=v;}
    inline void setqx(double v) {pose.orientation.x=v;}
    inline void setqy(double v) {pose.orientation.y=v;}
    inline void setqz(double v) {pose.orientation.z=v;}
    inline void setqw(double v) {pose.orientation.w=v;}
    inline void setr(double v) {r=v;}
    inline void setg(double v) {g=v;}
    inline void setb(double v) {b=v;}
    inline void setscale_x(double v) {scale_x=v;}
    inline void setscale_y(double v) {scale_y=v;}
    inline void setscale_z(double v) {scale_z=v;}

    inline void set_use_embedded_materials(bool t) {use_embedded_materials = t;}
};

std::vector<ObjectInfo> objects;

void SetWorld()
{
    ObjectInfo obj;

    objects.clear();

    obj.setObjPath("package://main_pkg/meshes/cube_tag0_grey.dae");
    obj.setTagID(0);
    obj.setGazeboModelID(0);
    obj.setname("cube_tag0_grey");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.2);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag1_violet.dae");
    obj.setTagID(1);
    obj.setGazeboModelID(1);
    obj.setname("cube_tag1_violet");
    obj.set_frame_id("base_link");
    obj.setx(0.4);
    obj.sety(0.2);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag2_indigo.dae");
    obj.setTagID(2);
    obj.setGazeboModelID(2);
    obj.setname("cube_tag2_indigo");
    obj.set_frame_id("base_link");
    obj.setx(0.3);
    obj.sety(0.2);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag3_blue.dae");
    obj.setTagID(3);
    obj.setGazeboModelID(3);
    obj.setname("cube_tag3_blue");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.1);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag4_green.dae");
    obj.setTagID(4);
    obj.setGazeboModelID(4);
    obj.setname("cube_tag4_green");
    obj.set_frame_id("base_link");
    obj.setx(0.4);
    obj.sety(0.1);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag5_yellow.dae");
    obj.setTagID(5);
    obj.setGazeboModelID(5);
    obj.setname("cube_tag5_yellow");
    obj.set_frame_id("base_link");
    obj.setx(0.3);
    obj.sety(0.1);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag6_orange.dae");
    obj.setTagID(6);
    obj.setGazeboModelID(6);
    obj.setname("cube_tag6_orange");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.0);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag7_red.dae");
    obj.setTagID(7);
    obj.setGazeboModelID(7);
    obj.setname("cube_tag7_red");
    obj.set_frame_id("base_link");
    obj.setx(-0.1);
    obj.sety(0.8);
    obj.setz(0.0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setscale_x(0.025);
    obj.setscale_y(0.025);
    obj.setscale_z(0.025);
    obj.set_use_embedded_materials(true);
    obj.setr(0.0);
    obj.setg(0.0);
    obj.setb(0.9);
    objects.push_back(obj);
}
tf2::Quaternion quaternion_multiply(tf2::Quaternion q0,tf2::Quaternion q1){


   // Extract the values from q0
   double w0 = q0.w();
   double x0 = q0.x();
   double y0 = q0.y();
   double z0 = q0.z();

   // Extract the values from q1
   double w1 = q1.w();
   double x1 = q1.x();
   double y1 = q1.y();
   double z1 = q1.z();

   // Computer the product of the two quaternions, term by term
   tf2::Quaternion final_quaternion;
   final_quaternion[0] = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;
   final_quaternion[1] = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
   final_quaternion[2] = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
   final_quaternion[3] = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;

   //normalize
   final_quaternion.normalize();

   RCLCPP_INFO(rclcpp::get_logger("rclccp"), "multiplied quaternion: qx=%f, qy=%f, qz=%f, qw=%f", final_quaternion.x(),final_quaternion.y(),final_quaternion.z(),final_quaternion.w());

   // Return a normalized quaternion
   return final_quaternion;

}

class WorldSetup : public rclcpp::Node
{
public:
  WorldSetup() : Node("world_setup"), attach_cube_id_(-1), attached_(false)
  {
    SetWorld();
    RCLCPP_INFO(this->get_logger(), "World is setup! Loaded %d object(s)", (int)objects.size());

    // Set initial values for variables
    topic_ = "visualization_marker_array";
    rate_ = 1;

    // Create publisher and subscriber
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_, rate_);
    state_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo_state/model_states", rate_, std::bind(&WorldSetup::update_marker_array, this, _1));

    attach_service_ = this->create_service<custom_interfaces::srv::SetObjId>("/world_setup/attach_obj", std::bind(&WorldSetup::attach_obj, this, _1, _2));
    detach_service_ = this->create_service<custom_interfaces::srv::SetObjId>("/world_setup/detach_obj", std::bind(&WorldSetup::detach_obj, this, _1, _2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create tf buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Create callback groups for timer and client.
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create a timer to send requests to Gazebo.
    timer_ptr_ = this->create_wall_timer(10ms, std::bind(&WorldSetup::timer_callback, this), timer_cb_group_);

    // Create a client to send requests to Gazebo.
    client_ptr_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
      "/gazebo_state/set_entity_state",
      rmw_qos_profile_services_default,
      client_cb_group_);

    // Wait until the Gazebo service is available.
    while (!client_ptr_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
  }

private:
  // Declare member variables
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_ptr_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr state_subscriber_;
  rclcpp::Service<custom_interfaces::srv::SetObjId>::SharedPtr attach_service_;
  rclcpp::Service<custom_interfaces::srv::SetObjId>::SharedPtr detach_service_;
  std::string topic_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;//{nullptr}
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  int rate_;
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::TransformStamped t_cube_eef_;
  tf2::Quaternion quat_;
  int attach_cube_id_;
  bool attached_;

  // Timer callback function for updating the Gazebo world.
  void timer_callback()
  {

    // Check, if an object is attached
    if (attach_cube_id_ >= 10)
    {
      if(!attached_){
        attached_ = true;
        start_pose_ = objects[attach_cube_id_].getPose();
      }
      // Create a request to set an object in Gazebo.
      auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
      request->state.name = objects[attach_cube_id_].getname();
      geometry_msgs::msg::TransformStamped t_cube_eef_old;
      //set fixed pose of cube relative to eef
      t_cube_eef_old = t_cube_eef_;
      tf2::Quaternion quat_old;
      tf2::convert(t_cube_eef_.transform.rotation,quat_old);
      try {
        t_cube_eef_ = tf_buffer_->lookupTransform(
          "base_link", "wrist_3_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          "gripper_right_pad", "v", ex.what());
        return;
      }
      /*try {
        t_cube_eef_ = tf_buffer_->lookupTransform(
          "base_link", "wrist_3_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          "base_link", "wrist_3_link", ex.what());
        return;
      }*/
      tf2::convert(t_cube_eef_.transform.rotation,quat_);
      start_pose_.position.x = start_pose_.position.x + t_cube_eef_.transform.translation.x - t_cube_eef_old.transform.translation.x;
      start_pose_.position.y = start_pose_.position.y + t_cube_eef_.transform.translation.y - t_cube_eef_old.transform.translation.y;
      start_pose_.position.z = start_pose_.position.z + t_cube_eef_.transform.translation.z - t_cube_eef_old.transform.translation.z;
      //invert old orientation to get the relative change between the old and new orientation
      /*quat_old.inverse();
      //tf2::Quaternion quat_diff = quaternion_multiply(quat_,quat_old);
      tf2::Quaternion quat_diff = quat_*quat_old;
      quat_diff.normalize();
      RCLCPP_INFO(this->get_logger(), "quat_diff: qx=%f, qy=%f, qz=%f, qw=%f", quat_diff.x(), quat_diff.y(),quat_diff.z(),quat_diff.w());
      //quat_diff.normalize();
      //tf2::Quaternion quat_new = quaternion_multiply(quat_,quat_diff);
      tf2::Quaternion quat_new = quat_ * quat_diff;
      quat_new.normalize();
      tf2::convert(quat_new, start_pose_.orientation);
      RCLCPP_INFO(this->get_logger(), "quat_new: qx=%f, qy=%f, qz=%f, qw=%f", quat_new.x(), quat_new.y(),quat_new.z(),quat_new.w());
      */request->state.pose = start_pose_;

      // Send the request asynchronously and wait for a response.
      auto result = client_ptr_->async_send_request(request);
      std::future_status status = result.wait_for(10s);  // Timeout to guarantee a graceful finish
      if (status == std::future_status::ready)
      {
        //RCLCPP_INFO(this->get_logger(), "Received response");
        RCLCPP_INFO(this->get_logger(), "attached: %d",attach_cube_id_);
      }
    }
    else{
      attached_ = false;
    }
  }

  //! Attach object to gripper
  void attach_obj2(std::shared_ptr<custom_interfaces::srv::SetObjId::Request>  req,
           std::shared_ptr<custom_interfaces::srv::SetObjId::Response> res)
  {

    int id = req->objid;


    tf2::Transform world2obj;
    //world2obj.setOrigin(tf2::Vector3(objects[id].getx(), objects[id].gety(), objects[id].getz()));
    //world2obj.setRotation(tf2::Vector3(objects[id].getqx(), objects[id].getqy(), objects[id].getqz(), objects[id].getqw()));
    tf2::fromMsg(objects[id].getPose(), world2obj);


    //transform from chessboard to robot gripper pad
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string rightpadframe = "wrist_3_link";//"gripper_right_pad";


    //rclcpp::Time now = this->get_clock()->now();
    //rclcpp::Time now = tf2::toMsg(tf2::TimePoint());
    //clcpp::Time now = this->now();
    //std::cout << "Current time: " << now.seconds << " sec and " << now.nanoseconds << " nanosecs\n";
    //int nanosecs = now.nanoseconds;
    //double secs = tf2_ros::timeToSec(now);

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attach obj %d to right gripper pad at time: %f seconds", id, secs);
    try {
      transformStamped = tf_buffer_->lookupTransform(
        "base_link", rightpadframe,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not transform %s to %s: %s",
        "base_link", rightpadframe.c_str(), ex.what());//wrist_3_link
      return;
    }

     //tf2::TimePointZero

    tf2::Stamped< tf2::Transform >  world2pad;
    tf2::fromMsg(transformStamped, world2pad);
    //std::cout<<"chess2pad!!!!!!! = "<<chess2pad.frame_id_<<std::endl;

    //transform from robot gripper pad to object
    tf2::Transform pad2world = world2pad.inverse();
    tf2::Transform pad2obj = pad2world * world2obj;

    //set the transform from right pad frame to oject
    objects[id].set_frame_id("base_link");
    objects[id].setx(pad2obj.getOrigin().getX());
    objects[id].sety(pad2obj.getOrigin().getY());
    objects[id].setz(pad2obj.getOrigin().getZ());
    objects[id].setqx(pad2obj.getRotation().getX());
    objects[id].setqy(pad2obj.getRotation().getY());
    objects[id].setqz(pad2obj.getRotation().getZ());
    objects[id].setqw(pad2obj.getRotation().getW());

    //sets global flag on attached_ obstactes
    attached_ = id;
    res->success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "attached_!");

  }

  //! Detach object to gripper
  void detach_obj2(std::shared_ptr<custom_interfaces::srv::SetObjId::Request>  req,
           std::shared_ptr<custom_interfaces::srv::SetObjId::Response> res)
  {
    int id = req->objid;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detach obj %d to right gripper pad", id);

    tf2::Transform pad2obj;
    tf2::fromMsg(objects[id].getPose(), pad2obj);


    //transform from chessboard to robot gripper pad
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string rightpadframe = "gripper_right_pad";


    rclcpp::Time now = this->get_clock()->now();
    transformStamped = tf_buffer_->lookupTransform(
     "world",
     rightpadframe,
     now,
     50ms);

    tf2::Stamped< tf2::Transform >  world2pad;
    tf2::fromMsg(transformStamped, world2pad);
    //std::cout<<"chess2pad!!!!!!! = "<<chess2pad.frame_id_<<std::endl;

    //transform from robot gripper pad to object
    tf2::Transform world2obj = world2pad * pad2obj;

    //set the transform from right pad frame to oject
    objects[id].set_frame_id("base_link");
    objects[id].setx(world2obj.getOrigin().getX());
    objects[id].sety(world2obj.getOrigin().getY());
    objects[id].setz(world2obj.getOrigin().getZ());
    objects[id].setqx(world2obj.getRotation().getX());
    objects[id].setqy(world2obj.getRotation().getY());
    objects[id].setqz(world2obj.getRotation().getZ());
    objects[id].setqw(world2obj.getRotation().getW());

    //sets global flag on attached_ obstactes
    attached_ = -1;
    res->success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detached!");

  }
  //! Attach object to gripper
  void attach_obj(std::shared_ptr<custom_interfaces::srv::SetObjId::Request>  req,
           std::shared_ptr<custom_interfaces::srv::SetObjId::Response> res)
  {
    //sets global flag on attached_ obstactes
    attach_cube_id_ = req->objid;
    if(attach_cube_id_>=0){
      //set the initial relative pose of the cube to the eef
      objects[attach_cube_id_].set_frame_id("gripper_right_pad");
      res->success = true;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cube %d is attached to the eef!", attach_cube_id_);
    }
    else{
      res->success = true;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cube is detached from the eef!");
    }

  }
  //! Detach object from gripper
  void detach_obj(std::shared_ptr<custom_interfaces::srv::SetObjId::Request>  req,
           std::shared_ptr<custom_interfaces::srv::SetObjId::Response> res)
  {
    //sets global flag on attached_ obstactes
    attach_cube_id_ = -1;
    objects[req->objid].set_frame_id("base_link");
    res->success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cube %d is detached from the eef: %d", req->objid, attach_cube_id_);

  }

  // Callback function for state subscriber
  void update_marker_array(gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    // If gazebo model IDs have not been set yet, set them
    if (!gazebo_model_id_set)
    {
      std::vector<std::string> names = msg->name;


      for (int i = 0; i < (int)names.size(); i++)
      {
        for (int j = 0; j < (int)objects.size(); j++)
        {
          if (objects[j].getname().compare(names[i]) == 0)
          {
            objects[j].setGazeboModelID(i);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "name: %s, id: %d!", names[i].c_str(), i);
          }
        }
      }

      gazebo_model_id_set = true;
    }

    // Update markers for each object
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray markers;

    for (int i = 0; i < (int)objects.size(); i++) {
      // Set the pose of the current object
      objects[i].setPose(msg->pose[objects[i].getGazeboModelID()]);

      // Create a new marker for the current object
      marker.header.frame_id = objects[i].get_frame_id();
      marker.header.stamp = this->get_clock()->now();
      marker.ns = objects[i].getname();//"cubes";
      marker.id = objects[i].getTagID();
      marker.mesh_resource = objects[i].getObjPath();
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = objects[i].getPose();
      marker.scale.x = objects[i].getscale_x();
      marker.scale.y = objects[i].getscale_y();
      marker.scale.z = objects[i].getscale_z();
      marker.color.a = 1.0;
      marker.color.r = objects[i].getr();
      marker.color.g = objects[i].getg();
      marker.color.b = objects[i].getb();
      marker.mesh_use_embedded_materials = objects[i].get_use_embeded_materials();

      // Add the marker to the list of markers
      markers.markers.push_back(marker);

      geometry_msgs::msg::TransformStamped t;

      // Read message content and assign it to
      // corresponding tf variables
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = objects[i].get_frame_id();
      t.child_frame_id = objects[i].getname();

      t.transform.translation.x = objects[i].getx();
      t.transform.translation.y = objects[i].gety();
      t.transform.translation.z = objects[i].getz();

      //tf2::Quaternion q;
      //q.setRPY(0, 0, msg->theta);
      t.transform.rotation.x = objects[i].getqx();
      t.transform.rotation.y = objects[i].getqy();
      t.transform.rotation.z = objects[i].getqz();
      t.transform.rotation.w = objects[i].getqw();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    // Publish the markers
    publisher_->publish(markers);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorldSetup>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  //rclcpp::spin(std::make_shared<WorldSetup>());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
