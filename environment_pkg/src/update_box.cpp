#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <environment_pkg/BoxPos.h>
#include <environment_pkg/BoxPickUp.h>
#include <environment_pkg/BoxPutDown.h>
#include <environment_pkg/BoxUpdatePos.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
// nav_msgs/Odometry
using namespace Eigen;
using namespace std;

struct RPY_Angle_s {
    double roll, pitch, yaw;

    RPY_Angle_s();
    RPY_Angle_s(double roll, double pitch, double yaw);
    void operator=(const RPY_Angle_s& e_angles);

    ~RPY_Angle_s();
};
RPY_Angle_s::RPY_Angle_s()
:roll(0)
,pitch(0)
,yaw(0)
{}

RPY_Angle_s::RPY_Angle_s(double roll, double pitch, double yaw)
{
  this->roll = roll;
  this->pitch = pitch;
  this->yaw = yaw;
}

RPY_Angle_s::~RPY_Angle_s(){}

void RPY_Angle_s::operator=(const RPY_Angle_s& e_angles)
{
  roll  = e_angles.roll;  
  pitch = e_angles.pitch;  
  yaw   = e_angles.yaw;
}



RPY_Angle_s RPY_To_Quaternion(double x, double y, double z, double w)
{
    RPY_Angle_s angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


Matrix4d Vector_To_HomogeneousMatrix(double x, double y, double z, double psi, double theta, double phi)
{
    Matrix4d T;
    
    T(0,0) = cos(phi)*cos(theta);
    T(0,1) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    T(0,2) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    T(0,3) = x;

    T(1,0) = sin(phi)*cos(theta);
    T(1,1) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
    T(1,2) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
    T(1,3) = y;

    T(2,0) = -sin(theta);
    T(2,1) = cos(theta)*sin(psi);
    T(2,2) = cos(theta)*cos(psi);
    T(2,3) = z;

    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;
    
    return T;
}

VectorXd HomogeneousMatrix_To_Vector(Matrix4d T){
    Eigen::VectorXd vec(6);
    vec(0) = T(0,3);
    vec(1) = T(1,3);
    vec(2) = T(2,3);
    vec(3) = atan2(T(2,1), T(2,2));
    vec(4) = atan2(-T(2,0), sqrt(pow(T(2,1),2) + pow(T(2,2),2)));
    vec(5) = atan2(T(1,0), T(0,0));
    return vec;
}

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
  public:
    ModelPush() : ModelPlugin()
    {
      T_tiago_box_down = Vector_To_HomogeneousMatrix(0.4, 0, 0.6, 0, 0, 0);
      T_tiago_box_up = Vector_To_HomogeneousMatrix(-0.13, 0, 1.5, 0, 0, 0);
      sub_ground_truth = n.subscribe("/ground_truth_odom", 1, &ModelPush::callbackGroundTruth, this);
      // print in green color PLugin started
      cout << "\033[1;32m[Plugin] Plugin Update Box started\033[0m" << endl;

    }


    ~ModelPush()
    {
      // print in red color PLugin stopped
      cout << "\033[1;31m[Plugin] Plugin Update Box stopped\033[0m" << endl;
    }


    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->box_model = _parent;
      this->box_model->SetStatic(false);
      // this->box_model->SetGravityMode(false);
      string box_name = this->box_model->GetName();
      // make a string with upper letter
      for (int i = 0; i < box_name.length(); i++)
        box_name[i] = toupper(box_name[i]);

      string service_pickup_name = box_name + "/pick_up";
      string service_get_position_name = box_name + "/get_position";
      string service_putdown_name = box_name + "/put_down";
      string service_update_position_name = box_name + "/update_position";

      this->service_pickup = n.advertiseService(service_pickup_name, &ModelPush::callbackServicePickUp, this);
      this->service_get_position = n.advertiseService(service_get_position_name, &ModelPush::callbackServiceGetPosition, this);
      this->service_putdown = n.advertiseService(service_putdown_name, &ModelPush::callbackServicePutDown, this);
      this->service_update_position = n.advertiseService(service_update_position_name, &ModelPush::callbackServiceUpdatePosition, this);
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate()
    {}


    void callbackGroundTruth(const nav_msgs::Odometry::ConstPtr &msg)
    {
      this->tiago_x_pos = msg->pose.pose.position.x;
      this->tiago_y_pos = msg->pose.pose.position.y;
      this->tiago_z_pos = msg->pose.pose.position.z;
      
      RPY_Angle_s angles = RPY_To_Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      this->T_tiago = Vector_To_HomogeneousMatrix(this->tiago_x_pos, this->tiago_y_pos, 0, 0, 0, angles.yaw);
      
    }

    bool callbackServiceGetPosition(environment_pkg::BoxPos::Request &req, environment_pkg::BoxPos::Response &res)
    {
      
      if (!req.request)
        return false;
      res.x_pos = this->box_model->WorldPose().Pos().X();
      res.y_pos = this->box_model->WorldPose().Pos().Y();
      res.z_pos = this->box_model->WorldPose().Pos().Z();
      // res.z_rot = this->box_model->WorldPose().Rot().Euler().Z();

      return true;
      
    }


    bool callbackServicePickUp(environment_pkg::BoxPickUp::Request &req, environment_pkg::BoxPickUp::Response &resp)
    {
      if (!req.action)
        resp.success = false;
      ignition::math::Pose3d pose = this->box_model->WorldPose();
      //se la norma di pose lungo x e y è < di 1 allora posso prendere la scatola
      double distance = sqrt(pow(pose.Pos().X() - this->tiago_x_pos, 2) + pow(pose.Pos().Y() - this->tiago_y_pos, 2));
      if  (distance> 0.5){
        cout << "\033[1;31m[Plugin] Box too far from Tiago\033[0m" << endl;
        // cout << distance << endl;
        resp.success = false;
        return false;
      }
      this->T_box = T_tiago * T_tiago_box_up;
      VectorXd vec = HomogeneousMatrix_To_Vector(this->T_box);
      
      this->box_model->SetWorldPose(ignition::math::Pose3d(vec(0), vec(1), vec(2), vec(3), vec(4), vec(5)));
      

      cout << "\033[1;32m[Plugin] Box picked up\033[0m" << endl;
      resp.success = true;
      return true;

    }

    bool callbackServicePutDown(environment_pkg::BoxPutDown::Request &req, environment_pkg::BoxPutDown::Response &resp)
    {
      this->T_box = this->T_tiago * this->T_tiago_box_down;
      VectorXd vec = HomogeneousMatrix_To_Vector(this->T_box);
      cout << "\033[1;32m[Plugin] Box put down in pos x: " << vec(0) << " y: " << vec(1) << " z: " << vec(2) << "\033[0m" << endl;
      this->box_model->SetWorldPose(ignition::math::Pose3d(vec(0), vec(1), vec(2), vec(3), vec(4), vec(5)));
      resp.success = true;
      return true;
    }

    bool callbackServiceUpdatePosition(environment_pkg::BoxUpdatePos::Request &req, environment_pkg::BoxUpdatePos::Response &resp)
    {
      this->box_model->SetWorldPose(ignition::math::Pose3d(req.x_pos, req.y_pos, req.z_pos, 0, 0, 0));
      return true;
    }

    // Pointer to the model
    private: 

        ros::NodeHandle n;
        ros::ServiceServer service_pickup, service_putdown;
        ros::ServiceServer service_get_position, service_update_position;
        physics::ModelPtr box_model;
        ros::Subscriber sub_ground_truth;

    // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        float tiago_x_pos, tiago_y_pos, tiago_z_pos, tiago_rot_yaw;
        Eigen::Matrix4d T_tiago;
        Eigen::Matrix4d T_box, T_tiago_box_down, T_tiago_box_up;
  
  
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

/*servizio update  e ridare la posizione delle scatole*/