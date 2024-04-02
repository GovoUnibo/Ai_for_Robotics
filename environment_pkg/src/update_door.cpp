#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "environment_pkg/DoorOpen.h"
#include "environment_pkg/DoorStatus.h"

using namespace std;



 

namespace gazebo
{
  class BoxUpdate : public ModelPlugin{
    public: 
        BoxUpdate() : ModelPlugin()
        , rate(10)
        {
          
        }

        ~BoxUpdate()
        {
          ros::shutdown();
          // print in red
          cout << "\033[1;31m [PLUGIN] SHUTDOWN \033[0m" << endl;
        }

        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
          // Store the pointer to the model
          this->model = _parent;
          // print in green "Box PLUGIN STARTED"
        //   this->model->SetStatic(true);
        

          
          this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              std::bind(&BoxUpdate::OnUpdate, this));

          if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "Box Plugin", ros::init_options::NoSigintHandler);
            
          }

          this->service_name = this->model->GetName() + "/update_pos";
          string service_status_name = this->model->GetName() + "/status";
          // print in yellow
          cout << "\033[1;32m[Plugin] " << this->model->GetName()<<  " has started: \033[0m"  << endl;

          service_update_door = n.advertiseService(service_name , &BoxUpdate::callbackUpdatePos, this);
          service_door_status = n.advertiseService(service_status_name , &BoxUpdate::callbackDoorStatus, this);
          
            ignition::math::Pose3d pose = this->model->WorldPose();
            this->initial_pos_x = pose.Pos().X();
            this->initial_pos_y = pose.Pos().Y();
            this->initial_pos_z = pose.Pos().Z();
            this->initial_ang_x = pose.Rot().Euler().X();
            this->initial_ang_y = pose.Rot().Euler().Y();
            this->initial_ang_z = pose.Rot().Euler().Z();
            this->open_quantity = 1.7;

        }

        void OnUpdate(){
            if (this->model->GetName() == "Door3")
                this->model->SetRelativePose(ignition::math::Pose3d(this->initial_pos_x - this->open_quantity - 0.55 , this->initial_pos_y  , this->initial_pos_z, this->initial_ang_x , this->initial_ang_y , this->initial_ang_z ));
            else
                this->model->SetRelativePose(ignition::math::Pose3d(this->initial_pos_x , this->initial_pos_y +this->open_quantity , this->initial_pos_z , this->initial_ang_x , this->initial_ang_y , this->initial_ang_z));
        }

    // Pointer to the model
    private: 
        physics::ModelPtr model;
        string service_name;
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        ros::NodeHandle n;
        ros::ServiceServer service_update_door, service_door_status;
        float initial_pos_x , initial_pos_y , initial_pos_z , initial_ang_x , initial_ang_y , initial_ang_z = 0;
        bool is_open = false;
        float open_quantity=0;
        ros::Rate rate;

        bool callbackUpdatePos(environment_pkg::DoorOpen::Request &req, environment_pkg::DoorOpen::Response &res)
        {
            cout << "Requested " << this-> service_name << " to open" << req.open << endl;
            this->is_open = req.open;
            if (req.open)
                this->open_quantity = 1.7;
            else
                this->open_quantity = 0;
            res.done = true;
            cout << "Open "<< this-> service_name << " of Quantity" << this->open_quantity << endl;

            return true;
        }

        bool callbackDoorStatus(environment_pkg::DoorStatus::Request &req, environment_pkg::DoorStatus::Response &res)
        {
            cout << "Requested Status " << this-> service_name << " is " << this->is_open << endl;
            res.is_open = this->is_open;
            return true;
        }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(BoxUpdate)
}