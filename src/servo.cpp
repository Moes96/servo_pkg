#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <boost/algorithm/string/replace.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sstream>


namespace igm = ignition::math;
namespace gzp = gazebo::physics;
namespace gzt = gazebo::transport;

namespace gazebo
{
  class ModelServo : public ModelPlugin
  {
    // aggiunto per comunicare angle
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    // fine aggiunta


    private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    gazebo::transport::NodePtr node;
    //gzt::node snode;
    std::string topic;

    public:
    double                            Torque;
    igm::Angle                        Reference;
    gazebo::transport::SubscriberPtr  sub;
    gazebo::transport::PublisherPtr   pub_main;
    gazebo::msgs::Any                 Info;
    double                            ltime;
    bool                              change = false;

    public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelServo::OnUpdate, this, _1));

      node = gzt::NodePtr(new gzt::Node());
      node->Init();
      // std::cout << "Servo Node Initialized " << std::endl;

      //Main Servo Topic
      topic = "/gazebo/servos/"+ sdf->GetAttribute("name")->GetAsString();

      //Publish info and alive Topic
      pub_main = node->Advertise<gazebo::msgs::Any>(topic);
      Info = gazebo::msgs::ConvertAny("Model:AX12A");
      pub_main->Publish(Info);
      // std::cout << "Topic Advertised> "<< topic << std::endl;

      //Torque Reference Topic
      //std::string torque_ref_topic = topic + "/torque_ref";
      //sub = node->Subscribe(torque_ref_topic,&ModelServo::OnTopicReception,this);
      //std::cout << "Subscribed to topic> " << torque_ref_topic << std::endl;


      // Torque = 0.0065;
      Torque = 0.065;
      ltime = 0;


      

      // aggiunto per comunicare angle

      ros::Rate loop_rate(10);

      if (!ros::isInitialized()){
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "servo_pkg", ros::init_options::NoSigintHandler);
      }

      this->rosnode_ = new ros::NodeHandle("servo_pkg");
      this->pub_ = this->rosnode_->advertise<std_msgs::Float64>("servo_angle", 1);

      // fine aggiunta


    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & info)
    {

      double angle = this->model->GetJoint("servo_ax12a::j_Body_Wheel")->Position();
      
      if(angle >= 1.3962222222222223 ){
        change=false;
      }
      if(angle <= -1.3962222222222223){
        change=true;
      }

      if(change)
        this->model->GetJoint("servo_ax12a::j_Body_Wheel")->SetForce(0,Torque);
      else 
        this->model->GetJoint("servo_ax12a::j_Body_Wheel")->SetForce(0,-Torque);

      //std::cerr << angle << std::endl;
      
      // aggiunta per comunicare angle
      std_msgs::Float64 msg;
      msg.data = angle;

      this->pub_.publish(msg);
      ros::spinOnce();
      //fine aggiunta
      

    }

    void OnTopicReception(ConstIntPtr& msg)
    {
      // std::cout << "OnTopicReception>";
      Torque = (double)msg->data();
      Torque /=(100000);
      // std::cout << "Message> Torque set to : " << Torque << std::endl;
    }    

  
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelServo);
}
