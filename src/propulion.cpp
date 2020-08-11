#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/Twist.h>
#include <ignition/math/Vector3.hh>
#include <thread>

namespace gazebo {
class Propulsion : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    if (_sdf->HasElement("propulsionTopicName")) {
      this->propulsionTopicName = _sdf->Get<std::string>("propulsionTopicName");
    } else {
      ROS_WARN("No Topic Given name given, setting default name %s",
               this->propulsionTopicName.c_str());
    }

    if (_sdf->HasElement("NameLinkToApplyPropulsion")) {
      this->NameLinkToApplyPropulsion =
          _sdf->Get<std::string>("NameLinkToApplyPropulsion");
    } else {
      ROS_WARN("No NameLinkToApplyPropulsion Given name given, setting default "
               "name %s",
               this->NameLinkToApplyPropulsion.c_str());
    }

    if (_sdf->HasElement("rate")) {
      this->rate = _sdf->Get<double>("rate");
    } else {
      ROS_WARN("No rate Given name given, setting default "
               "name %f",
               this->rate);
    }

    // Store the pointer to the model
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->link_to_apply_resitance =
        this->model->GetLink(this->NameLinkToApplyPropulsion);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Propulsion::OnUpdate, this));

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "model_mas_controler_rosnode",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("model_mas_controler_rosnode"));

#if (GAZEBO_MAJOR_VERSION >= 8)
    this->last_time = this->world->SimTime().Float();
#else
    this->last_time = this->world->GetSimTime().Float();
#endif

    // Freq
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            this->propulsionTopicName, 1,
            boost::bind(&Propulsion::OnRosMsg, this, _1), ros::VoidPtr(),
            &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&Propulsion::QueueThread, this));

    ROS_WARN("Loaded Propulsion Plugin with parent...%s, Started ",
             this->model->GetName().c_str());
  }

  // Called by the world update start event
public:
  void OnUpdate() {
    // Get simulator time
#if (GAZEBO_MAJOR_VERSION >= 8)
    float current_time = this->world->SimTime().Float();
#else
    float current_time = this->world->GetSimTime().Float();
#endif
    float dt = current_time - this->last_time;
    this->last_time = current_time;
    float period = 1.0 / this->rate;
    if (dt <= period)
      return;

    ROS_WARN("Update Tick...");
  }

public:
  void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg) {

    // TODO: Add multivector force apply
    this->force.x = _msg->linear.x;
    this->force.y = _msg->linear.y;
    this->force.z = _msg->linear.z;

    this->torque.x = _msg->angular.x;
    this->torque.y = _msg->angular.y;
    this->torque.z = _msg->angular.z;

    this->link_to_apply_resitance->AddRelativeForce(this->force);
#if (GAZEBO_MAJOR_VERSION >= 8)
    this->link_to_apply_resitance->AddRelativeTorque(this->torque);
#else
    this->link_to_apply_resitance->AddRelativeTorque(this->torque);
#endif
    ROS_WARN("Force = [%f,%f,%f] ",this->force.x, this->force.y, this->force.z);
  }

  /// \brief ROS helper function that processes messages
private:
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;

  /// \brief A ROS subscriber
private:
  physics::LinkPtr link_to_apply_resitance;

private:
  std::string propulsionTopicName = "fluid_resitance";

private:
  std::string NameLinkToApplyPropulsion = "base_link";

private:
  double rate = 1.0;

private:
  float last_time = 0.0;

private:
  /// \brief The parent World
  physics::WorldPtr world;

private:
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d force, torque;
#else
    math::Vector3 force, torque;
#endif

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Propulsion)
} // namespace gazebo
