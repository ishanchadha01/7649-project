#include "main.hh"

#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
// #include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

void gazebo::SetupArena::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  this->initialize(_parent, _sdf);
  this->setupPhysics(_parent, _sdf);
  this->setupArena(_parent, _sdf);
}

void gazebo::SetupArena::initialize(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  // Create a new transport node
  this->node = transport::NodePtr(new transport::Node());
  // Initialize the node with the world name
  this->node->Init(_parent->Name());

  // Create a publisher on the ~/physics topic
  this->physicsPub = node->Advertise<msgs::Physics>("~/physics");
  // Create a publisher on the ~/factory topic
  this->factoryPub = node->Advertise<msgs::Factory>("~/factory");
}

void gazebo::SetupArena::setupPhysics(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  // Create the message
  msgs::Physics physicsMsg;
  physicsMsg.set_type(msgs::Physics::ODE);

  // Set the step time
  physicsMsg.set_max_step_size(0.01);

  // Change gravity
  msgs::Set(physicsMsg.mutable_gravity(),
      ignition::math::Vector3d(0, 0, 0));
  this->physicsPub->Publish(physicsMsg);
}

void gazebo::SetupArena::setupArena(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  // Model file to load
  this->factoryMsg.set_sdf_filename("model://arena");

  // Pose to initialize the model to
  msgs::Set(this->factoryMsg.mutable_pose(),
      ignition::math::Pose3d(
        ignition::math::Vector3d(-5, -5, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  // Send the message
  this->publishAndClearFactoryMsg();

  // Model file to load
  this->factoryMsg.set_sdf_filename("model://box");
  this->factoryMsg.mutable_pose()->mutable_position()->set_x(0.5);
  this->factoryMsg.mutable_pose()->mutable_position()->set_y(0.5);
  this->factoryMsg.mutable_pose()->mutable_position()->set_z(5);
  this->publishAndClearFactoryMsg();
    
  double minX = 0.5;
  double maxX = 89.5;
  double minY = 0.5;
  double maxY = 89.5;
  for (int i = 0; i < 10; i++) {
    // Model file to load
    this->factoryMsg.set_sdf_filename("model://box");
    this->factoryMsg.set_allow_renaming(true);


    double x = minX + (maxX - minX) * rand() / (RAND_MAX + 1.0);
    double y = minY + (maxY - minY) * rand() / (RAND_MAX + 1.0);
    double z = 5;

    printf("%f %f\n", x, y);

    // Pose to initialize the model to
    msgs::Set(this->factoryMsg.mutable_pose(),
        ignition::math::Pose3d(
          ignition::math::Vector3d(x, y, z),
          ignition::math::Quaterniond(0, 0, 0)));

    // Send the message
    this->publishAndClearFactoryMsg();
  }

}

void gazebo::SetupArena::publishAndClearFactoryMsg(void) {
  // Send the message
  this->factoryPub->Publish(this->factoryMsg);
  // Clear the message
  this->factoryMsg.Clear();
}