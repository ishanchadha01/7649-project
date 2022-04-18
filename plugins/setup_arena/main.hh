#ifndef __PLUGINS__WORLD_EDIT__MAIN_HH
#define __PLUGINS__WORLD_EDIT__MAIN_HH

#include <sdf/sdf.hh>
// #include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
// #include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
// #include "gazebo/transport/transport.hh"

/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo {
  class SetupArena : public WorldPlugin {
   public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
   private:
    // functions
    void initialize(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    void setupModels(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    void setupPhysics(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    void setupArena(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    // transport
    void publishAndClearFactoryMsg(void);

    // members
    transport::NodePtr node;
    transport::PublisherPtr physicsPub;
    transport::PublisherPtr factoryPub;
    msgs::Factory factoryMsg;
    // sdfs/models
    sdf::SDF templateObstacleSdf;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(SetupArena)
}

#endif