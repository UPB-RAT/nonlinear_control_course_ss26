#ifndef GZ_SIM_SYSTEMS_ROBOTPOSEPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_ROBOTPOSEPUBLISHER_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class RobotPosePublisherPrivate;

  /// \brief Pose publisher system. Attach to an entity to publish the
  /// transform of its child entities in the form of ignition::msgs::Pose
  /// messages, or a single ignition::msgs::Pose_V message if
  /// "use_pose_vector_msg" is true.
  ///
  /// ## System Parameters
  ///
  /// - `<publish_model_pose>`: Set to true to publish model pose.
  /// - `<use_pose_vector_msg>`: Set to true to publish a ignition::msgs::Pose_V
  ///   message instead of multiple ignition::msgs::Pose messages.
  /// - `<update_frequency>`: Frequency of pose publications in Hz. A negative
  ///   frequency publishes as fast as possible (i.e, at the rate of the
  ///   simulation step)
  class RobotPosePublisher
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    public: RobotPosePublisher();
    public: ~RobotPosePublisher() override = default;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    private: std::unique_ptr<RobotPosePublisherPrivate> dataPtr;
  };
}
}
}
}

#endif