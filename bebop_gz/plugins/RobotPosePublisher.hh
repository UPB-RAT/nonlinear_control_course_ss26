#ifndef GZ_SIM_SYSTEMS_ROBOTPOSEPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_ROBOTPOSEPUBLISHER_HH_
#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
class RobotPosePublisherPrivate;

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
