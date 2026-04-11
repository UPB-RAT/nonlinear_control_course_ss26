#ifndef GZ_SIM_SYSTEMS_SETPOSEPLUGIN_HH_
#define GZ_SIM_SYSTEMS_SETPOSEPLUGIN_HH_
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/common/Console.hh>
#include <mutex>
#include <optional>

namespace gz
{
namespace sim
{
namespace systems
{
class SetPosePlugin : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  SetPosePlugin();
  virtual ~SetPosePlugin();

  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm, EventManager &eventMgr) override;

  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override;

private:
  void OnPoseMsg(const gz::msgs::Pose &msg);

  Model model;
  std::string topic;
  std::unique_ptr<gz::transport::Node> node;
  std::optional<gz::math::Pose3d> newPose;
  std::mutex mutex;
};
} // namespace systems
} // namespace sim
} // namespace gz
#endif // GZ_SIM_SYSTEMS_SETPOSEPLUGIN_HH_
