#include "SetPosePlugin.hh"
using namespace gz;
using namespace sim;
using namespace systems;

SetPosePlugin::SetPosePlugin() = default;
SetPosePlugin::~SetPosePlugin() = default;

void SetPosePlugin::Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
EntityComponentManager &ecm, EventManager & /*eventMgr*/)
{
  this->model = Model(entity);
  if (!this->model.Valid(ecm))
  {
    gzerr << "SetPosePlugin debe estar asociado a un modelo válido." << std::endl;
    return;
  }
  if (sdf->HasElement("topic"))
  {
    this->topic = sdf->Get<std::string>("topic");
  }
  else
  {
    gzerr << "SetPosePlugin requiere el elemento <topic>." << std::endl;
    return;
  }
  this->node = std::make_unique<gz::transport::Node>();
  if (!this->node->Subscribe(this->topic, &SetPosePlugin::OnPoseMsg, this))
  {
    gzerr << "Error al suscribirse al tópico [" << this->topic << "]." << std::endl;
    return;
  }
  gzmsg << "SetPosePlugin suscrito al tópico [" << this->topic << "]" << std::endl;
}

void SetPosePlugin::PreUpdate(const UpdateInfo & /*info*/, EntityComponentManager &ecm)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->newPose)
  {
    this->model.SetWorldPoseCmd(ecm, *this->newPose);
    gzmsg << "Posición actualizada a: " << this->newPose->Pos() << std::endl;
    this->newPose.reset();
  }
}

void SetPosePlugin::OnPoseMsg(const gz::msgs::Pose &msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->newPose = gz::math::Pose3d(
    gz::math::Vector3d(msg.position().x(), msg.position().y(), msg.position().z()),
    gz::math::Quaterniond(msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z())
  );
  gzmsg << "Mensaje recibido: posición (" << msg.position().x() << ", "
        << msg.position().y() << ", " << msg.position().z() << ")" << std::endl;
}

GZ_ADD_PLUGIN(gz::sim::systems::SetPosePlugin, gz::sim::System,
              gz::sim::systems::SetPosePlugin::ISystemConfigure,
              gz::sim::systems::SetPosePlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::SetPosePlugin, "gz::sim::systems::SetPosePlugin")
