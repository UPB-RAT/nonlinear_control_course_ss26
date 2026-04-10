#include "SetPosePlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Constructor por defecto
SetPosePlugin::SetPosePlugin() = default;

// Destructor por defecto
SetPosePlugin::~SetPosePlugin() = default;

// Método Configure: inicializa el plugin
void SetPosePlugin::Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                              EntityComponentManager &ecm, EventManager & /*eventMgr*/)
{
  // Inicializa el modelo con la entidad proporcionada
  this->model = Model(entity);
  if (!this->model.Valid(ecm))
  {
    ignerr << "SetPosePlugin debe estar asociado a un modelo válido." << std::endl;
    return;
  }

  // Verifica si el elemento <topic> está presente en el SDF
  if (sdf->HasElement("topic"))
  {
    this->topic = sdf->Get<std::string>("topic");
  }
  else
  {
    ignerr << "SetPosePlugin requiere el elemento <topic>." << std::endl;
    return;
  }

  // Crea un nuevo nodo de transporte
  this->node = std::make_unique<ignition::transport::Node>();

  // Suscribe el nodo al tópico para recibir mensajes de pose
  if (!this->node->Subscribe(this->topic, &SetPosePlugin::OnPoseMsg, this))
  {
    ignerr << "Error al suscribirse al tópico [" << this->topic << "]." << std::endl;
    return;
  }

  ignmsg << "SetPosePlugin suscrito al tópico [" << this->topic << "]" << std::endl;
}

// Método PreUpdate: actualiza la pose del modelo antes de cada paso de simulación
void SetPosePlugin::PreUpdate(const UpdateInfo & /*info*/, EntityComponentManager &ecm)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->newPose)
  {
    // Establece la nueva pose para el modelo
    this->model.SetWorldPoseCmd(ecm, *this->newPose);
    ignmsg << "Posición actualizada a: " << this->newPose->Pos() << std::endl;
    this->newPose.reset();
  }
}

// Método OnPoseMsg: callback para manejar mensajes de pose recibidos
void SetPosePlugin::OnPoseMsg(const ignition::msgs::Pose &msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  // Convierte el mensaje a un objeto Pose3d
  this->newPose = ignition::math::Pose3d(
    ignition::math::Vector3d(msg.position().x(), msg.position().y(), msg.position().z()),
    ignition::math::Quaterniond(msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z())
  );
  ignmsg << "Mensaje recibido: posición (" << msg.position().x() << ", "
        << msg.position().y() << ", " << msg.position().z() << ")" << std::endl;
}

// Registra el plugin en Gazebo
IGNITION_ADD_PLUGIN(SetPosePlugin, System, ISystemConfigure, ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(SetPosePlugin, "ignition::gazebo::systems::SetPosePlugin")