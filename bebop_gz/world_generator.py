import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import sys


def main():
    num_drones = 3
    
    if len(sys.argv) > 1:
        try:
            for arg in sys.argv[1:]:
                if arg.startswith("num_drones="):
                    num_drones = int(arg.split("=")[1])
                    break
            else:
                num_drones = int(sys.argv[1])
        except ValueError:
            print("Error: El número de drones debe ser un valor entero")
            print("Uso: python3 generador_bebop.py [num_drones=X]")
            sys.exit(1)
    
    if num_drones < 1:
        print("Error: El número de drones debe ser al menos 1")
        sys.exit(1)
    
    generate_bebop_files(num_drones)


def generate_bebop_files(num_drones):
    home_dir = os.path.expanduser('~')
    world_path = os.path.join(home_dir, 'nlc_ws/src/bebop_gz/worlds/bebop_multi.world')
    yaml_path = os.path.join(home_dir, 'nlc_ws/src/sim_env/config/bebop_bridge.yaml')
    
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)

    sdf_content = generate_sdf_world(num_drones)
    yaml_content = generate_yaml_config(num_drones)
    
    with open(world_path, 'w') as f:
        f.write(sdf_content)
    
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    
    print(f"\nArchivos generados exitosamente:")
    print(f"- Mundo SDF: {world_path}")
    print(f"- Config YAML: {yaml_path}")
    print(f"\nSe configuraron {num_drones} drones Bebop")


def generate_sdf_world(num_drones):
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='bebop')

    # Physics
    physics = ET.SubElement(world, 'physics', name='ode_physics', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.004'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'
    ET.SubElement(physics, 'real_time_update_rate').text = '250'
    
    ode = ET.SubElement(physics, 'ode')
    solver = ET.SubElement(ode, 'solver')
    ET.SubElement(solver, 'type').text = 'world'
    ET.SubElement(solver, 'iters').text = '50'
    ET.SubElement(solver, 'sor').text = '1.0'
    
    constraints = ET.SubElement(ode, 'constraints')
    ET.SubElement(constraints, 'cfm').text = '0.00001'
    ET.SubElement(constraints, 'erp').text = '0.2'

    # World-level plugins — correct filenames for Ignition Fortress 6
    plugins = [
        ('ignition-gazebo-physics-system',            'gz::sim::systems::Physics'),
        ('ignition-gazebo-scene-broadcaster-system',  'gz::sim::systems::SceneBroadcaster'),
        ('ignition-gazebo-user-commands-system',      'gz::sim::systems::UserCommands'),
    ]
    
    for filename, name in plugins:
        ET.SubElement(world, 'plugin', filename=filename, name=name)
    
    sensors_plugin = ET.SubElement(world, 'plugin',
                                   filename='ignition-gazebo-sensors-system',
                                   name='gz::sim::systems::Sensors')
    ET.SubElement(sensors_plugin, 'render_engine').text = 'ogre2'

    # Light
    light = ET.SubElement(world, 'light', type='directional', name='sun')
    ET.SubElement(light, 'cast_shadows').text = 'true'
    ET.SubElement(light, 'pose').text = '0 0 50 0.1 0.3 -0.9'
    ET.SubElement(light, 'diffuse').text = '0.95 0.95 0.95 1'
    ET.SubElement(light, 'specular').text = '0.3 0.3 0.3 1'
    
    attenuation = ET.SubElement(light, 'attenuation')
    ET.SubElement(attenuation, 'range').text = '500'
    ET.SubElement(attenuation, 'constant').text = '0.9'
    ET.SubElement(attenuation, 'linear').text = '0.01'
    ET.SubElement(attenuation, 'quadratic').text = '0.001'

    # Ground plane
    ground = ET.SubElement(world, 'model', name='ground_plane')
    ET.SubElement(ground, 'static').text = 'true'
    
    link = ET.SubElement(ground, 'link', name='link')
    
    collision = ET.SubElement(link, 'collision', name='collision')
    geometry = ET.SubElement(collision, 'geometry')
    plane = ET.SubElement(geometry, 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = '200 200'
    
    surface = ET.SubElement(collision, 'surface')
    friction = ET.SubElement(surface, 'friction')
    ode_friction = ET.SubElement(friction, 'ode')
    ET.SubElement(ode_friction, 'mu').text = '100'
    ET.SubElement(ode_friction, 'mu2').text = '50'
    
    visual = ET.SubElement(link, 'visual', name='visual')
    geometry2 = ET.SubElement(visual, 'geometry')
    plane2 = ET.SubElement(geometry2, 'plane')
    ET.SubElement(plane2, 'normal').text = '0 0 1'
    ET.SubElement(plane2, 'size').text = '200 200'
    
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = '0.8 0.8 0.8 1'
    ET.SubElement(material, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(material, 'specular').text = '0.8 0.8 0.8 1'

    # Drones
    for i in range(1, num_drones + 1):
        bebop_name = f'bebop{i}'
        x_pos = (i - 1) * 0.3

        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = 'model://parrot_bebop_2'
        ET.SubElement(include, 'name').text = bebop_name
        ET.SubElement(include, 'pose').text = f'{x_pos} 0 0.3 0 0 0'

        # SetPose plugin
        set_pose_plugin = ET.SubElement(include, 'plugin',
                                        name='ignition::gazebo::systems::SetPosePlugin',
                                        filename='libSetPosePlugin.so')
        ET.SubElement(set_pose_plugin, 'robotNamespace').text = bebop_name
        ET.SubElement(set_pose_plugin, 'topic').text = f'{bebop_name}/set_pose'

        # PosePublisher plugin
        pose_pub_plugin = ET.SubElement(include, 'plugin',
                                        name='ignition::gazebo::systems::RobotPosePublisher',
                                        filename='libRobotPosePublisher.so')
        ET.SubElement(pose_pub_plugin, 'robotNamespace').text = bebop_name
        ET.SubElement(pose_pub_plugin, 'topic').text = f'{bebop_name}/pose'

        # Motor plugins
        motor_joints = [
            ('propeller_rr_joint', 'propeller_rr', 'ccw', 0),
            ('propeller_rl_joint', 'propeller_rl', 'cw',  1),
            ('propeller_fr_joint', 'propeller_fr', 'cw',  2),
            ('propeller_fl_joint', 'propeller_fl', 'ccw', 3),
        ]

        for joint, link_name, turn_dir, actuator_num in motor_joints:
            motor = ET.SubElement(include, 'plugin',
                                  filename='ignition-gazebo-multicopter-motor-model-system',
                                  name='gz::sim::systems::MulticopterMotorModel')
            ET.SubElement(motor, 'robotNamespace').text = bebop_name
            ET.SubElement(motor, 'jointName').text = joint
            ET.SubElement(motor, 'linkName').text = link_name
            ET.SubElement(motor, 'turningDirection').text = turn_dir
            ET.SubElement(motor, 'timeConstantUp').text = '0.0125'
            ET.SubElement(motor, 'timeConstantDown').text = '0.025'
            ET.SubElement(motor, 'maxRotVelocity').text = '400.0'
            ET.SubElement(motor, 'motorConstant').text = '8.54858e-06'
            ET.SubElement(motor, 'momentConstant').text = '0.005964552'
            ET.SubElement(motor, 'rotorDragCoefficient').text = '8.06428e-05'
            ET.SubElement(motor, 'rollingMomentCoefficient').text = '1e-06'
            ET.SubElement(motor, 'rotorVelocitySlowdownSim').text = '10'
            ET.SubElement(motor, 'motorNumber').text = str(actuator_num)
            ET.SubElement(motor, 'motorType').text = 'velocity'

        # Velocity controller
        control = ET.SubElement(include, 'plugin',
                                filename='ignition-gazebo-multicopter-control-system',
                                name='gz::sim::systems::MulticopterVelocityControl')
        ET.SubElement(control, 'robotNamespace').text = bebop_name
        ET.SubElement(control, 'commandSubTopic').text = 'cmd_vel'
        ET.SubElement(control, 'enableSubTopic').text = 'enable'
        ET.SubElement(control, 'comLinkName').text = 'body'
        ET.SubElement(control, 'velocityGain').text = '0.5 0.5 0.5'
        ET.SubElement(control, 'attitudeGain').text = '0.5 0.5 0.05'
        ET.SubElement(control, 'angularRateGain').text = '0.1 0.1 0.05'
        ET.SubElement(control, 'maximumLinearAcceleration').text = '0.5 0.5 0.5'

        rotor_config = ET.SubElement(control, 'rotorConfiguration')
        rotors = [
            ('propeller_rr_joint', '8.54858e-06', '0.005964552', '1'),
            ('propeller_rl_joint', '8.54858e-06', '0.005964552', '-1'),
            ('propeller_fr_joint', '8.54858e-06', '0.005964552', '-1'),
            ('propeller_fl_joint', '8.54858e-06', '0.005964552', '1'),
        ]
        for r_joint, force, moment, direction in rotors:
            rotor = ET.SubElement(rotor_config, 'rotor')
            ET.SubElement(rotor, 'jointName').text = r_joint
            ET.SubElement(rotor, 'forceConstant').text = force
            ET.SubElement(rotor, 'momentConstant').text = moment
            ET.SubElement(rotor, 'direction').text = direction

    rough_string = ET.tostring(sdf, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    pretty_xml = reparsed.toprettyxml(indent="  ")
    pretty_xml = '\n'.join([line for line in pretty_xml.split('\n') if line.strip()])
    return pretty_xml


def generate_yaml_config(num_drones):
    yaml_lines = []

    for i in range(1, num_drones + 1):
        bebop_name = f'bebop{i}'

        drone_config = f"""# Robot {i}
# Velocity command configuration.
- topic_name: "/{bebop_name}/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  lazy: true
  direction: ROS_TO_GZ


# Motor speed command configuration.
- topic_name: "/{bebop_name}/gazebo/command/motor_speed"
  ros_type_name: "std_msgs/msg/Float32"
  gz_type_name: "gz.msgs.Float"
  lazy: true
  direction: ROS_TO_GZ


# Enable Quadrotor command configuration.
- topic_name: "/{bebop_name}/enable"
  ros_type_name: "std_msgs/msg/Bool"
  gz_type_name: "gz.msgs.Boolean"
  lazy: true
  direction: ROS_TO_GZ


# Pose state configuration.
- topic_name: "/{bebop_name}/pose"
  ros_type_name: "geometry_msgs/msg/Pose"
  gz_type_name: "gz.msgs.Pose"
  lazy: true
  direction: GZ_TO_ROS


# Initial Pose configuration.
- topic_name: "/{bebop_name}/set_pose"
  ros_type_name: "geometry_msgs/msg/Pose"
  gz_type_name: "gz.msgs.Pose"
  lazy: true
  direction: ROS_TO_GZ


# rgbd_camera_bridge configuration.
- topic_name: "/world/bebop/model/{bebop_name}/link/body/sensor/rgb_camera_sensor/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  lazy: true
  direction: GZ_TO_ROS


- topic_name: "/world/bebop/model/{bebop_name}/link/body/sensor/rgb_camera_sensor/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  lazy: true
  direction: GZ_TO_ROS
"""
        yaml_lines.append(drone_config)

    return "\n".join(yaml_lines)


if __name__ == '__main__':
    main()