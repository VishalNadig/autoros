import os
import sys
import yaml
import shutil
import subprocess
import xml.etree.ElementTree as ET
import pymeshlab


HOME_DIR = os.path.expanduser("~")

def create_workspace_path(workspace_name: str = None):
    """
    Creates a workspace path based on the given workspace name.

    Args:
        workspace_name (str, optional): The name of the workspace. Defaults to None.

    Returns:
        str: The path to the workspace.

    Raises:
        ValueError: If the workspace name is None.
    """
    if workspace_name is None:
        raise ValueError("Workspace name cannot be None")
    if " " in workspace_name:
        raise ValueError("Workspace name cannot contain spaces. Check the config file for the 'name_workspace' field.")
    workspace_name = workspace_name.lower()
    workspace_path = os.path.join(HOME_DIR, workspace_name)
    if not os.path.exists(workspace_path):
        os.makedirs(os.path.join(workspace_path, 'src'))
    return os.path.join(workspace_path)

def create_ros2_workspace(workspace_name: str, build_type: str, package_name: str, dependencies: list = None):
    """
    Creates a ROS 2 workspace and generates a ROS 2 package inside the workspace.

    Args:
        workspace_name (str): The name of the workspace.
        build_type (str): The type of build for the package ('python' or 'cmake').
        package_name (str): The name of the ROS 2 package to be created.
        dependencies (list, optional): A list of package dependencies. Defaults to None.
    """
    if " " in package_name or package_name is None:
        raise ValueError("Package name cannot contain spaces. Check the config file for the 'packages: name' field.")
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    package_name = package_name.lower()
    build_type = build_type.lower()
    if build_type not in ["python", "cmake"]:
        sys.exit("Build type must be either 'python' or 'cmake'")
    if dependencies:
        subprocess.run(f"cd {os.path.join(workspace_path, 'src')} && ros2 pkg create --build-type ament_{build_type} {package_name} --dependencies {dependencies}", shell=True)
    else:
        subprocess.run(f"cd {os.path.join(workspace_path, 'src')} && ros2 pkg create --build-type ament_{build_type} {package_name}", shell=True)
    subprocess.run(f"cd {os.path.join(workspace_path, 'src')}  && colcon build --packages-select {package_name}", shell=True)

def create_ros2_package(workspace_name: str, package_name: str, build_type: str, dependencies: list = None):
    """
    Creates a ROS 2 package inside a specified workspace.

    Args:
        workspace_name (str): The name of the workspace.
        package_name (str): The name of the ROS 2 package to be created.
        build_type (str): The type of build for the package ('python' or 'cmake').
        dependencies (list, optional): A list of package dependencies. Defaults to None.

    Raises:
        SystemExit: If the build type is not 'python' or 'cmake'.

    Returns:
        None
    """
    sys.stdout.write("Creating ROS 2 package...\n")
    if " " in package_name:
        raise ValueError("Package name cannot contain spaces. Check the config file for the 'name_package' field.")
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    package_name = package_name.lower()
    build_type = build_type.lower()
    dependency_string = " ".join(dependencies)
    if not os.path.exists(os.path.join(workspace_path, 'src')):
        sys.stdout.write("Creating package src path...\n")
        subprocess.run("mkdir -p " + workspace_path + "/src", shell=True)

    else:
        if build_type not in ["python", "cmake"]:
            sys.exit("Build type must be either 'python' or 'cmake'")
        if os.path.exists(os.path.join(workspace_path, 'src', package_name)):
            sys.stdout.write("Package already exists...\n")
        else:
            if dependencies:
                sys.stdout.write("Dependencies found...\n")
                subprocess.run(f"cd '{workspace_path + '/src/'}' && ros2 pkg create --build-type ament_{build_type} {package_name} --dependencies {dependency_string} && cd {workspace_path} && colcon build", shell=True)
            else:
                subprocess.run(f"cd '{workspace_path + '/src/'}' && ros2 pkg create --build-type ament_{build_type} {package_name} && cd {workspace_path} && colcon build", shell=True)

    if build_type not in ["python", "cmake"]:
            sys.exit("Build type must be either 'python' or 'cmake'")
    if dependencies:
        sys.stdout.write("Dependencies found...\n")
        subprocess.run(f"cd {workspace_path + '/src/'} && ros2 pkg create --build-type ament_{build_type} {package_name} --dependencies {dependency_string} && cd {workspace_path} && colcon build", shell=True)
    else:
        subprocess.run(f"cd {workspace_path + '/src/'} && ros2 pkg create --build-type ament_{build_type} {package_name} && cd {workspace_path} && colcon build", shell=True)

def get_interface_type(workspace_name: str, type: str):
    """
    Retrieves the interface type for a given workspace and type.

    Args:
        workspace_name (str): The name of the workspace.
        type (str): The type of the interface.

    Returns:
        str: The import statement for the interface type.

    Raises:
        None
    """
    worspace_path = create_workspace_path(workspace_name=workspace_name)
    with open(os.path.join(worspace_path, 'ros2_interfaces.yaml')) as file:
        interface_file = yaml.safe_load(file)
    if type in interface_file['Messages'].keys():
        interface_type = f"from {interface_file['Messages'][type]} ".replace('/', '.') + f"import {type}\n"
    elif type in interface_file['Services']:
        interface_type = f"from {interface_file['Services'][type]} ".replace('/', '.') + f"import {type}\n"
    else:
        interface_type = f"from {interface_file['Actions'][type]} ".replace('/', '.') + f"import {type}\n"
    return interface_type

def create_urdf_files(workspace_name: str, package_name: str, urdf_path: str = None):
    """
    Creates URDF files for a given workspace, package, and URDF path.

    Args:
        workspace_name (str): The name of the workspace.
        package_name (str): The name of the package.
        urdf_path (str, optional): The path to the URDF files. Defaults to None.

    Returns:
        None
    """
    # if " " in urdf_path:
    #     raise ValueError("URDF path cannot contain spaces. Check the config file for the urdf: 'urdf_folder_path' field.")
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    if " " in package_name:
        raise ValueError("Package name cannot contain spaces. Check the config file for the 'name_package' field.")
    if not os.path.exists(os.path.join(workspace_path, 'src', package_name, "rviz")):
        os.makedirs(os.path.join(workspace_path, 'src', package_name, "rviz"))
    urdf_target_path = os.path.join(workspace_path, 'src', package_name)
    if urdf_path and os.path.exists((urdf_path)):
        if not os.path.exists(os.path.join(urdf_path,'meshes','visual')) or not os.path.exists(os.path.join(urdf_path,'meshes','collision')):
            os.makedirs(os.path.join(urdf_path,'meshes','visual'))
            os.makedirs(os.path.join(urdf_path,'meshes','collision'))
            subprocess.run(f"cp {urdf_path}/meshes/* {urdf_path}/meshes/visual/", shell=True)
            subprocess.run(f"cp {urdf_path}/meshes/* {urdf_path}/meshes/collision/", shell=True)
        if os.path.exists(os.path.join(urdf_path, "package.xml")) or os.path.exists(os.path.join(urdf_path, "CMakeLists.txt")):
            subprocess.run(f"rm {os.path.join(urdf_path, 'package.xml')} {os.path.join(urdf_path, 'CMakeLists.txt')}", shell=True)
        subprocess.run(f"cp -r {urdf_path}/* {urdf_target_path}/", shell=True)
    else:
        directories = ['urdf', 'meshes', 'meshes/collision', 'meshes/visual', 'config', 'textures']
        for directory in directories:
            os.makedirs(os.path.join(workspace_path, 'src', package_name, directory), exist_ok=True)
    if os.path.exists(os.path.join(workspace_path, 'src', package_name, "launch", "display.launch")) and os.path.exists(os.path.join(workspace_path, 'src', package_name, "launch", "gazebo.launch")):
        subprocess.run(f"mv {os.path.join(workspace_path, 'src', package_name, 'launch', 'display.launch')} {os.path.join(workspace_path, 'src', package_name, 'launch', 'display.launch.py')} && mv {os.path.join(workspace_path, 'src', package_name, 'launch', 'gazebo.launch')} {os.path.join(workspace_path, 'src', package_name, 'launch', 'gazebo.launch.py')}", shell=True)

def create_launch_file(workspace_name: str, package_name: str, nodes: list = None, launch_folder: str = None, filename: str = "sample", launch_gazebo: bool = False, controllers: list = None, broadcasters: list = None):
    """
    Creates launch files for a given workspace, package, and list of nodes.

    Args:
        workspace_name (str): The name of the workspace.
        package_name (str): The name of the package.
        nodes (list, optional): A list of dictionaries representing the nodes. Each dictionary should have the keys 'name' and 'type'. Defaults to None.
        launch_folder (str, optional): The path to the launch folder. Defaults to None.

    Returns:
        None
    """
    if " " in package_name:
        raise ValueError("Package name cannot contain spaces. Check the config file for the 'name_package' field.")
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    sys.stdout.write(f"Creating launch files for {package_name}\n")
    for file in os.listdir(os.path.join(workspace_path, 'src', package_name, 'urdf')):
        if file.endswith(".urdf"):
            urdf_filename = file
            break
        # subprocess.run("ros2 run gazebo_ros spawn_entity.py -entity " + package_name + " -file " + os.path.join(workspace_path, 'src', package_name, 'gazebo', urdf_filename + "-topic robot_description"))

    controllers_string = ""
    add_ld_controllers_string = ""
    for controller in controllers:
        controllers_string += f"""\n\n\tload_{controller} = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', '{controller}'],
            output='screen')\n\n\t"""
        add_ld_controllers_string += f"\n\tld.add_action(load_{controller})\n\t"
    
    broadcasters_string = ""
    add_ld_broadcasters_string = ""
    for broadcaster in broadcasters:
        broadcasters_string += f"""\n\n\tload_{broadcaster} = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', '{broadcaster}'],
            output='screen')\n\n\t"""
        add_ld_broadcasters_string += f"\n\tld.add_action(load_{broadcaster})\n\t"


    if urdf_filename != "None":
        nodes_string = """\tdeclare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'config.rviz'),
        description='Full path to the RVIZ config file to use')
\tdeclare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
\tdeclare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Whether to start the joint state publisher')
\tdeclare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

\tdeclare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(bringup_dir, 'urdf', """ + f"'{urdf_filename}'" + """),
        description='Whether to start RVIZ')\n\n


\tgazebo_cmd = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen')\n\n

\tspawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot_1', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-topic', '/robot_description'],
        output='screen')\n\n

\tstart_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        #parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file])\n\n

\trviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')""" + controllers_string + broadcasters_string
    else:
        nodes_string = """\tdeclare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'config.rviz'),
        description='Full path to the RVIZ config file to use')
\tdeclare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
\tdeclare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Whether to start the joint state publisher')
\tdeclare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

\tgazebo_cmd = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen')\n\n

\tspawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot_1', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-topic', '/robot_description'],
        output='screen')\n\n

\tstart_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        #parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file])\n\n

\trviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')\n\n\t""" + controllers_string + broadcasters_string
    if launch_gazebo:
        add_ld = """
\tld.add_action(declare_rviz_config_file_cmd)
\tld.add_action(declare_urdf_cmd)
\tld.add_action(declare_use_robot_state_pub_cmd)
\tld.add_action(declare_use_joint_state_pub_cmd)
\tld.add_action(declare_use_rviz_cmd)
\tld.add_action(start_robot_state_publisher_cmd)
\tld.add_action(gazebo_cmd)
\tld.add_action(spawn_robot)
\tld.add_action(rviz_cmd)\n\n\t""" + add_ld_controllers_string + add_ld_broadcasters_string
    else:
        add_ld = """
\tld.add_action(declare_rviz_config_file_cmd)
\tld.add_action(declare_urdf_cmd)
\tld.add_action(declare_use_robot_state_pub_cmd)
\tld.add_action(declare_use_joint_state_pub_cmd)
\tld.add_action(start_robot_state_publisher_cmd)
\tld.add_action(declare_use_rviz_cmd)
\tld.add_action(rviz_cmd)\n\n\t""" + add_ld_controllers_string + add_ld_broadcasters_string

    launch_import_string = f'''import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node\n\nfrom ament_index_python import get_package_prefix
pkg_share_path = os.pathsep + os.path.join(get_package_prefix('{package_name}'), 'share')
if 'GAZEBO_MODEL_PATH' in os.environ:
    os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
else:
    os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path\n\ndef generate_launch_description():\n''' + f"\tbringup_dir = get_package_share_directory('{package_name}')\n\n" + "\tlaunch_dir = os.path.join(bringup_dir, 'launch')\n\n" + """\trviz_config_file = LaunchConfiguration('rviz_config_file')
\tuse_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
\tuse_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
\tuse_rviz = LaunchConfiguration('use_rviz')
\turdf_file= LaunchConfiguration('urdf_file')\n\n"""
    for node in nodes:
        nodes_string = nodes_string + f"{node.get('name')}=Node(package='{package_name}', executable='{node.get('name')}', name='{node.get('name')}')\n\t"
        add_ld = add_ld + f"ld.add_action({node.get('name')})\n\t"
    nodes_string = nodes_string+ "\n\tld = LaunchDescription()\n\n" + "\t" + add_ld + "\n\treturn ld\n\n"
    os.makedirs(os.path.join(workspace_path, 'src', package_name, launch_folder), exist_ok=True)
    with open(os.path.join(workspace_path, 'src', package_name, launch_folder, f'{filename}'), 'w') as file:
        file.write(launch_import_string)
        file.write(nodes_string)

def modify_setup_file(workspace_name: str, package_name: str, console_scripts: list = [], maintaners: list = None, maintainer_emails: list = None, package_description: str = None, version: str = None, gazebo_folder_path: str = None):
    """
    Modifies the setup file of a given package in a specified workspace.

    Args:
        workspace_name (str): The name of the workspace where the package is located.
        package_name (str): The name of the package.
        console_scripts (list, optional): A list of console scripts to be added to the entry points. Defaults to an empty list.

    Returns:
        None
    """
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    if " " in package_name:
        raise ValueError("Package name cannot contain spaces. Check the config file for the 'name_package' field.")
    if maintainer_emails is None:
        maintainer_emails = ['sample@example.com']
    if maintaners is None:
        maintaners = ['John Doe', 'Jane Doe']
    if package_description is None:
        package_description = 'This is a sample package.'
    if version is None:
        version = '0.0.0'
    if gazebo_folder_path:
        setup_file_string = f"""from setuptools import setup
from glob import glob

import os
package_name = '{package_name}'

setup(
    name=package_name,
    version='{version}',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),glob('launch/*launch.[pxy][yma]*')),
    (os.path.join('share', package_name, 'urdf/'),glob('urdf/*.[urdf]*')),
    (os.path.join('share', package_name, 'rviz/'),glob('riviz/*.[rviz]*')),
    (os.path.join('share', package_name, 'meshes/collision/'), glob('meshes/collision/*')),
    (os.path.join('share', package_name, 'meshes/visual/'), glob('meshes/visual/*')),
    (os.path.join('share', package_name, 'meshes/'), glob('meshes/*.STL')),
    (os.path.join('share', package_name, 'config/'), glob('config/*')),
    (os.path.join('share', package_name, 'model/'), glob('model/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainers='{" ".join(maintaners)}',
    maintainer_emails='{" ".join(maintainer_emails)}',
    description='{package_description}',
    license='TODO: License declaration',
    tests_require=['pytest'],""" + """\n\tentry_points={
        'console_scripts':""" + f"\n\t\t{console_scripts}"  + """,
    },\n)"""
    else:
        setup_file_string = f"""from setuptools import setup
from glob import glob

import os
package_name = '{package_name}'

setup(
    name=package_name,
    version='{version}',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),glob('launch/*launch.[pxy][yma]*')),
    (os.path.join('share', package_name, 'urdf/'),glob('urdf/*.[urdf]*')),
    (os.path.join('share', package_name, 'rviz/'),glob('riviz/*.[rviz]*')),
    (os.path.join('share', package_name, 'meshes/collision/'), glob('meshes/collision/*')),
    (os.path.join('share', package_name, 'meshes/visual/'), glob('meshes/visual/*')),
    (os.path.join('share', package_name, 'meshes/'), glob('meshes/*.STL')),
    (os.path.join('share', package_name, 'config/'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainers='{" ".join(maintaners)}',
    maintainer_emails='{" ".join(maintainer_emails)}',
    description='{package_description}',
    license='TODO: License declaration',
    tests_require=['pytest'],""" + """\n\tentry_points={
        'console_scripts':""" + f"\n\t\t{console_scripts}"  + """,
    },\n)"""
    with open(os.path.join(workspace_path, 'src', package_name, "setup.py"), 'w') as file:
        file.write(setup_file_string)

def create_interface_file(workspace_name: str):
    """
    Creates an interface file for a given workspace name.

    Args:
        workspace_name (str): The name of the workspace.

    Returns:
        None
    """
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    subprocess.run(f"ros2 interface list >> {os.path.join(workspace_path, 'ros2_interfaces.yaml')}", shell=True)
    with open(os.path.join(workspace_path, "ros2_interfaces.yaml")) as file:
        interface_file = yaml.safe_load(file)

    interface_file['Messages'] = interface_file['Messages'].split(" ")
    interface_file['Services'] = interface_file['Services'].split(" ")
    interface_file['Actions'] = interface_file['Actions'].split(" ")
    sample_dictionary = {}
    for message in interface_file['Messages']:
        last_word = message.split("/")[-1]
        sample_dictionary[last_word] = message.removesuffix(f"/{last_word}")
    interface_file['Messages'] = sample_dictionary
    sample_dictionary = {}
    for action in interface_file['Actions']:
        last_word = action.split("/")[-1]
        sample_dictionary[last_word] = action.removesuffix(f"/{last_word}")
    interface_file['Actions'] = sample_dictionary
    sample_dictionary = {}
    for service in interface_file['Services']:
        last_word = service.split("/")[-1]
        sample_dictionary[last_word] = service.removesuffix(f"/{last_word}")
    interface_file['Services'] = sample_dictionary
    with open(os.path.join(workspace_path, "ros2_interfaces.yaml"), "w") as file:
        yaml.dump(interface_file, file)

if __name__ == "__main__":
    create_ros2_package()