import yaml
import os
import sys

with open("/home/vishal/sample_ws/config.yaml", "r") as file:
    config = yaml.safe_load(file)
HOME_DIR = os.path.expanduser("~")


def create_workspace_path(workspace_name: str = None):
    if workspace_name is not None:
        workspace_name = workspace_name.lower()
        workspace_path = os.path.join(HOME_DIR, workspace_name)
        if not os.path.exists(workspace_path):
            os.makedirs(os.path.join(workspace_path, 'src'))
        return os.path.join(HOME_DIR, workspace_name)
    else:
        raise ValueError("Workspace name cannot be None")

def create_ros2_package(workspace_name: str, package_name: str, build_type: str, dependencies: list = None):
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    package_name = package_name.lower()
    build_type = build_type.lower()
    if os.path.exists(os.path.join(workspace_path, 'src', package_name)):
        print(os.path.join(workspace_path, 'src', package_name))
        pass
    else:
        if build_type not in ["python", "cmake"]:
            sys.exit("Build type must be either 'python' or 'cmake'")
        print("Creating ros2 package")
        if dependencies:
            dependency_string = " ".join(dependencies)
            
            os.system(f"cd {workspace_path + '/src/'} && ros2 pkg create --build-type ament_{build_type} {package_name} --dependencies {dependency_string} && cd {workspace_path} && colcon build")
        else:
            os.system(f"cd {workspace_path + '/src/'} && ros2 pkg create --build-type ament_{build_type} {package_name} && cd {workspace_path} && colcon build")

def create_config_file(workspace_name: str = None):
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    if workspace_name is None:
        config_dictionary = {'name_workspace': 'ros2_sample_ws', 'packages': [{'build_type': 'python', 'urdf': {'exists': True, 'urdf_folder_path': "ros2_sample_ws/src/sample_package/urdf"}, "launch":{
    "exists": True, 'launch_folder_path': 'ros2_sample_ws/src/sample_package/launch'}, 'name': 'sample_package', 'dependencies': ['rclpy', 'geometry_msgs', 'std_msgs'], 'nodes': [{'name': 'minimal_publisher', 'publishers': [{'name': 'sample_publisher', 'rate': 10, 'topic': '/minimal_publisher', 'type': 'String', 'queue_size': 10, 'callback_function': 'message_publisher'}], 'subscribers': None, 'services': None, 'actions': None}, {'name': 'minimal_subscriber', 'publishers': None, 'subscribers': [{'name': 'sample_subscriber', 'rate': 10, 'topic': '/minimal_publisher', 'type': 'String', 'queue_size': 10, 'callback_function': 'message_subscriber'}], 'services': None, 'actions': None}]}, {'urdf': True, 'urdf_folder_path': "ros2_sample_ws/src/sample_package/urdf"}]}
    else:
        config_dictionary = {'name_workspace': workspace_name, 'packages': [{'build_type': 'python', 'urdf': {'exists': True, 'urdf_folder_path': "ros2_sample_ws/src/sample_package/urdf"}, "launch":{
    "exists": True, 'launch_folder_path': f'{workspace_name}/src/sample_package/launch'}, 'name': 'sample_package', 'dependencies': ['rclpy', 'geometry_msgs', 'std_msgs'], 'nodes': [{'name': 'minimal_publisher', 'publishers': [{'name': 'sample_publisher', 'rate': 10, 'topic': '/minimal_publisher', 'type': 'String', 'queue_size': 10, 'callback_function': 'message_publisher'}], 'subscribers': None, 'services': None, 'actions': None}, {'name': 'minimal_subscriber', 'publishers': None, 'subscribers': [{'name': 'sample_subscriber', 'rate': 10, 'topic': '/minimal_publisher', 'type': 'String', 'queue_size': 10, 'callback_function': 'message_subscriber'}], 'services': None, 'actions': None}]}]}
    with open(os.path.join(workspace_path, 'config.yaml'), 'w') as file:
        yaml.safe_dump(config_dictionary, file)

def config_parser(workspace_name: str = "sample_ws"):
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    if not os.path.exists(os.path.join(workspace_path, 'config.yaml')):
        create_config_file(workspace_name)
    workspace_name = config.get("name_workspace")
    packages = config.get("packages")
    for package in packages:
        package_name = package.get("name")
        if package_name is not None:
            if package_name == "random":
                raise ValueError("Package name cannot be random")
            build_type = package.get("build_type")
            dependencies = package.get("dependencies")
            create_ros2_package(workspace_name=workspace_name, package_name=package_name, build_type=build_type, dependencies=dependencies)
            nodes = package.get("nodes")
            if package.get("launch").get("exists") is True:
                create_launch_file(workspace_name=workspace_name, package_name=package_name, nodes=nodes)
            for node in nodes:
                node_name = node.get('name')
                publishers = node.get('publishers')
                subscribers = node.get('subscribers')
                services = node.get('services')
                actions = node.get('actions')
                print(publishers, subscribers, services, actions)

def create_launch_file(workspace_name: str, package_name: str, nodes: list = None):
    print(f"Creating launch files for {package_name}")
    launch_import_string = '''from launch import LaunchDescription\nfrom launch_ros.actions import Node\n\ndef generate_launch_description():\n'''
    workspace_path = create_workspace_path(workspace_name=workspace_name)
    os.makedirs(os.path.join(workspace_path, 'src', package_name, 'launch'), exist_ok=True)
    nodes_string = "\t"
    return_string = []
    for node in nodes:
        nodes_string = nodes_string + f"{node.get('name')}=Node(package='{package_name}', executable='{node.get('name')}', name='{node.get('name')}')\n\t"
        # print(nodes_string)
        return_string.append(node.get('name'))
    nodes_string = nodes_string+ "\n\treturn LaunchDescription([" + ", ".join(return_string) + "])\n\n"
    with open(os.path.join(workspace_path, 'src', package_name, 'launch', f'{package_name}.launch.py'), 'w') as file:
        file.write(launch_import_string)
        file.write(nodes_string)


with open("/home/vishal/git/RoboticsLog/ros_helpers/sw2urdf_ros2/replace_files/setup.py", "r") as file:
    content = file.read()
    content = content[content.find("data_files=["):content.find("\t],")]
with open("/home/vishal/sample_ws/src/sample_package/setup.py", "r") as file2:
    content2 = file2.read()
    content2 = content2[content2.find("data_files=["):content2.find("\t],")]
    # print(content[content.find("data_files=["):content.find("\t],")])

print(content == content2)