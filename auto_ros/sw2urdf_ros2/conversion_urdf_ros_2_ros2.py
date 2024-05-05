import os
import argparse

argument_parser = argparse.ArgumentParser()
argument_parser.add_argument("-s", "--source_dir", required=True, help="Complete source directory path", type=str)
argument_parser.add_argument("-t", "--target_dir", required=True, help="Complete target directory path", type=str)
argument_parser.add_argument("-p", "--package_name", required=True, help="Name of the ros2 package", type=str)
argument_parser.add_argument("-o", "--output_folder_name", help="Name of the output folder. Defaults to the name of the package.", type=str, default=argument_parser.parse_args().package_name + "_urdf")
# Configuration variable

# This variable is the path to the solidworks output folder of urdf files
# source_dir = '/home/sxm/Project/ros/test_urdf/'
source_dir = argument_parser.parse_args().source_dir #'/home/vishal/Documents/servo_motor_6dof_arm/'

# This variable is the package path of ros2
# target_dir = '/home/sxm/Project/ros/test_urdf_tool_ws/src/test_urdf_tool/'
target_dir = argument_parser.parse_args().target_dir #'/home/vishal/Documents/robot_ws/src/robot_arm/'
# This variable is the package-name of ros2
package_name = argument_parser.parse_args().package_name #"robot_arm"

# This variable is the solidworks output folder name
# output_folder_name = "test_urdf"
output_folder_name = argument_parser.parse_args().output_folder_name #"servo_motor_6dof_arm"


def run_command_dir(command_dir, command):
    # print(f"cp  {source_dir}/urdf/" + output_folder_name + ".urdf " + target_dir + "urdf/")
    os.system("cd " + command_dir + " && " + command)


def replace_str(file, old_str, new_str):
    file_data = ""
    with open(file, "r+", encoding="utf-8") as file:
        for line in file:
            if old_str in line:
                line = line.replace(old_str, new_str)
            file_data += line
    with open(file, "w", encoding="utf-8") as file:
        file.write(file_data)


if __name__ == '__main__':
    # print(argument_parser.parse_args().output_folder_name)
    # Create folders
    run_command_dir(target_dir, "mkdir launch meshes meshes/collision meshes/visual urdf")

    # Copy files
    # Copy stl files
    run_command_dir(source_dir, "cp -r ./meshes/* " + target_dir + "/meshes/visual")
    run_command_dir(source_dir, "cp -r ./meshes/* " + target_dir + "/meshes/collision")
    # Copy urdf files
    run_command_dir(source_dir, f"cp  {source_dir}/urdf/" + output_folder_name + ".urdf " + target_dir + "urdf/")

    # replace files
    os.system("cp -f /home/vishal/git/RoboticsLog/ros_helpers/sw2urdf_ros2/replace_files/setup.py " + target_dir)
    os.system("cp -f /home/vishal/git/RoboticsLog/ros_helpers/sw2urdf_ros2/replace_files/package.xml " + target_dir)
    os.system("cp -f /home/vishal/git/RoboticsLog/ros_helpers/sw2urdf_ros2/replace_files/launch.py " + target_dir + "/launch")

    # Change file content
    # launch.py
    replace_str(target_dir + "/launch/launch.py", "lesson_urdf", package_name)
    replace_str(target_dir + "/launch/launch.py", "planar_3dof.urdf", output_folder_name + ".urdf")
    # setup.py
    replace_str(target_dir + "/setup.py", "lesson_urdf", package_name)
    # package.xml
    replace_str(target_dir + "/package.xml", "lesson_urdf", package_name)
    # urdf files
    replace_str(target_dir + "/urdf/" + output_folder_name + ".urdf", output_folder_name + "/meshes",
                package_name + "/meshes/visual")

    # Insert base_footprint
    keyword = "name=\"" + output_folder_name + "\">"
    str = ""
    with open("/home/vishal/git/RoboticsLog/ros_helpers/sw2urdf_ros2/replace_files/insert_content.txt", "r", encoding="utf-8") as file:
        str = file.read()

    file = open(target_dir + "/urdf/" + output_folder_name + ".urdf", 'r')
    content = file.read()
    post = content.find(keyword)
    if post != -1:
        content = content[:post + len(keyword)] + "\n" + str + content[post + len(keyword):]
        file = open(target_dir + "/urdf/" + output_folder_name + ".urdf", "w")
        file.write(content)
    file.close()

    print("conversion success!")
