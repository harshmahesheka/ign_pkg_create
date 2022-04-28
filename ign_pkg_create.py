import argparse
import os
import requests
import yaml


#Generates colcon.pkg for sourcing hooks
def colcon_pkg_create():
    colcon_pkg_path = package_path + "/colcon.pkg"
    colcon_pkg = open(colcon_pkg_path, "w")
    colcon_pkg.write(
         "{\n" 
        f'    "hooks": ["share/{args.name[0]}/hooks/hook.dsv"]\n' 
         "}"
    )

#Generates hooks for exporting path
def hooks_create():
    hooks_path = package_path + "/hooks"
    os.makedirs(hooks_path)
    hooks_dsv_path = hooks_path + "/hook.dsv.in"
    hooks_dsv = open(hooks_dsv_path, "w")
    hooks_dsv.write(
        "prepend-non-duplicate;IGN_GAZEBO_RESOURCE_PATH;@CMAKE_INSTALL_PREFIX@/share/@PROJECT_NAME@/models\n"
        "prepend-non-duplicate;IGN_GAZEBO_RESOURCE_PATH;@CMAKE_INSTALL_PREFIX@/share/@PROJECT_NAME@/worlds\n"
        "prepend-non-duplicate;IGN_GAZEBO_SYSTEM_PLUGIN_PATH;@CMAKE_INSTALL_PREFIX@/lib\n"
        "prepend-non-duplicate;LD_LIBRARY_PATH;@CMAKE_INSTALL_PREFIX@/lib\n"
    )

#Adds standard ignition package dependencies 
def dependencies():
      ign_gazebo_version = str(args.ignition_version[0])

      #Fetches required ignition package version from internet based on user input
      if ign_gazebo_version == "3":
        dependencies_file = requests.get("https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-citadel.yaml")
      elif ign_gazebo_version == "4":
        dependencies_file = requests.get("https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-dome.yaml")
      elif ign_gazebo_version == "5":
        dependencies_file = requests.get("https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-edifice.yaml")
    #   elif ign_gazebo_version == "7":
    #     dependencies_file = requests.get("https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-garden.yaml")
      else:
        dependencies_file = requests.get("https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml")
      dependencies_yaml=yaml.safe_load(dependencies_file.content)
      ign_plugin_version =dependencies_yaml['repositories']['ign-plugin']['version']
      ign_cmake_version = dependencies_yaml['repositories']['ign-cmake']['version']
      ign_common_version = dependencies_yaml['repositories']['ign-common']['version']
      ign_math_version = dependencies_yaml['repositories']['ign-math']['version']
      ign_rendering_version = dependencies_yaml['repositories']['ign-rendering']['version']
      ign_transport_version = dependencies_yaml['repositories']['ign-transport']['version']
      ign_msgs_version = dependencies_yaml['repositories']['ign-msgs']['version']
      ign_fuel_version = dependencies_yaml['repositories']['ign-fuel-tools']['version']
      ign_gazebo_version=dependencies_yaml['repositories']['ign-gazebo']['version']
      dependencies_txt = (
        f"find_package({ign_cmake_version} REQUIRED)\n"
        f"find_package({ign_gazebo_version} REQUIRED COMPONENTS gui)\n"
        f"set(IGN_GAZEBO_VER ${{{ign_gazebo_version}_VERSION_MAJOR}})\n"
        f"find_package({ign_common_version }  REQUIRED COMPONENTS graphics)\n"
        f"set(IGN_COMMON_VER ${{{ign_common_version}_VERSION_MAJOR}})\n"
        f"find_package({ign_fuel_version} REQUIRED)\n"
        f"find_package({ign_math_version} REQUIRED)\n"
        f"set(IGN_MATH_VER ${{{ign_math_version}_VERSION_MAJOR}})\n"
        f"find_package({ign_rendering_version} REQUIRED)\n"
        f"set(IGN_RENDERING_VER ${{{ign_rendering_version}_VERSION_MAJOR}})\n")
      return dependencies_txt

#Creates CMakelists.txt file for package
def CMakeLists_create():
    CMakeLists_path = package_path + "/CMakeLists.txt"
    CMakeLists = open(CMakeLists_path, "w")
    CMakeLists.write("cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)\n"
                    f"project({args.name[0]})\n")
    if args.standard_dependencies[0] == "True":
        dependenices_txt = dependencies()
        CMakeLists.write(dependenices_txt)
    if args.package_type[0] == "advanced":
        CMakeLists.write(
            "configure_file(\n"
            '      "hooks/hook.dsv.in"\n'
            '      "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY'
            ")\n"
            "install(DIRECTORY\n"
            "      worlds\n"
            "      models\n"
            "      launch\n"
            "      ${CMAKE_CURRENT_BINARY_DIR}/hooks\n"
            "      DESTINATION share/${PROJECT_NAME})\n"
        )

    if args.built_type[0] == "ament_cmake":
        CMakeLists.write("find_package(ament_cmake REQUIRED)\n")
    CMakeLists.write("if(BUILD_TESTING)\n")
    if args.built_type[0] == "ament_cmake":
        CMakeLists.write("  find_package(ament_cmake_gtest REQUIRED)\n\n")
    CMakeLists.write(
        '  set("PROJECT_BINARY_PATH" ${CMAKE_CURRENT_BINARY_DIR})\n'
        '  set("PROJECT_SOURCE_PATH" ${CMAKE_CURRENT_SOURCE_DIR})\n\n'
        "endif()\n"
    )
    if args.built_type[0] == "ament_cmake":
        CMakeLists.write("ament_package()\n")

#Generates package.xml file if required
def package_xml_create():
    package_xml_path = package_path + "/package.xml"
    package_xml = open(package_xml_path, "w")
    package_xml.write(
        '<?xml version="1.0"?>\n'
        '<package format="2">\n'
        f"<name> {args.name[0]} </name>\n"
        "<version>0.0.0</version>\n"
        "<description>Add a short description of your project</description>\n"
        '<maintainer email="youremail@domain.com">Your Name</maintainer>\n'
        "<license>Apache License 2.0</license>\n"
        "<author>Your Name</author>\n"
        "<buildtool_depend>ament_cmake</buildtool_depend>\n"
        "<exec_depend>ament_index_python</exec_depend>\n"
        "<exec_depend>launch</exec_depend>\n"
        "<exec_depend>launch_ros</exec_depend>\n"
        "<exec_depend>xacro</exec_depend>\n"
        "<test_depend>ament_cmake_gtest</test_depend>\n"
        "<test_depend>ament_lint_auto</test_depend>\n"
        "<test_depend>ament_lint_common</test_depend>\n"
        "<export>\n"
        "    <build_type>ament_cmake</build_type>\n"
        "</export>\n"
        "</package>"
    )
if __name__ == "__main__":
    #Takes required inputs from the user
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "name", metavar="name", type=str, nargs=1, help="name of new package")

    parser.add_argument(
        "path",
        metavar="path",
        type=str,
        nargs=1,
        help="path of the directory where new package will be created")

    parser.add_argument(
        "built_type",
        metavar="built_type",
        type=str,
        nargs=1,
        help="the build type to process the package between ament_cmake or cmake")

    parser.add_argument(
        "ignition_version",
        metavar="ignition_version",
        type=int,
        nargs=1,
        help="version of igniton(integer) on which package will depend")

    parser.add_argument(
        "-package_type",
        metavar="{basic,advanced}",
        type=str,
        nargs=1,
        help="complexity of package between basic or advanced",
        default="basic")

    parser.add_argument(
        "-standard_dependencies",
        metavar="{True,False}",
        type=str,
        nargs=1,
        help="will the package depend on standard ignition packages",
        default=["True"])

    #Creates required files based on arguments given
    args = parser.parse_args()
    package_path = os.path.join(args.path[0], args.name[0])
    os.makedirs(package_path)
    CMakeLists_create()
    if args.built_type[0] == "ament_cmake":
        package_xml_create()

    if args.package_type[0] == "advanced":
        colcon_pkg_create()
        hooks_create()
        os.makedirs(package_path+"/worlds")
        os.makedirs(package_path+"/models")
        os.makedirs(package_path+"/launch")
