import os
def plugin_text_func(plguin_name):
  plugin_txt=f'''// We'll use a string and the ignmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <ignition/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <ignition/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "{plguin_name}.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    hello_world::HelloWorld,
    ignition::gazebo::System,
    hello_world::HelloWorld::ISystemPostUpdate)

using namespace hello_world;

// Here we implement the PostUpdate function, which is called at every
// iteration.
void HelloWorld::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{{
  // This is a simple example of how to get information from UpdateInfo.
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  // Messages printed with ignmsg only show when running with verbosity 3 or
  // higher (i.e. ign gazebo -v 3)
  ignmsg << msg << std::endl;
}}



'''
  return plugin_txt
def plugin_header_text_func():
  plugin_header_txt='''
#ifndef SYSTEM_PLUGIN_HELLOWORLD_HH_
#define SYSTEM_PLUGIN_HELLOWORLD_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <ignition/gazebo/System.hh>

// It's good practice to use a custom namespace for your project.
namespace hello_world
{
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class HelloWorld:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPostUpdate
  {
    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
  };
}
#endif                      
'''
  return plugin_header_txt

def create_plugin(package_path,plugin_name):
  os.makedirs(package_path+"/plugins")
  plugin_path = package_path + "/plugins/"+plugin_name+".cc"
  plugin = open(plugin_path, "w")
  plugin_text=plugin_text_func(plugin_name)
  plugin.write(plugin_text)
  plugin_header_path = package_path + "/plugins/"+plugin_name+".hh"
  plugin = open(plugin_header_path, "w")
  plugin_header_text=plugin_header_text_func()
  plugin.write(plugin_header_text)
          