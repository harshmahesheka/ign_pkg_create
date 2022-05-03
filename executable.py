import os
def create_executable(package_path,executable_name):
        os.makedirs(package_path+"/src")
        executable_path =package_path+"/src/"+executable_name+".cc"
        executable = open(executable_path, "w")
        executable.write("//This is just a placeholder code for executable example\n")
        executable.write('''#include <cstdio>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("Hello World");
  return 0;
}''')
