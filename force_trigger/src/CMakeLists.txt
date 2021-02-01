
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libforce_sensor_probe-<target>.so
# and installs in the directory lib/orocos/force_sensor_probe/
#
orocos_component(force_trigger force_trigger-component.hpp force_trigger-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/force_sensor_probe/ directory
#
orocos_install_headers(force_trigger-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

