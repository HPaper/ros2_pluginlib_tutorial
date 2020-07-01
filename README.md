# Description
This repository includes tutorial packages to use ros pluginlib which tools for writing and dynamically loading plugins using the ROS build infrastructure in ROS2 environments. This repository includes three packages. The `polygon_base` package declares a virtual class that can insert plug-in classes. The `plugin_application` package includes application node using the plugin class. In the `polygon_plugins` package, a plugin class that uses pluginlib is declared, and creates an XML file the plugin loader to finds a library and know what to reference within the library.

# Reference
- pluginlib/Tutorials/Writing and Using a Simple Plugin in ``ROS.org``

url : http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin
