    src/: This folder contains the source code of your ROS package, including all ROS nodes, libraries, or other code files you have developed.

    include/: This folder is used to store header files (.h) for your ROS package-specific classes or functions.

    config/: Can contain various types of configuration files, depending on the needs of the package. These files may include parameters, settings, calibration data, or any other configuration information required by the nodes in the package.

    launch/: Here, you store launch files (.launch) that are used to start and configure ROS nodes with the appropriate parameters.

    cfg/: If you implement dynamic reconfiguration for your ROS package, the corresponding .cfg files can be placed in this folder.

    scripts/: This folder contains executable script files (usually in Python or Bash) used for specific tasks in your ROS package.

    rviz/: it likely contains the RViz configuration files specific to that package

    test/: Here, you place test files (usually in Python or C++) to test the functionality of your ROS package.

    package_template_msgs/msg/: If you create your own ROS messages, the corresponding .msg files are placed in this folder.

    package_template_msgs/srv/: Similar to messages, if you create your own ROS service definitions, the corresponding .srv files are stored here.

    package_template_msgs/action/: If you implement an action server in your package, the corresponding .action files are placed in this folder.

    CMakeLists.txt and package.xml: These files are required to build and configure the ROS package. They contain information such as the package name, dependencies, compiler options, etc.