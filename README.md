sot-rh_cop_interface
====================

This package provides an interface to the SoT and the rh_cop_msgs via an actionlib model.
It uses tasks based on the expressiongraph.


Setup
-----

To compile this package, it is recommended to create a separate build directory:

mkdir _build-RELEASE
cd _build-RELEASE
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_EXE_LINKER_FLAGS_RELEASE=-Xlinker -export-dynamic -Wl,-O1 -Wl,-Bsymbolic-functions -DCMAKE_INSTALL_PREFIX=/path/to/workspace/install -DCXX_DISABLE_WERROR=1 -DCMAKE_CXX_FLAGS=-O3 -pipe -fomit-frame-pointer -ggdb3 ..
make install

Please note that CMake produces a 'CMakeCache.txt' file which should 
be deleted to reconfigure a package from scratch.


Explanation
-----------

In the src/ folder you find the news operators elements to communicate with Rviz.

In the python/ folder you find many scripts:
- addTransformService.py: Provides you a service to be able to add an external frame into the SoT
- server.py: Provides you the actionlib server that communicate with the sm_client 
             in rh_cop/rh_cop_boxy package. Take care of the name of the actionlib, it must be the same
             for the client and the server.

- map_frames_joints.py: Provides you the correspondance between the frame name in Rviz
                        and the frame name in SoT
- tools_v2.py: Provides you the tools to parse the goal obtained by the server to create all elements
               required to perform it

- boxy_startup.py: It is the file that you need to initialize the SoT and Boxy

In the scripts/ folder you find:
- simple_sm_action.py: It just the same file than the one in rh_cop_bxy/rh_cop
- test_boxy_sot.py: Provides you a test experiment file to try all options avaible


Utilisation
-----------

In terminal A:
roslaunch iai_boxy_bringup boxy_sim.launch 

In terminal B:
rosparam set robot boxy
rosrun sot_controller_wrapper start_controller

In terminal C:
rosrun dynamic_graph_bridge run_command /path/to/boxy_startup.py
rosservice call /start_dynamic_graph

In terminal D:
roslaunch sot-rh_cop_interface sot_boxy_test.launch

The last launch file should launch addTransformService node, server node and client node.
To add an external frame you need to add this part on your launch file:

  <node name="name_of_tf" pkg="tf" type="static_transform_publisher" args="x y z qx qy qz qw ref_frame_id child_frame_id period_in_ms" />

  <node name="child_id_name" pkg="dynamic_graph_bridge" type="tf_publisher">
    <param name="frame" type="string" value="child_id_frame" />
    <param name="child_frame" type="string" value="odom" />
    <param name="topic" type="string" value="child_id_frame" />
  </node>


