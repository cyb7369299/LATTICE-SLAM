# lattice_slam 
[ROS WORKSPACE]
this work is a improved version of hector slam which extra add a back-end.

corresponding video: 		        https://www.youtube.com/watch?v=N1KQz5MYElA

source code mainly lies in: 	  /xyi_v2_ws/src/xyi2_brain/src

other support packages:
        
        xyi2_bot                launch file:    robot platform: lidar drive program or...
        
        xyi2_env                launch file:    offer static map
        
        xyi2_bringup            launch file:    manage launchs
        
        xyi2_brain              src file:       lattice slam main program: mapping, navigation, graph construction...
        
        xyi2_buildmap           src file:       back_end program
        
        xyi2_pshow & xyi2_test  src file:       test code
        
main package: xyi2_brain:
/src:

        thread_manage/thread_manage.cpp:        multi-threads manage and global data allocation

        com_bus/xxx.cpp:                        com bus drive programs
        
        graph_base/xxx.cpp:                     back end graph io, structure... 
        
        map/xxx.cpp:                            static map data structure and parameters.

        matchings/xxx.cpp:                      localization based on odom + lidar data
        
        navigation/xxx.cpp:                     heuristic navigation algorithm which consider the robot's shape and dynamic map.
        
        sensors/xxx.cpp:                        sensor model, like lidar
        
        
