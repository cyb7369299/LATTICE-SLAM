# lattice_slam
this work is a improved version of hector slam which extra add a back-end.

corresponding video: 		        https://www.youtube.com/watch?v=N1KQz5MYElA

source code mainly lies in: 	  /xyi_v2_ws/src/xyi2_brain/src

others:
        xyi2_bot                robot launch file
        
        xyi2_env                robot environment support (simulation, drawn map) launch file
        
        xyi2_pshow & xyi2_test  test code
        
        xyi2_buildmap           back_end program
        
        xyi2_bringup            launch manage pkg
        
        xyi2_brain              lattice slam main program: mapping, navigation, graph construction...
        

mulit-thread manage.cpp:        /xyi_v2_ws/src/xyi2_brain/src/thread_manage/thread_manage.cpp
