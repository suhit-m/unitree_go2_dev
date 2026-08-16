Typically you use Docker and docker_shared to use SCOTS, but luckily there's a local installation at ../scots

BasicPIDControlTest.py, GetSportModeTest.py, and SCOTSControlTest.py are all (likely) functional test programs that experiment with the GO2 SDK as well as the symbolic controller toolbox, SCOTS.

AutoDeploy_GO2.py is the main program, which calls upon scots_dev/go2_controller.cc to create the controller and SCOTSDeploy.py to act as the pipeline between go2_controller.cc and arena_config.txt.