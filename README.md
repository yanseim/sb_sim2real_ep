# sb_sim2real_ep

### log
#### grasp
* at 1st tried position control, but once finished the position control not reaching the desired grasping position, it stopped.(0320)
* so i tried veloc control, work well(0321)
* solved misdetection between O and 3(0321)
* the ideal plane is [[0,1,0],[1,0,0],[0,0,-1]](0321)
* Originally wanted to change to ore as a reference frame, but the fine adjustment of step 2 was not adjusted well, and the position where the final grasp was always tilted to the right. In the end, I could only tilt the goal to the left, barely working. But still often stuck in step 2 around the swing. At last switch to the camera as a reference frame.(0321)
* grasp/place service and client are finished. clip is awesome.(0323)

### installation
```
python3 -m pip install -U scipy
```

### Usage
* run roscore, server, keyboard
* remember to publish tf!
    ```
    roslaunch sb_ep_descreption ep_description.launch
    ```
* run sb_detect_cube.py
* run sb_grasp_cube_server.py, sb_place_cube_server.py (maybe have error, leave them alone)
* modify test_grasp_place_client.py according to your situations and run test_grasp_place_client.py
