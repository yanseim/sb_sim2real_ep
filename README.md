# sb_sim2real_ep

### log
#### grasp
* at 1st tried position control, but once finished the position control not reaching the desired grasping position, it stopped.(0320)
* so i tried veloc control, work well(0321)
* solved misdetection between O and 3(0321)
* the ideal plane is [[0,1,0],[1,0,0],[0,0,-1]](0321)
* Originally wanted to change to ore as a reference frame, but the fine adjustment of step 2 was not adjusted well, and the position where the final grasp was always tilted to the right. In the end, I could only tilt the goal to the left, barely working. But still often stuck in step 2 around the swing. At last switch to the camera as a reference frame.(0321)

### install
```
python3 -m pip install -U scipy
```