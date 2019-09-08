
# FP.5 : Performance Evaluation 1
TTC can be minus value when the velocity is under the zero. It means that the LiDAR and the preceding vehicle are getting further.<br>
And, when the distance between the LiDAR and the preceding vehicle is constant, TTC becomes inf.<br><br>
There is not serious problem on the result of LiDAR TTC EXCEPT the last frame. <br>
**On the last frame, there wes not any point on the preceding vehicle.** It seems like that the LiDAR is facing up slightly because of the accelerating. <br>
<img src="images/fp5_graph.png" width="949" height="979" /> <br>
<br>
<img src="images/fp5.png" width="700" height="1100" /> <br><br>

### FP.5-1. Side Mirror 
When the preceding vehicle is close, the LiDAR measures the distance to objects reflected in the side mirror of the preceding vehicle.
Measured distance is totally different from the distance to the side mirror.<br><br>
<img src="images/fp5_1.png" width="538" height="895" /> <br>

### FP.5-2. Ground 
Depending on the size of the bounding box and the height threshold of the filter that crops LiDAR points, LiDAR points reflected from the road (ground) could be considered as a part of the preceding vehicle.
The case of the hill, the distance between the LiDAR and the ground is not constant.<br><br>
<img src="images/fp5_2_1.png" width="1242" height="405" /> <br>
<img src="images/fp5_2_2.png" width="700" height="730" /> <br>

### FP.5-3. Other bounding box
Lidar points on the preceding vehicle can be considered as a part of the truck on the right lane.
<br><br>
<img src="images/fp5_3.png" width="538" height="895" /> <br>

--------------
# FP.6 : Performance Evaluation 2


