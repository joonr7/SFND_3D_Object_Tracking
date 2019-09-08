
# FP.5 : Performance Evaluation 1
TTC can be minus value when the velocity is under the zero. It means that the LiDAR and the preceding vehicle are getting further.<br>
And, when the distance between the LiDAR and the preceding vehicle is constant, TTC becomes inf.<br>
**Caution> After the 50th image, the distance between the LiDAR and the preceding vehicle may be constant (not moving.)**<br><br>
There is not serious problem on the result of LiDAR TTC EXCEPT the last frame. <br>
**On the last frame, there wes not any point on the preceding vehicle.** It seems like that the LiDAR is facing up slightly because of the accelerating. <br><br><br>
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

Looking at the following table and the graph, SIFT-SIFT is the closest graph to the LiDAR TTC graph.<br>
However, BRIEF descriptor is much more faster than SIFT descriptor.<br>
So, I took the **SIFT-BRIE** as the best detector / descriptor combinations.
Ave.(diff) = AVERAGE( (lidar tcc - )^2 ) from image 1 to image 50.<br><br>
<img src="images/fp6_rank.png" width="490" height="800" /> <br>

<br><br>
<img src="images/TTC_CAMERA_1_SHITOMASI.png" width="1067" height="453" /> <br>
<img src="images/TTC_CAMERA_2_FAST.png" width="1067" height="453" /> <br>
<img src="images/TTC_CAMERA_3_BRISK.png" width="1067" height="453" /> <br>
<img src="images/TTC_CAMERA_4_ORB.png" width="1067" height="453" /> <br>
<img src="images/TTC_CAMERA_5_AKAZE.png" width="1067" height="453" /> <br>
<img src="images/TTC_CAMERA_6_SIFT.png" width="1067" height="453" /> <br>
SIFT-ORB(X)->SIFT-BRIEF<br>

<!-- ### SHITOMASI
In the 5th image, the median distance ratio (dist curr / dist prev)  <br>
<img src="images/SHITOMASHI-glass.png" width="1200" height="400" /> <br>
-->