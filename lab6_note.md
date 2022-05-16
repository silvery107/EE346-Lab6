1. Change the `.yaml` name
    `~/. ros/camera_info/camera_name.yaml`

2. Find the homography

    ```python
    top_x = 42 # 47
    top_y = 44 # 38
    bottom_x = 145
    bottom_y = 115        

    # selecting 4 points from the original image
    pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y], [160 - bottom_x, 120 + bottom_y]])

    # selecting 4 points from image that will be transformed
    pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

    # finding homography matrix
    h, status = cv2.findHomography(pts_src, pts_dst)

    ```
3. Change HSV thresholds
    > Black [0, 0, 0] -> [180, 255, 46]

4. Merge two mask detections, or devide one frame into left and right parts

5. Change max velocities
    linear x: `0.22`, anguler z: `2.84`. May choose a bigger value.

6. Modify stop logic
    May fuse odom and image again
    <!-- TODO -->
    这里需要考虑结束后完全停止

7. Find a way to detect intersections
    <!-- TODO -->
    维护左右边线数组，右边线丢线->补线->右边线不丢线->过路口

    将右转写成固定的控制序列
    
    维护路口计数器，共4次，右转、直走、直走、右转+开启ArUco识别和停车逻辑
