# TinyGrapeKit
## A repositorie for state estimation.
This repo is divided into two parts, one is the basic algorithm, in the **library** folder. The other is the actual multi-sensor fusion algorithm (e.g. SLAM), in the **app** folder.

Detailed derivations can be found in: https://www.zhihu.com/column/slamTech

# Install
## Prerequisites
opencv, ceres, Eigen

## Build 
```
chmod +x build.sh
./build.sh
```

# Applications in the **app** folder
## VWO-MSCKF : MSCKF Based Visual Wheel Odometry. 
An odometry algorithm by fusing visual and wheel information in an Extended Kalman Filter.

For details, please refer to: TODO

![image](https://github.com/ydsf16/imu_gps_localization/blob/master/doc/path.png)

We used the KAIST dataset to test our method. https://irap.kaist.ac.kr/dataset/


# Contact us
For any issues, please feel free to contact **[Dongsheng Yang](https://github.com/ydsf16)**: <ydsf16@buaa.edu.cn>, <ydsf16@163.com>
