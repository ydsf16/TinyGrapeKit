# TinyGrapeKit
## A bunch of state estimation algorithms.
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
## FilterFusion : Filter-Base Sensor Fusion. 
Fusion wheel, visual, and GNss in an Extended Kalman Filter.

For details, please refer to: https://zhuanlan.zhihu.com/p/270670373 and https://zhuanlan.zhihu.com/p/330880853

![image](https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/KAIST.png)
![image](https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/SIM.png)

### Dataset 
We used the KAIST dataset to test our method. https://irap.kaist.ac.kr/dataset/

### Example
For examples, please refer to the **Example** folder.
```
./RunKAISTData ${REPO_PATH}/TinyGrapeKit/app/FilterFusion/params/KAIST.yaml ${KAIST_PATH}
```

# Contact us
For any issues, please feel free to contact **[Dongsheng Yang](https://github.com/ydsf16)**: <ydsf16@buaa.edu.cn>, <ydsf16@163.com>
