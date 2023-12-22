# 招人啦！！！

Hello，小伙伴们！

我目前在蔚来汽车[NIO]自动驾驶部门工作，负责多传感器融合定位、SLAM等方面的研发工作。我们正在寻找新的同学，一起探索自动驾驶技术的量产落地。

目前，我们团队参与研发的智能驾驶功能已经在多个场景实现量产落地。高速城快领航辅助驾驶功能于2022年发布，累积服务里程超过1亿公里[截止10月]。今年蔚来开始向用户推送了技术难度更大的城区领航功能，近期正在通过群体智能方式不断拓展可用范围。蔚来独有的高速服务区领航体验也在今年11月份进行了发布，实现了高速到服务区换电场景的全流程自动化和全程领航体验。除了上面这些高阶自动驾驶功能，像AEB、LCC等背后也都有我们团队的身影。更多功能场景的发布，敬请期待。

**[全域领航辅助｜超3倍完成年度目标，提速规划节奏](app.nio.com/app/community_content_h5/module_10050/content?id=531584&type=article&is_nav_show=false&wv=lg)**

**现在，我们组正在寻找计算机视觉、Learning、SLAM、多传感器融合等技术背景的同学加入，全职和实习均可以，欢迎来聊。Email: ydsf16@163.com, 微信: YDSF16**

![image](https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/20231223-004022.jpeg){:width="300px" height="200px"}


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
## FilterFusion : Filter-Based Sensor Fusion. 
Fuse wheel, visual, and GNSS in an Extended Kalman Filter.

For visual-wheel fusion, please refer to: https://zhuanlan.zhihu.com/p/270670373

For fusing of GNSS data, please refer to: https://zhuanlan.zhihu.com/p/330880853

You can select the sensors to participate in the fusion through the configuration file.
```
sys_config.enable_plane_update: 1
sys_config.enable_gps_update: 1
```

![image](https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/Visual-Wheel-GNSS-Localization.png)
![image](https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/VWO-MSCKF.png)
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

# Just for JYYJ - w.

