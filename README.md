# 招人啦！！！

嗨，大家好！

我是蔚来汽车[NIO]自动驾驶团队的一员，负责多传感器融合定位、SLAM等领域的研发工作。目前，我们正在寻找新的队友加入我们。

我们团队一直在推动智能驾驶功能的研发，已成功在多个场景实现量产。例如，我们的高速城快领航辅助驾驶功能于2022年发布，截至2023年10月，已在累积服务里程超过1亿公里。今年，我们还推出了技术更为复杂的城区领航功能，并通过群体智能不断拓展其可用范围。同时，我们还在11月份发布了独特的高速服务区领航体验，实现了高速到服务区换电场景的全流程自动化和全程领航体验。此外，我们团队也参与了一些基础功能背后的研发，如AEB、LCC等。未来还有更多令人期待的功能发发布，敬请期待。

如果你对计算机视觉、深度学习、SLAM、多传感器融合、组合惯导等技术有着扎实的背景，不论是全职还是实习，我们都欢迎你加入我们的团队。有兴趣的话，可以通过微信联系我们：YDSF16。期待与你共同探索智能驾驶领域的未来！

**[全域领航辅助｜超3倍完成年度目标，提速规划节奏](app.nio.com/app/community_content_h5/module_10050/content?id=531584&type=article&is_nav_show=false&wv=lg)**

<a href="https://youtu.be/3A5wpWgrHTI" target="_blank"><img src="https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/20231223-004022.jpeg" 
alt="YDS" width="300" height="300"/></a>

NIO社招内推码: B89PQMZ 
投递链接: https://nio.jobs.feishu.cn/referral/m/position/detail/?token=MTsxNzAzMjY0NzE2NTYyOzY5ODI0NTE1OTI5OTgxOTI2NDg7NzI2MDc4NjA0ODI2Mjk2NTU0MQ

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

