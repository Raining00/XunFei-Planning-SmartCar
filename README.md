# Roborts-Planning-Master4.2
小版本更替为0.x
大版本更替为x.0

#### 介绍
本仓库用存放北京科技大学ROS组讯飞智慧餐厅线下赛代码，持续更新优化

#### 软件架构
软件架构说明

#### 更新说明
##### --4.25 update--
1. 优化了全局算法A*的性能，新增了多中启发函数可供选择，默认采用对角启发函数。在启发函数中加入了tie break，解决了路径摇摆的问题。

2. 解决了局部代价地图不更新前向代价，不接受雷达话题的情况。

3. 优化了AMCL定位算法，里程计与地图之间漂移显著减小。

4. 更改了地盘控制逻辑，删除了速度自动置零的问题，使得控制更加流畅

##### --4.27 update--
1. delete some useless files.
2. changing the work into three parts: Planning Master, Voice Master and Vision Master. This space is only for Planning Master.
3. A protect methed is added in to the project so that I can stop the car by using the keybord when the car is sucked and it has no willing to stop moving. Also, I generally add a behavior tree so that I can choose the behavior mode by keybord. But now the tree only has three mode. Maybe I will update more mode in the coming time.


##### --5.22 update--
1. 新增了全局路径算法,由于算法中碰撞检测并没有写，建议在将为空旷宽敞的地图下使用，将会获得比较好的效果;
2. 新增了A*算法的启发函数，并且预留了配置文件接口供选择，请按照配置文件中具体说明进行配置;
3. 雷达测据频率更新为了4KHz，优化了检测盲区，但实际并没有很好的效果；
4. 降低了TEB计算频率，减少了TEB对于速度控制器造成的干扰，使运动更为平滑；
5. 降低了代价地图更新频率，目前够用，在不影响的前提下为CPU计算资源留有富余;
6. 买了个拖把；经过测试发现车轮和地面过于干净的时候，车辆漂移能力下降；
7. 我新买了几件衣服，感觉不错.



#### --5.29 update--
1. 修改了目前的导航框架，新增了中央任务调度和决策节点，负责统筹在什么时候作出怎样的行为决策。
2. 更改了发车方式，如果需要使用多点导航，请发送 --nav_start-- 话题
3. 如果不想使用多点导航，使用rviz直接发布导航点也可以直接进行导航
4. 以前的导航参数和档位全都失效，需要新的参数和档位
5. 关于中央任务调度节点的更新：
	* 多点导航管理，如果需要新增导航点或者更改泊车点，请在task_config.prototxt文件中进行修改
	* 语音播报管理，根据视觉节点返回的二维码信息，自主选择播报语音种类。
	* 人物模型播报，如果没有收到视觉识别结果或者视觉识别结果存在明显错误，均播报人物模型为。
6. 关于新版框架中导航的调整思路：
	* 如果想要避障，则需要x与y方向速度有着一定的比例，一般需要y方向加速度为x方向的2倍以上
	* 如果想要高稳定性过窄道（即车头朝前通过，则需要将一次规划的距离适当降低）
	* 虽然短的规划距离有着一定的优势和通过率，但是鉴于目前没有对全局路径进行优化，因此较大的跟随力度并不是一个特别好的决策，需要后续对全局路径进行平滑处理。
	* 长的规划距离由于暂未实现HCP，因此无法规避局部极值的问题，暂不使用
	* 本想考虑使用换档的决策，发现一个合适的速度比例和规划距离也能有着不错的效果，因此暂时未启动换档决策（即使中央决策节点已经实现了相应的代码）

### --6.12 update--
1. 重新更改了任务调度的方案，目前采用python脚本文件读取csv文件的方式;
2. 导航挡位重新变化，挡位不同请更改规划距离;
3. 存在问题：① 进入C区遇到障碍物后，极大概率容易出现定位丢失的情况; ② 避障参数存在不完美性，容易出现避障过度或者不够的情况。


### --6.15 update--
 删除了ydlidar中3w多行的html说明文档，精简了工作空间大小.


### -- 6.27 update--
1. 新增了几个稳定的档位;
2. 删除了行驶过程中一些不必要的 Way Points;
3. 开始使用新地图作为导航地图；
4. 更改报数的方式为服务申请，极大的提高了语音播放的稳定性；
5. 修复了以往到达终点可能出现不报数的情况bug。


### -- 7.3 update-- ###
1. 对于amcl的全局定位失效问题进行了有效调整，在进入C区，由于存在未知障碍物，并且地图特征少，非常容易重采样到到错误的粒子，造成全局定位失效，对此，根据对amcl异常值的处理思路，需要做的只是丢弃那些由未建模的实体引起的高似然测量值。
2. 添加了对导航中异常状态的处理，即有时候可能存在不更新下一导航点，或者到达停车区域后，由于定位出现的误差，判定为没有到达终点，从而不进行语音播报的异常情况，由于停车到播报之间容许10s的时间，因此对于time_tolerance的设置也为10s；
3. 更新了辅助停车功能，需要对其进行细化，暂时不在本版本中进行辅助停车的使用，该功能将在后续小版本更新中实装。
4. 新改的定位需要对参数进行一定的整定，目前定位效果没有达到最好情况。


### -- 7.5 update-- ###
1. 优化定位，增加了对于雷达畸变矫正的处理，使用畸变矫正后的雷达数据作为定位输入；
2. 优化了定位的参数，依旧不是最优解，并且根据跑车档位的不同，对定位参数也需要不同的微小调整，一般体现在对于里程计模型噪声的调整上；
3. 划分了档位，目前为两个比赛用速度档位，三套方案（主要针对定位）；
4. 以前定位并没有抛弃，可以根据运行不同的导航启动文件决定使用哪一个定位，是否启用畸变矫正功能。
5. 关于新版本各个不同方案的启动方式，请查看后续的**运行方式**一栏


### 文件结构说明


```
.
├── 2d_lidar_undistortion-master				#激光雷达畸变矫正功能包
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── lidar_undistortion_offline.launch			#仿真启动文件
│   │   └── lidar_undistortion_online.launch			#在线机器人平台启动文件
│   ├── LICENSE
│   ├── package.xml
│   ├── pics
│   │   └── result.png						#矫正效果测试图
│   ├── README.md
│   ├── rviz
│   │   └── lidar_undistortion.rviz
│   └── src
│       └── lidar_undistortion.cpp				#激光雷达矫正算法实现
├── CMakeLists.txt
├── fdilink_ahrs						#IMU固件包
│   ├── CMakeLists.txt
│   ├── include
│   ├── launch
│   ├── package.xml
│   ├── README.md
│   └── src
├── README.en.md
├── README.md
├── roborts_common						#通用依赖包，定义了一些数学算法以及proto文件读取模板
│   ├── CMakeLists.txt
│   ├── cmake_module
│   ├── include
│   ├── math							#自定义数学工具
│   │   ├── CMakeLists.txt
│   │   ├── geometry.h
│   │   └── math.h
│   └── package.xml
├── roborts_costmap						#代价地图相关
│   ├── CMakeLists.txt
│   ├── cmake_module
│   ├── config							# 决策、规划中costmap配置以及costmap各层参数设置
│   │   ├── costmap_parameter_config_for_decision.prototxt	
│   │   ├── costmap_parameter_config_for_global_plan.prototxt	#全局代价地图配置
│   │   ├── costmap_parameter_config_for_local_plan.prototxt	#局部代价地图配置
│   │   ├── detection_layer_config.prototxt			#检测层
│   │   ├── dynamic_obstacle_layer_config.prototxt		#动态障碍层	
│   │   ├── inflation_layer_config_min.prototxt			#局部代价地图膨胀设置
│   │   ├── inflation_layer_config.prototxt			#全局代价地图膨胀设置
│   │   ├── local_static_layer_config.prototxt			#局部静态层参数设置
│   │   ├── obstacle_layer_config.prototxt			#障碍物层参数设置
│   │   └── static_layer_config.prototxt
│   ├── include
│   │   └── costmap
│   ├── package.xml
│   ├── proto
│   └── src
│       ├── costmap_2d.cpp
│       ├── costmap_interface.cpp
│       ├── costmap_layer.cpp
│       ├── costmap_math.cpp
│       ├── detection_layer.cpp				#检测层
│       ├── dynamic_obstacle_layer.cpp			#动态障碍层
│       ├── footprint.cpp
│       ├── friend_layer.cpp				#友方层
│       ├── inflation_layer.cpp
│       ├── layer.cpp
│       ├── layered_costmap.cpp
│       ├── local_static_layer.cpp			# 局部静态层
│       ├── observation_buffer.cpp
│       ├── obstacle_layer.cpp
│       ├── static_layer.cpp
│       └── test_costmap.cpp
├── roborts_decision
│   ├── behavior_tree					#行为树定义
│   │   ├── behavior_node.h
│   │   ├── behavior_state.h
│   │   └── behavior_tree.h
│   ├── blackboard							
│   │   ├── blackboard.h				#决策框架，管理机器人行为
│   │   └── communication.h		 		#多机器人通信（需要多边协同工作时，才使用）
│   ├── CMakeLists.txt
│   ├── cmake_module
│   ├── config
│   │   └── sel_behave.prototxt				#决策配置文件
│   ├── example_behavior
│   │   ├── chase_behavior.h				#任务调度头文件，包括管理规划等一系列控制
│   │   └── goal_behavior.h				#导航行为定义，该文件负责管理多点导航
│   ├── executor					#底盘任务调度模块
│   │   ├── chassis_executor.cpp
│   │   ├── chassis_executor.h
│   │   ├── gimbal_executor.cpp
│   │   └── gimbal_executor.h
│   ├── package.xml
│   ├── proto
│   └── sel_behavior_node.cpp				#实例化机器人行为决策
├── roborts_localization				#定位算法功能包
│   ├── amcl
│   │   ├── amcl_config.h				#amcl参数类
│   │   ├── amcl.cpp					#amcl主要逻辑实现
│   │   ├── amcl.h
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   ├── map						#amcl似然地图相关代码
│   │   ├── particle_filter				#粒子滤波相关算法
│   │   └── sensors					#里程计模型与激光雷达传感器模型
│   ├── CMakeLists.txt
│   ├── cmake_module
│   │   ├── FindEigen3.cmake
│   │   └── FindGlog.cmake
│   ├── config											
│   │   └── localization.yaml				#localization参数配置文件，需要在launch file中load
│   ├── localization_config.h				#localization参数类
│   ├── localization_math.cpp				#模块内通用数学算法
│   ├── localization_math.h
│   ├── localization_node.cpp				#定位主节点和main函数
│   ├── localization_node.h
│   ├── log.h						#Golg Wrapper
│   ├── package.xml
│   └── types.h
├── roborts_msgs					#自定义消息类型
│   ├── action
│   │   ├── GlobalPlanner.action
│   │   └── LocalPlanner.action
│   ├── CMakeLists.txt
│   ├── msg
│   │   ├── AllyPose.msg
│   │   ├── ArmorPos.msg
│   │   ├── ArmorsPos.msg
│   │   ├── BallCollision.msg
│   │   ├── Distance.msg
│   │   ├── DodgeMode.msg
│   │   ├── FusionTarget.msg
│   │   ├── FVector.msg
│   │   ├── GimbalAngle.msg
│   │   ├── GimbalInfo.msg
│   │   ├── GimbalPID.msg
│   │   ├── GimbalRate.msg
│   │   ├── LaserTarget.msg
│   │   ├── ObstacleMsg.msg
│   │   ├── referee_system
│   │   ├── ShooterCmd.msg
│   │   ├── ShootInfo.msg
│   │   ├── ShootState.msg
│   │   ├── TargetInfo.msg
│   │   ├── Target.msg
│   │   └── TwistAccel.msg
│   ├── package.xml
│   └── srv
│       ├── FricWhl.srv
│       ├── GimbalMode.srv
│       └── ShootCmd.srv
├── roborts_planning					#运动规划算法包
│   ├── CMakeLists.txt
│   ├── cmake_module
│   ├── global_planner					#全局路径规划算法
│   │   ├── a_star_planner				A*算法实现
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   ├── global_planner_algorithms.h			#全局路径规划模块内通用算法
│   │   ├── global_planner_base.h			#全局路径规划基类，算法类需要从该基类进行派生
│   │   ├── global_planner_node.cpp			#全局路径规划主节点
│   │   ├── global_planner_node.h
│   │   ├── global_planner_test.cpp
│   │   ├── informed_rrt_star_planner			#启发式快速搜索随机生成树规划算法实现
│   │   └── proto
│   ├── local_planner					#局部路径规划
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   ├── include
│   │   ├── README.md
│   │   ├── src
│   │   └── timed_elastic_band					#TEB规划算法
│   └── package.xml	
├── startup_scripts						#初始化脚本（仅在第一次使用小车时运行）
│   └── initdev_mini.sh
├── ucar_controller						#底盘通信节点功能包
│   ├── CHANGELOG.md
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── driver_params_mini.yaml				#mini版本配置文件
│   │   └── driver_params_xiao.yaml				#xiao版本配置文件
│   ├── include
│   │   └── ucar_controller
│   ├── launch
│   │   ├── base_driver.launch					#启动底盘通信节点
│   │   └── tf_server.launch
│   ├── log_info
│   │   ├── car_Mileage_info.txt
│   │   └── car_Mileage_info.txt.bp
│   ├── package.xml
│   ├── README.md
│   ├── scripts
│   │   ├── performance_test.py
│   │   └── sensor_tf_server.py
│   ├── src
│   │   ├── base_driver.cpp
│   │   └── crc_table.cpp
│   └── srv
│       ├── GetBatteryInfo.srv
│       ├── GetMaxVel.srv
│       ├── GetSensorTF.srv
│       ├── SetLEDMode.srv
│       ├── SetMaxVel.srv
│       └── SetSensorTF.srv
├── ucar_map							#SLAM功能启动文件包
│   ├── cfg
│   │   ├── carto_2d.lua					#cartographer算法配置文件
│   │   ├── localization_2d.lua					#cartographer纯定位配置文件
│   │   └── test_2d.lua	
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── cartographer_start.launch				#cartographer建图启动文件
│   │   ├── gmapping_demo.launch				#gmapping建图启动文件
│   │   ├── gmapping.launch
│   │   └── ucar_mapping.launch
│   ├── maps							#占据式栅格地图实例
│   │   ├── 123.pgm
│   │   ├── 123.yaml
│   │   ├── ucar_map_001.pgm
│   │   └── ucar_map_001.yaml
│   └── package.xml
├── ucar_nav							#导航启动管理
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── amcl_7.launch
│   │   ├── amcl.launch				
│   │   ├── Obviously.launch			
│   │   └── pedestrain.launch
│   ├── maps							#导航地图存放处
│   │   ├── 0607.pgm
│   │   ├── 0607.yaml
│   │   ├── map1.pgm
│   │   ├── map1.yaml
│   │   ├── map20210519.pgm
│   │   ├── map20210519.yaml
│   │   ├── map2021513.pgm
│   │   ├── map2021513.yaml
│   │   ├── map2021616.pgm
│   │   ├── map2021616.yaml
│   │   ├── map2021629.pgm
│   │   ├── map2021629.yaml
│   │   ├── map2.png
│   │   ├── map3.pgm
│   │   ├── map3.yaml
│   │   ├── map_418_002.pgm
│   │   ├── map_418_002.yaml
│   │   ├── map4.pgm
│   │   ├── map4.yaml
│   │   ├── map629.pgm
│   │   ├── map629.yaml
│   │   ├── mapps.png
│   │   ├── map_ps.yaml
│   │   ├── new_map.png
│   │   ├── new_map.yaml
│   │   ├── test.pgm
│   │   ├── test.yaml
│   │   ├── ucar_map_001.pgm
│   │   └── ucar_map_001.yaml
│   ├── package.xml
│   ├── param
│   │   └── ekf_param
│   └── scripts						#比赛任务调度和管理脚本
│       ├── 4.py
│       ├── assist_parking.py
│       ├── mission_task_amcl.py
│       ├── mission_task.py
│       ├── mission_task_quanguo.py
│       ├── parking.csv
│       ├── singal.csv
│       ├── test2.csv
│       ├── test3.csv
│       └── test.csv
├── xunfei_params					#参数配置保存
└── ydlidar						#雷达固件
    ├── CMakeLists.txt
    ├── launch
    │   ├── display.launch
    │   ├── gazebo.launch
    │   ├── lidar.launch
    │   ├── lidar.rviz
    │   ├── lidar_view.launch	
    │   ├── ydlidar_distoration.launch			#启动畸变矫正
    │   └── ydlidar.launch				#不使用畸变矫正
    ├── LICENSE
    ├── meshes
    │   ├── ydlidar.dae
    │   └── ydlidar.png
    ├── package.xml
    ├── README.md
    ├── sdk
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── license
    │   ├── README.md
    │   ├── samples
    │   └── src
    ├── src
    │   ├── ydlidar_client.cpp
    │   └── ydlidar_node.cpp
    ├── urdf
    │   └── ydlidar.urdf
    └── ydlidar.rviz
```

#### 安装教程

### **依赖工具及环境**

1. NVIDIA JETSON NANO计算平台

2. Ubuntu 18.04

3. Ros melodic平台

4. 安装ROS所需第三方依赖包，以及‘SuitSparse’，‘Glog’，‘protbuf’等其他依赖。

```bash
sudo apt-get install -y ros-melodic-cv-bridge                         \
						ros-melodic-image-transport        \
						ros-melodic-map-server                  \
						ros-melodic-laser-geometry          \
						ros-melodic-interactive-markers \
						ros-melodic-tf                                       \
						ros-melodic-pcl-*                                \
						ros-melodic-libg2o                             \
						ros-melodic-rplidar-ros                    \
						ros-melodic-robot-localization                    \
						protobuf-compiler                              \
						libprotobuf-dev                                   \ 
						libsuitesparse-dev                              \
						libgoogle-glog-dev                             \
						libzmq3-dev
```
其它软件
- cmake
- vim
- terminator
- htop

### **编译及安装方式**

由于Jestson Nano性能的影响，在编译的时候可能会出现low memory的警告最终导致CMake进程被杀死，解决方法如下：

	参考博客：https://www.jianshu.com/p/c4ef42f6b2ec

解决：增大交换空间的大小，临时增加了2G交换空间

	sudo dd if=/dev/zero of=/swapfile bs=64M count=32       #根据自身nano上emmc剩余的空间也可以将count大小设置为24

	#count的大小就是增加的swap空间的大小，64M是块大小，所以空间大小是bs*count=2048MB

	sudo mkswap /swapfile

	#把刚才空间格式化成swap格式

	sudo chmod 0600 /swapfile 

	sudo swapon /swapfile

	#使用刚才创建的swap空间


释放空间命令：

	swapoff -a

关于编译时出现找不到头文件：

1. 如果是提示消息类型找不到，请确认是否是本工程下的自定义消息，如果是，请多尝试几次CMake即可生成相应文件;

2. 如果提示zmp.hpp找不到，请打开终端运行以下命令：

	sudo apt-get install libzmq3-dev

3. 如果不是以上两种情况，请根据前述环境依赖提示的功能完成相应安装。


### **运行方式**
初始化工作空间：
	source devel/setup.sh

进入 ***xunfei_params*** 文件夹，选择想要的档位，直接运行sh文件，例：

	./1.sh

新开终端，进入 **ucar_nav/scripts** 文件夹下，先source视觉的工作空间，再运行如下命令：

	python2 mission_task.py
	
**注意** 对于本次更新后的定位功能，上述启动方式不可用，新版本启动方式请以此执行：

	source devel/setup.sh
	python2 mission_task_amcl.py
	新开终端，进入 **ucar_nav/scripts** 文件夹下：
	source ~/vision_ws/devel/setup.sh

如果想要启动畸变矫正功能，请修改雷达启动luanch文件为：

	ydlidar_distoration.launch

使用老版本定位请确保运行导航文件为 --pedestrian.launch--

使用新版本定位请确保运行的导航文件为： --Obviously.launch--

也可以直接进入 --xunfei_params-- 文件夹中，启动不同档位，直接运行相应的shell脚本即可,例如：
	./Ca.sh

如有任何问题，请通过邮箱联系我：

邮箱： xyuan1517@gmail.com



