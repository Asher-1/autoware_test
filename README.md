# autoware.test
autoware.test基于autoware.ai的v1.13.0版本，本人日常开发和维护的环境为ubuntu18.04。

# 分支介绍：
main 源代码分支，同课程内容中的源代码讲解一致，随着课程的推进持续更新中;

demo_dataset 数据集分支，包括点云地图，矢量地图，配置文件等相应对应不同模拟场景的demo数据集;

homework 分支，包含每个章节项目练习的原始数据，参考代码等等;

tools 工具分支，一些可能用到的小工具;

# 仿真环境部署

准备工作：

[网盘链接](https://pan.baidu.com/s/1zJA13SP6fFi3lmb_Q9xxlA?pwd=3w8a]), 提取码: 3w8a

1、将网盘链接里的gazebo模型库中"gazebo_models.zip"下载下来并解压；

2、将解压得到的"models"文件夹放在"/home/用户名/.gazebo"下,replace原有；

3、"./gazebo"为一个隐藏文件夹，如果没有说明没有运行过gazebo，运行一次后会自动生成；

4、将网盘中课程资料里的gazebo模型库中"actor_collisions.zip"下载并解压；

5、cd actor_collisions;mkdir build;cd build;cmake ..;make;

6、将生成的"libActorCollisionsPlugin.so"放入/usr/lib/x86_64-linux-gnu/gazebo-9/plugins/

7、pull本repo下的demo_dataset分支最新版，并更新到".autoware"文件夹；

# 仿真1：简化版的仿真启动

仿真1为简易的仿真环境，车辆静止，主要便于大家进行感知模块的仿真,操作步骤如下：

依次启动如下文件：

1、roslaunch autoware_quickstart_examples mini_map.launch；

2、roslaunch autoware_quickstart_examples mini_localization.launch;

3、rviz;

4、手动给定一个初始位姿；

5、roslaunch autoware_quickstart_examples mini_sil_env.launch（多等一会）;

6、roslaunch autoware_quickstart_examples mini_detection.launch

# 仿真2：完整版的仿真启动

仿真2为完整的仿真环境，后面讲解的规划控制模块都是基于它,操作步骤如下：

依次启动如下文件：

1、roslaunch autoware_quickstart_examples new_map.launch；

2、roslaunch autoware_quickstart_examples new_localization.launch;

3、rviz;

4、手动给定一个初始位姿；

5、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_a.launch(多等一会，3-5mins都有可能)

6、roslaunch vehicle_gazebo_simulation_launcher world_test_citysim_b.launch(等到前面的gazebo world启动成功且定位成功后再启动)

7、roslaunch autoware_quickstart_examples new_detection.launch

决策规划方式1：基于vector map的lane来进行寻轨，遇到障碍物停止，不规划避让

8、roslaunch autoware_quickstart_examples new_mission_planning.launch;

9、roslaunch autoware_quickstart_examples new_motion_planning.launch;

决策规划方式2：基于vector map的lane来进行寻轨，基于astar avoid来进行规划避让 

8、roslaunch costmap_generator costmap_generator.launch;

9、roslaunch autoware_quickstart_examples new_mission_planning.launch;

10、roslaunch autoware_quickstart_examples new_avoid_motion_planning.launch;

决策规划方式3：用astar算法来进行规划避让, 手动选择目的地 

8、roslaunch autoware_quickstart_examples new_manual_astar_planner.launch;

9、roslaunch autoware_quickstart_examples new_motion_planning.launch;

决策规划方式4：基于vector map的lane来进行寻轨，基于op-planner来进行规划避让 

8、roslaunch autoware_quickstart_examples new_op_planner.launch;

9、roslaunch autoware_quickstart_examples new_motion_planning.launch;

mpc控制：

9、roslaunch autoware_quickstart_examples new_mpc_planning.launch;
# autoware.ai环境配置

安装Ubuntu 18.04（建议使用双系统，虚拟机会很卡）

安装ROS Melodic（可以使用鱼香ROS，一键自动安装）

```shell
wget http://fishros.com/install -O fishros && . fishros
```

------

安装Ubuntu/ROS系统依赖

```shell
sudo apt update
sudo apt install python3-pip
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools
```

安装CUDA 10.0，参考：https://developer.nvidia.com/cuda-10.0-download-archive?target_os=Linux&amp;target_arch=x86_64&https://github.com/Autoware-AI/autoware.ai/wiki/Source-Buildamp;target_distro=Ubuntu&amp;target_version=1804&amp;target_type=runfilelocal 



使能CUDA，安装（或更新）Eigen为3.3.7版本（可能会造成系统兼容性问题），参考：https://blog.csdn.net/reasonyuanrobot/article/details/114372363



创建工作空间

```shell
mkdir -p autoware.ai
```

如果能够"正常"上网，可以参考：https://github.com/Autoware-AI/autoware.ai/wiki/Source-Build


如果因为网络问题导致下载失败，可以采用如下方法：

```
git clone -b main https://github.com/Asher-1/autoware_test.git
```

**克隆得到的autoware.test里含有src文件夹，将src文件夹整个复制到autoware.ai文件夹中，便可进行下一步操作。**

使用rosdep安装依赖。

```shell
rosdep update
```

如果不能"正常"上网，这一步一般会出现问题。因此，可以使用鱼香ROS进行一键rosdep。

```shell
wget http://fishros.com/install -O fishros && . fishros
rosdepc update
cd autoware.ai
rosdepc install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

编译工作空间（带CUDA）(if failed, please try again!)
```shell
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error
```

编译更新特定的package
```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select [package_name]
```

# 文件加载
编译完成后，运行demo前，需要配置一下文件加载目录，默认的目录为在将“demo_dataset”分支的data文件夹
clone下来后，将其放在home路径下建立的".autoware"文件夹中。


TODO。。。。 持续更新中。。。。。。。

