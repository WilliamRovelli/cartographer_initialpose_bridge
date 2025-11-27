# cartographer_initialpose_bridge 使用说明
## 说明 
由于agv在运行时存在丢失定位的情况，且FCCSM有范围限制，因此需要一个手动重定位的功能。cartographer本身缺少手动重定位的功能，但是内部有初始化接口。
本程序的主要功能是利用cartographer的接口实现在rviz中为agv恢复重定位。
地图pbstream格式根据“操作步骤”提示操作即可。
非pbstream格式，需要自己载入到rviz中，然后执行roslaunch abtr_initial_pose abtr_initial_pose.launch,再根据“操作步骤”的step4操作即可。
## 操作步骤 
step1:
当前目录catkin_make。

step2：
1，将pbstream格式的地图放入src/abtr_initial_pose/run目录下（如果有特定的rviz需要打开也可以将rviz放到这个目录下，默认使用abtr_rviz.rviz）
2，在abtr_initial_pose.sh中填入无后缀地图名称，比如（map1.pbstream，只需要填map1），如有rviz，则填入无后缀rviz名称。
3，填入cartographer的绝对路径

step3：
chmod +x abtr_initial_pose.sh （首次使用用一次）
./abtr_initial_pose.sh

step4:
点击rviz的 2D Pose Estimate 根据雷达点云在物理世界的位置拖拽雷达点云到地图对应的物理世界的大概位置。等一下会儿算法会自动触发分支定界，然后准确配准。

