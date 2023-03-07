**结构总览**

* node：\
  kukafri_hw
* topic：\
  /curJoint ——publish current joints position （°）\
  /curPos ——publish current end-effector pose （m、°）\
  /cmdJoint ——subscribe joints degree command （°）\
  /cmdPosEulerZYX ——subscribe end-effector pose command （m、°）\
  /cmdPosQuaternion ——subscribe end-effector pose command （m、°）
* server：\
  /MoveToHome ——将机械臂运动到控制前准备姿态\
  /setMoveMode ——设置 插值方式（一阶/阶跃/sin），运动类型（关节角度/末端位姿），运动时间（完成一次指令的时间/s)

**使用方式**

1. 安装：\
   放工作空间里编译就行
2. 配置：\
   需要配置IP，参照如下信息。端口是在程序中使用的，不用管。`左右臂使用同一子网时ping不通，故用了不同网段`：

   ```
   左臂KONI 192.168.1.1
   左臂X66 172.31.1.147
   左臂上位机 192.168.1.2
   左臂端口 30201
   
   右臂KONI 192.168.168.2
   右臂X66 172.31.1.147
   右臂上位机 192.168.168.10
   右臂端口 30200
   
   
   ```
3. 运行：\
   运行`kukafri_hw.launch`文件,后面跟上参数side选择启动哪根机械臂: `dual-双臂，right-右臂，left-左臂，默认为右臂`；参数sim选择仿真与否：`1-仿真，0-真机，默认为1仿真`\
   eg: `roslaunch kukafri_hw kukafri_hw.launch side:=dual sim:=1`\
   `kukafri_hw.roslaunch`文件中有几个重要的参数需要注意：\
   \- `node`标签中的`args`参数：为传入`main`函数的`argv`，含义分别为IP地址、端口号、机械臂名称
4. 自己的相机坐标系作为基坐标系的指定方式\
   在launch文件中，通过rosparam参数服务器，加载参数文件`.yaml`，从而通过相机标定得到的参数，将左右臂的末端位姿转换到相机坐标系下。\
   参数文件`.yaml`中指定的参数的含义：相机标定基于左臂还是右臂的link_0；相机标定得到的结果。\
   `cameralsy.yaml`及`camerahxz.yaml`为现有的两个相机的标定参数。如需新增，可仿照着新建参数文件。
5. home位姿的调整\
   `homejoints.yaml`中设置了左臂和右臂的关节角，在准备使用机械臂之前，通过/MoveToHome服务将机械臂运动到这个位置\
   如需使用别的home位姿，可在该文件中进行修改，或仿照新建别的参数文件，并在launch文件中load
6. /setMoveMode介绍\
   该server中的request有三个量：\
   \- moveMode：设置机械臂运动模式，0-关节角度，1-末端位姿。节点启动时默认为0\
   \- pathMode：设置路径插值方式。关节角度运动模式时：0-一次插值（匀速运动），1-阶跃插值（不插值），2-sin插值（速度为sin）。\
   末端位姿运动模式时：只有一次差值（匀速，走直线）。节点启动时默认为0\
   \- moveDuration：设置一次运动完成的时间，注意赋值大于0的值。启动节点时默认为10\
   response为true或false
7. /MoveToHome介绍\
   该server中的request没用，就让它保持默认值0就行，随便给也没关系。\
   response为true或false，但是response不代表运动完成与否，程序中使用时,暂时先使用sleep(moveDuration)的方法来等待机械臂运动到home姿态

**！！注意事项！！** 重要

1. /MoveToHome server的使用：call这个server前须确保运动模式为关节角度运动(/setMoveMode/moveMode=0)，且此运动耗时也由现有的/setMoveMode/duration决定；考虑一般使用情况， 是在开机后自己的控制程序启动前call一下这个server，故能满足基本的要求。
2. 由于未设置碰撞检测等安全措施，故使用时注意安全，速度设置慢点。
3. 若要使用仿真，请确保`iiwa_gazebo`是能被ros找到的（即source了`iiwa_gazebo`所在的工作空间）。`kuka_ws`项目下的`iiwa_gazebo.launch`有更新，请注意。