## 引言
 在这个-SLAM建图和导航仿真实例-项目中，主要分为三个部分，分别是
 - [（一）模型构建](https://blog.csdn.net/qq_37266917/article/details/106317862)
 - [（二）根据已知地图进行定位和导航](https://blog.csdn.net/qq_37266917/article/details/106415436)
 - [（三）使用RTAB-MAP进行建图和导航](https://blog.csdn.net/qq_37266917/article/details/106416213)

该项目的slam_bot已经上传我的[Github](https://github.com/Xiangyu-Fu/slam_bot)。

由于之前的虚拟机性能限制，我在这个项目中使用了新的ubantu 16.04环境，虚拟机配置
- 内存 8G
- CPU 四核
- **禁用硬件加速**（重要：否则gazebo会打开失败）
## 一、模型构建
### 1、使用Solidworks建立slam_bot包
使用Solidworks建立模型如下图，其中有很多部分值得注意。

1. 定义底盘底面为base_link坐标系的xoy平面，底面中点为坐标原点，小车正方向为x轴正方向建立右手系；
2. 各个车轮坐标系原点为各个车轮内侧平面圆心，x轴与base_link坐标系x轴对齐，y轴与车轴平齐（与base_link坐标系y轴对齐或相向）建立右手系；
3. 车轮的旋转轴分别为对应的转轴轴线。

值得注意的是，在下图中有两个坐标系，分别是**坐标系1**和**coordinate0**。在一开始，我没有意识到使用coordinate0作为坐标原点是一个多么错误的选择。在后来使用`skid_steer_drive_controller`gazebo插件时，模型一直在原地转圈，直到我在rviz中查看tf转换关系时，才发现是错误的tf转换，导致了插件的控制错误。所以一定要严格遵循上述所说的坐标系建立规则。
![urdf](https://img-blog.csdnimg.cn/20200528205207290.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)<center><font face="楷体">图1-1 solidworks模型</font></center>


使用solidworks的urdf_exporter插件导出urdf和模型文件。在图1-2中base_link忘记选择参考坐标系了，若以上图坐标系为准，则需要选择 - **坐标系1**。
> 关于更多urdf_exporter安装和使用可以参考我的博客-[【从零开始的ROS四轴机械臂控制】（一）- 实际模型制作、Solidworks文件转urdf与rviz仿真.](https://editor.csdn.net/md/?articleId=104651696)。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200528210745530.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center =500x)
<center><font face="楷体">图1-2 urdf-exporter导出界面</font></center>

若选择无误，就可以导出模型了。
> 我在Github中提供了最初始的[slam_bot package文件](https://github.com/Xiangyu-Fu/slam_bot/tree/master/slam_bot_initial)。

### 2.更改slam_bot包
#### （1）urdf文件夹
导航到urdf文件夹
```
$ ~/catkin_ws/src/slam_bot/urdf
```
将`slam_bot.urdf` 更名成`slam_bot.xacro`，并新建`slam_bot.gazebo.xacro`文件。

##### 更改slam_bot.xacro

添加xacro namespace。
 
```xml
<robot  name="slam_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
```
添加slam_bot.gazebo.xacro，导入gazebo插件
```xml
  <xacro:include filename="$(find slam_bot)/urdf/slam_bot.gazebo.xacro" />
```

在base_link节点之前添加base_footprint节点
```xml
  <!--base_footprint link-->
  <link name="base_footprint">  </link>

  <joint name="base_link_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="base_footprint"/>
    <child link="base_link" />
  </joint>
```
在camera节点之后添加laser节点

```xml
  <!--laser_link-->
  <link name="hokuyo">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://slam_bot/meshes/hokuyo.dae"/>
      </geometry>
    </collision>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://slam_bot/meshes/hokuyo.dae"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0 0 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 1.57 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>

  <joint name="hokuyo_joint" type="fixed">
    <axis xyz="0 0 0" />
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="hokuyo"/>
  </joint>
```

更改各个joint设置，如joint_wheel_FR。注意`<axis>`的设置，否则也会导致错误的控制。
```xml
  <joint
    name="joint_wheel_FR"
    type="continuous">
    <origin
      xyz="0.0800000000000013 -0.0799999999999997 0.04"
      rpy="0 0 0" />
    <parent
      link="base_link" />
    <child
      link="link_wheel_FR" />
    <axis
      xyz="0 1 0" />
  </joint>
```
joint type设置为“continuous”，类似于转动关节，但对其旋转没有限制。它可以绕一个轴连续旋转。关节会有自己的旋转轴`axis `，一些特定的关节动力学`dynamics `与关节的物理特性(如“摩擦”)相对应，以及对该关节施加最大“努力”和“速度”的某些限制`limits `。这些限制对于物理机器人是有用的约束，并且可以帮助在仿真中创建更健壮的机器人模型。

>点击[这里](http://wiki.ros.org/pr2_controller_manager/safety_limits)来更好地理解这些限制。


RGB-D点云是默认朝上,需向slam_bot.xacro中添加如下的link和joint
```xml
  <link name="camera_depth_optical_frame" />

  <joint name="camera_depth_optical_joint" type="fixed">
    <origin rpy="-1.57079632679 0 -1.57079632679" xyz="0 0 0" />
    <parent link="camera"/>
    <child link="camera_depth_optical_frame" />
  </joint>
```
##### 更改slam_bot.gazebo.xacro
添加如下插件
- 四轮机器人控制驱动：`skid_steer_drive_controller`
- RGBD 深度摄像机:`libgazebo_ros_openni_kinect`
-  激光传感器: `head_hokuyo_sensor`

```xml
<?xml version="1.0"?>
<robot>

  <gazebo>
    <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">
	<alwaysOn>true</alwaysOn>
        <updateRate>100.0</updateRate>
        <robotNamespace>/</robotNamespace>
        <leftFrontJoint>joint_wheel_FL</leftFrontJoint>
        <rightFrontJoint>joint_wheel_FR</rightFrontJoint>
        <leftRearJoint>joint_wheel_BL</leftRearJoint>
        <rightRearJoint>joint_wheel_BR</rightRearJoint>
        <wheelSeparation>0.16</wheelSeparation>
        <wheelDiameter>0.08</wheelDiameter>
        <robotBaseFrame>base_footprint</robotBaseFrame>
        <torque>20</torque>
        <commandTopic>cmd_vel</commandTopic>
    	<odometryTopic>odom</odometryTopic>
    	<odometryFrame>odom</odometryFrame>
    	<broadcastTF>1</broadcastTF>
    </plugin>
  </gazebo>

  <!--RGBD camera -->
  <gazebo reference="camera">
   <sensor type="depth" name="camera">
     <always_on>true</always_on>
     <visualize>false</visualize>
     <update_rate>15.0</update_rate>
     <camera name="front">
       <horizontal_fov>1.047197</horizontal_fov>
       <image>
         <!-- openni_kinect plugin works only with BGR8 -->
         <format>B8G8R8</format>
         <width>400</width>
         <height>300</height>
       </image>
       <clip>
         <near>0.01</near>
         <far>8</far>
       </clip>
     </camera>
     <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
       <baseline>0.1</baseline>
       <alwaysOn>true</alwaysOn>
       <updateRate>15.0</updateRate>
       <cameraName>camera</cameraName>
       <imageTopicName>/camera/rgb/image_raw</imageTopicName>
       <cameraInfoTopicName>/camera/rgb/camera_info</cameraInfoTopicName>
       <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
       <depthImageCameraInfoTopicName>/camera/depth_registered/camera_info</depthImageCameraInfoTopicName>
       <pointCloudTopicName>/camera/depth_registered/points</pointCloudTopicName>
       <frameName>camera_depth_optical_frame</frameName>
       <pointCloudCutoff>0.35</pointCloudCutoff>
       <pointCloudCutoffMax>4.5</pointCloudCutoffMax>
       <CxPrime>0</CxPrime>
       <Cx>0</Cx>
       <Cy>0</Cy>
       <focalLength>0</focalLength>
       <hackBaseline>0</hackBaseline>
     </plugin>
   </sensor>
  </gazebo>

 <!-- hokuyo -->
  <gazebo reference="hokuyo">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for Hokuyo laser
               achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
               stddev of 0.01m will put 99.7% of samples within 0.03m of the true
               reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
        <topicName>/slam_bot/laser/scan</topicName>
        <frameName>hokuyo</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

#### （2）worlds文件夹
在worlds中储存gazebo world。world是模型的集合，还可以定义此world特定的其他几个物理属性。

让我们创建一个简单的世界，其中没有将在以后的gazebo中启动的对象或模型。
```
$ cd worlds
$ nano slam.world
```
将以下内容添加到 slam.world
```xml
<?xml version="1.0" ?>

<sdf version="1.4">

  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen='0'>
      <camera name='world_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```
`.world` 文件使用 XML 文件格式来描述所有被定义为 Gazeboeboard 环境的元素。在上面创建的简单世界有以下元素

`<sdf>`: 基本元素，封装了整个文件结构和内容。
`<world>`：世界元素定义了世界描述和与世界相关的几个属性。每个模型或属性可以有更多的元素来描述它。例如，`camera`有一个`pose`元素，定义了它的位置和方向。
`<include>`：include元素和`<uri>`元素一起，提供了通往特定模型的路径。在Gazebo中，有几个模型是默认包含的，可以在创建环境时包含它们。

#### （3）launch文件夹

创建一个新的启动文件，这将有助于加载URDF文件。
```
$ cd ~/catkin_ws/src/slam/launch/
$ nano robot_description.launch
```
将以下内容复制到上述文件中。
```xml
<launch>

  <!-- send urdf to param server -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find slam_bot)/urdf/slam_bot.xacro'" />

  <!-- Send fake joint values-->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>

  <!-- Send robot states to tf -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>

</launch>

```
上面的代码定义了一个参数robot_description，该参数用于设置单个命令，以使用xacro软件包从xacro文件生成URDF。

借助robot_description生成的URDF文件，gazebo_ros包生成对应模型。

接下来创建一个launch文件。ROS中的启动文件使我们可以同时执行多个节点，这有助于避免在单独的shell或终端中定义和启动多个节点的繁琐工作。
```
$ nano slam.launch
```
将以下内容添加到启动文件中。
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <include file="$(find slam_bot)/launch/robot_description.xml"/>

  <arg name="world" default="empty"/> 
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find slam_bot)/worlds/kitchen_dining.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  <!--spawn a robot in gazebo world-->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" 
     output="screen" args="-urdf -param robot_description -model slam_bot"/>

  <!--launch rviz-->
  <node name="rviz" pkg="rviz" type="rviz" respawn="false"/>

</launch>

```
和`.world`文件一样，`.launch`文件也是基于XML的。

首先，使用 `<arg> `元素定义某些参数。每个这样的元素都会有一个`name`属性和一个`default`值。
然后，在`gazebo_ros` 包中包含了`empty_world.launch `文件。[empty_world](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/launch/empty_world.launch)文件包含了一组重要的定义，这些定义被我们创建的world所继承。使用 `world_name` 参数和传递给该参数的`value`的` .world `文件的路径，所以我们能够在 Gazebo 中启动world。

### 3.运行slam_bot

现在可以使用启动文件来启动Gazebo环境。
```
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch slam slam.launch
```

若gazebo长时间打不开，解决办法如下
```
cd ~/
hg clone https://bitbucket.org/osrf/gazebo_models
```
下载完成后将gazebo_models复制到`~/.gazebo`文件夹中，重命名为`models`。

若模型正常运行，则如图3-1所示。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200528231629838.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70)

<center><font face="楷体">图3-1 模型在gazebo中运行</font></center>

#### （1）测试RGBD相机和激光雷达
这次，凉亭和RViz都应该启动。加载后

选择RViz窗口，然后在左侧的Displays中：

- 选择“ odom”作为fixed frame

点击“Add”按钮，然后
- 添加“ RobotModel”
- 添加“pointcloud2”并选择在gazebo插件中定义的/camera/depth_registered/points主题
- 添加“ LaserScan”并选择在gazebo插件中定义的hokuyo主题。

机器人模型应在RViz中加载。

在凉亭中，单击“插入”，然后从列表中添加机器人前面世界中的任何物品。

至此应该能够在Rviz中的“pointcloud2”查看器中看到该项目，并且也可以对该对象进行激光扫描。


![在这里插入图片描述](https://img-blog.csdnimg.cn/20200528223856898.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)
<center><font face="楷体">图3-2 测试RGBD相机和激光雷达</font></center>

图3-2来自较旧版本的模型，所以看起来与之前模型会有所不同。


#### （2）测试四轮驱动插件

在上述所有内容仍在运行时，打开一个新的终端窗口，然后输入
```
$ rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

上面的命令会将消息发布到cmd_vel，这是在驱动器控制器插件中定义的主题。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200528231441347.gif#pic_center)

<center><font face="楷体">图3-3 测试四轮驱动插件</font></center>

可以看到模型正常移动，gazebo因为录屏软件奔溃了，不过正常运行是不会这样的。

模型应当沿x轴正方向移动，如果出现运动不正常的现象，可以检查
- `skid_steer_drive_controller`定义
- urdf中各个joint定义
- 检查tf变换，使用`rosrun tf view_frames`以查看tf信息


![在这里插入图片描述](https://img-blog.csdnimg.cn/2020052823251142.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70)
<center><font face="楷体">图3-4 slam_bot的tf树示例</font></center>

创建和查看tf-tree是确保所有链接顺序正确的好方法。

也可以在RViz中绘制不同的框架并在那里进行图形上的检查。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200528233008433.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center =640x)

<center><font face="楷体">图3-5 rviz中的tf框架</font></center>


## 引言
 在这个-SLAM建图和导航仿真实例-项目中，主要分为三个部分，分别是
 - [（一）模型构建](https://blog.csdn.net/qq_37266917/article/details/106317862)
 - [（二）根据已知地图进行定位和导航](https://blog.csdn.net/qq_37266917/article/details/106415436)
 - [（三）使用RTAB-MAP进行建图和导航](https://blog.csdn.net/qq_37266917/article/details/106416213)

该项目的slam_bot已经上传我的[Github](https://github.com/Xiangyu-Fu/slam_bot)。

这是第二部分。
## 二、 根据已知地图进行定位和导航
### 1.添加地图

在slam_bot目录中创建一个新文件夹。
```
$ cd ~/catkin_ws/src/slam_bot/
$ mkdir maps
$ cd maps
```
从[Github](https://github.com/Xiangyu-Fu/slam_bot/tree/master/maps)中将jackal_race.pgm和jackal_race.yaml复制到“ maps”文件夹中。

```
$ cd ..
$ cd worlds
```
将文件jackal_race.world从[Github](https://github.com/Xiangyu-Fu/slam_bot/tree/master/worlds)复制到“ worlds”文件夹中。

接下来，修改slam.launch文件并更新此新地图/世界的路径。
```
$ cd ..
$ cd launch/
$ nano slam_bot.launch
```
修改参数world_name，使其指向jackal_race.world。现在可以在新地图中启动了。
```
$ cd ~/catkin_ws/
$ roslaunch slam_bot/slam.launch
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529004543724.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)
<center><font face="楷体">图2-1 jackal_race.world</font></center>

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529004600113.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)

<center><font face="楷体">图2-2 在jackal_race.world下的RViz界面/font></center>

### 2.导航堆栈
**导航堆栈**(Navigation Stack)在概念层面上相当简单。它从**里程表**(odometry )和传感器流中获取信息，并输出速度指令发送到移动基地。然而，在机器人上使用**导航堆栈**则要复杂一些。作为**导航堆栈**使用的前提条件，机器人必须运行ROS，有一个TF转换树，并使用正确的ROS消息类型发布传感器数据。同时，**导航堆栈**需要对机器人的形状和动态进行配置，以使机器人的性能达到较高的水平。


##### 导航堆栈需求:
虽然导航堆栈旨在尽可能地通用，但存在三个主要的硬件要求限制了它的使用：

- 它仅适用于差速驱动和完整轮式机器人。假定通过发送所需的速度命令来控制移动基座，以达到以下形式：x速度，y速度，theta速度。
- 它需要将平面激光器安装在活动基座上的某个位置。该激光器用于地图构建和定位。
- 导航堆栈是在方形机器人上开发的，因此其性能在接近正方形或圆形的机器人上将是最好的。它可以在任意形状和大小的机器人上运行，但是在狭窄的空间（如门口）中使用大型矩形机器人可能会遇到困难。
> 更多可以查看[ros wiki navigation](http://wiki.ros.org/navigation)。


##### move_base包

使用move_base程序包，通过该程序包我们可以在地图中为机器人定义目标位置，然后机器人将导航至该目标位置。

![move_base](https://img-blog.csdnimg.cn/20200529003356898.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)
<center><font face="楷体">图2-3 move_base包</font></center>

move_base软件包是一个非常强大的工具。它利用**成本图**（costmap）将地图的每个部分划分为占用的区域（如墙壁或障碍物）和未占用的区域。当机器人四处移动时，相对于**全局成本图**的**局部成本图**会不断更新，从而使程序包可以为机器人定义连续的移动路径。

move_base程序包具有一些内置的纠正行为或操作。根据特定条件，例如检测到特定障碍物或机器人是否被卡住，它将围绕障碍物导航机器人或旋转机器人，直到找到合适的路径。


### 3.AMCL包
在之前的博客中介绍了[蒙特卡洛定位算法（MCL）](https://blog.csdn.net/qq_37266917/article/details/105804597)。当机器人在地图中四处导航时，自适应蒙特卡洛定位（AMCL）可在一段时间内动态调整粒子数量。与MCL相比，这种自适应过程具有明显的计算优势。

ROS amcl软件包实现了此变体，我们将使用此软件包与机器人集成在一起以在提供的地图中对其进行定位。

创建一个新的启动文件。
```
$ cd ~/catkin_ws/src/slam_bot/launch/
$ nano amcl.launch
```
该启动文件具有三个节点，其中一个用于amcl包。
```
<?xml version="1.0"?>
<launch>

  <!-- Map server -->
  <arg name="map_file" default="$(find slam_bot)/maps/jackal_race.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

  <!-- Localization-->
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <remap from="scan" to="/slam_bot/laser/scan"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="odom_model_type" value="diff-corrected"/>
    <param name="base_frame_id" value="base_footprint"/>
    <param name="global_frame_id" value="map"/>
  </node>

  <!-- Move base -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find slam_bot)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find slam_bot)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find slam_bot)/config/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find slam_bot)/config/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find slam_bot)/config/base_local_planner_params.yaml" command="load" />

    <remap from="cmd_vel" to="/cmd_vel"/>
    <remap from="odom" to="/odom"/>
    <remap from="scan" to="/slam_bot/laser/scan"/>

    <param name="base_global_planner" type="string" value="navfn/NavfnROS" />
    <param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS"/>

  </node>


</launch>

```
##### map_server包
使用map_server包的节点加载提供的地图。`robot_state_publisher`根据URDF文件构建了机器人的整个tf树。但是它并没有通过链接“`map`”框架来扩展tf树。amcl软件包通过链接“ `map`”和“ `odom`”框架自动完成此操作。

##### amcl包
添加一个将启动amcl包的节点。该软件包具有自己的一组参数，这些参数定义了它在RViz中的行为以及一切与机器人和所提供地图的关系，以便机器人可以有效地对其自身进行定位。amcl软件包完全依赖于机器人的odom和激光扫描数据。

##### move_base包
move_base具有自己的一组必需参数，以帮助其有效执行。可以指定remap特定的主题，以使他们能够从里程计或激光数据中获取输入，还可以定义一些参数文件或配置文件，这些文件涉及与成本图有关的参数或定义，以及用于创建路径并沿该路径导航机器人的本地计划器。

首先添加以下配置文件。

在[Github](https://github.com/Xiangyu-Fu/slam_bot/tree/master/config)中复制以下文件，并将其添加到config文件夹：

- local_costmap_params.yaml
- global_costmap_params.yaml
- costmap_common_params.yaml
- base_local_planner_params.yaml

###### costmap_common_params.yaml配置文件
```xml
map_type: costmap

obstacle_range: 5
raytrace_range: 20.0

transform_tolerance: 1.0

inflation_radius: 1.5

observation_sources: laser_scan_sensor

laser_scan_sensor: {sensor_frame: /hokuyo, data_type: LaserScan, topic: /slam_bot/laser/scan, marking: true, clearing: true}
```
这些参数对于定义你的成本图如何更新障碍物信息以及你的机器人在导航时如何响应障碍物信息是非常重要的。

`obstacle_range` - 例如，如果设置为0.1，这意味着如果激光传感器检测到的障碍物距离机器人的底部0.1米以内，该障碍物将被添加到成本图中。调整这个参数可以帮助丢弃噪声、错误检测到障碍物，甚至是计算成本。

`raytrace_range` - 这个参数用于在机器人移动时清除和更新成本图中的自由空间。

`inflation_radius` - 这个参数决定了机器人几何体和障碍物之间的最小距离。试着为这个参数设置一个非常高的值，然后启动项目并选择全局成本图选定。你会发现障碍物似乎被 "膨胀 "了，如下图所示。这个参数的适当值可以保证机器人顺利地在地图上导航，不会撞到墙面上，不会被卡住，甚至可以通过任何狭窄的通道。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529004131890.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)
<center><font face="楷体">图2-4 inflation_radius参数</font></center>

### 4.运行amcl.launch
首先运行我们的模型
```
$ cd ~/catkin_ws/
$ roslaunch slam_bot slam.launch
```

在新的终端中
```
$ roslaunch slam_bot amcl.launch
```

报错：
```
ERROR: cannot launch node of type [map_server/map_server]: map_server
ROS path [0]=/opt/ros/kinetic/share/ros
ROS path [1]=/home/stan/catkin_ws/src
ROS path [2]=/opt/ros/kinetic/share
ERROR: cannot launch node of type [amcl/amcl]: amcl
ROS path [0]=/opt/ros/kinetic/share/ros
ROS path [1]=/home/stan/catkin_ws/src
ROS path [2]=/opt/ros/kinetic/share
ERROR: cannot launch node of type [move_base/move_base]: move_base
ROS path [0]=/opt/ros/kinetic/share/ros
ROS path [1]=/home/stan/catkin_ws/src
ROS path [2]=/opt/ros/kinetic/share
```
解决办法
```
$ sudo apt-get install ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-amcl
$ sudo apt-get install ros-kinetic-move-base
```

在RViz中

- 选择“ odom”作为固定框
- 点击“Add”按钮，然后
  - 添加“ RobotModel”
  - 添加“Map”并选择第一个topic/map
    - 列表中的第二个和第三个主题将显示global costmap和local costmap
  - 添加“ PoseArray”并选择主题/ particlecloud
    - 这将在机器人周围显示一组箭头

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529005328699.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)
<center><font face="楷体">图2-5 PoseArray</font></center>


可以在RViz工具栏中，选择“ 2D Nav Goal”（2D导航目标），在地图上的其他任何地方单击，然后从那里拖动以定义目标位置以及机器人在目标位置处的方向。

### 5. 测试
在[Github](https://github.com/Xiangyu-Fu/slam_bot/tree/master/src)中提供了一个C++节点，该节点可以将机器人导航到目标位置。

需要为此创建一个新文件夹。
```
$ cd ~/catkin_ws/src/slam_bot
$ mkdir src
$ cd src
```
将navigation_goal.cpp文件复制到此文件夹。

为了使用或启动此节点，首先需要对其进行编译,所以需要为此修改CMakeLists.txt文件。

```
$ cd ~/catkin_ws/src/slam_bot
nano CMakeLists.txt
```
将以下内容添加到CMakeLists.txt中
```
find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
    actionlib
    move_base_msgs
    roscpp
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_goal src/navigation_goal.cpp)

target_link_libraries(navigation_goal ${catkin_LIBRARIES})
```
修改完成后，对slam_bot进行编译
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

在slam_bot.launch和amcl.launch运行的情况下，打开一个新的terminal
```
$ cd ~/catkin_ws
$ rosrun slam_bot navigation_goal
```
上面的代码将运行该节点，机器人会移动至目标位置。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529010436671.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center)
<center><font face="楷体">图2-6 navigation_goal</font></center>

```
[ INFO] [1590585650.579740467, 1608.259000000]: Waiting for the move_base action server
[ INFO] [1590585650.827041070, 1608.334000000]: Connected to move_base server
[ INFO] [1590585650.827097987, 1608.334000000]: Sending goal
[ INFO] [1590585964.490917946, 1727.796000000]: Excellent! Your robot has reached the goal position.
```


## 引言
 在这个-SLAM建图和导航仿真实例-项目中，主要分为三个部分，分别是
 - [（一）模型构建](https://blog.csdn.net/qq_37266917/article/details/106416213)
 - [（二）根据已知地图进行定位和导航](https://blog.csdn.net/qq_37266917/article/details/106415436)
 - [（三）使用RTAB-MAP进行建图和导航](https://blog.csdn.net/qq_37266917/article/details/106317862)

该项目的slam_bot已经上传我的[Github](https://github.com/Xiangyu-Fu/slam_bot)。

这是第三部分，完成效果如下

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529075334133.gif#pic_center)

<center><font face="楷体">图1 建图和导航</font></center>

## 三、使用RTAB-Map进行建图和导航
### 1. rtabmap_ros介绍
之前在[GraphSLAM](https://blog.csdn.net/qq_37266917/article/details/106247220)博客中我提到了RTAB-Map。

RTAB-Map是具有实时约束的RGB-D SLAM方法，它是一种基于增量基于外观的闭环检测器的RGB-D，基于立体声和激光雷达图的SLAM方法。闭环检测器使用词袋方法来确定新图像来自先前位置或新位置的可能性。当接受循环闭合假设时，新约束将添加到地图的图形中，然后图形优化器将地图中的错误最小化。使用内存管理方法来限制用于闭环检测和图形优化的位置数量，以便始终遵守对大型环境的实时约束。

**代码库：**

https://github.com/introlab/rtabmap_ros.git
http://introlab.github.io/rtabmap

**节点：**

所有sensor_msgs/Image话题使用image_transport.

**rtabmap**

这是核心节点，是RTAB-Map核心库的封装，这是在检测到循环闭合时增量构建和优化地图的图形。
节点的在线输出是具有地图上最新添加数据的本地图。
默认RTAB-Map 数据库的位置是 "~/.ros/rtabmap.db"，工作空间也设置为 "~/.ros"
通过订阅cloud_map话题获取3D点云图, grid_map or proj_map话题获取2D网格图

**rtabmapviz**

RTAB-Map的可视化接口，是 RTAB-Map GUI图形库的封装，类似rviz但有针对RTAB-Map的可选项


### 2.rtabmap_ros安装
#### 安装kinetic版本：
```
$ sudo apt-get install ros-kinetic-rtabmap-ros
```
必要的依赖安装 (Qt, PCL, VTK, OpenCV, ...)：
```
$ sudo apt-get install ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
$ sudo apt-get remove ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
```

#### 下载安装RTAB-Map

源码安装rtabmap
```
$ cd ~
$ git clone https://github.com/introlab/rtabmap.git rtabmap
$ cd rtabmap/build
$ cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ..
$ make -j4
$ make install
```
源码安装rtabmap_ros
```
$ cd ~/catkin_ws
$ git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
$ catkin_make -j1 
```
内存太小的话，使用-j1 ，内存大可以去掉。


### 3.创建启动文件
根据[ROS wiki](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot)的建议，我们可以用类似以下的方式搭建mapping的启动文件。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529013826250.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center =800x)
<center><font face="楷体">图3-1 Kinect + Odometry + 2D laser</font></center>

```xml
<?xml version="1.0" encoding="UTF-8"?>

<launch>

  <!-- Arguments for launch file with defaults provided -->
  <arg name="database_path"     default="rtabmap.db"/>
  <arg name="rgb_topic"   default="/camera/rgb/image_raw"/>
  <arg name="depth_topic" default="/camera/depth/image_raw"/>
  <arg name="camera_info_topic" default="/camera/rgb/camera_info"/>  


  <!-- Mapping Node -->
  <group ns="rtabmap">
    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">

      <!-- Basic RTAB-Map Parameters -->
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="base_footprint"/>
      <param name="odom_frame_id"       type="string" value="/odom"/>
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>

      <!-- RTAB-Map Inputs -->
      <remap from="scan"            to="/slam_bot/laser/scan"/>
      <remap from="rgb/image"       to="$(arg rgb_topic)"/>
      <remap from="depth/image"     to="$(arg depth_topic)"/>
      <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>

      <!-- RTAB-Map Output -->
      <remap from="grid_map" to="/map"/>

      <!-- Rate (Hz) at which new nodes are added to map -->
      <param name="Rtabmap/DetectionRate" type="string" value="1"/> 

      <!-- 2D SLAM -->
      <param name="Reg/Force3DoF" type="string" value="true"/>  

      <!-- Loop Closure Constraint -->
      <!-- 0=Visual, 1=ICP (1 requires scan)-->
      <param name="Reg/Strategy" type="string" value="0"/>  

      <!-- Loop Closure Detection -->
      <!-- 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE-->
      <param name="Kp/DetectorStrategy" type="string" value="0"/> 

      <!-- Maximum visual words per image (bag-of-words) -->
      <param name="Kp/MaxFeatures" type="string" value="400"/>  

      <!-- Used to extract more or less SURF features -->
      <param name="SURF/HessianThreshold" type="string" value="100"/>


      <!-- Minimum visual inliers to accept loop closure -->
      <param name="Vis/MinInliers" type="string" value="15"/> 

      <!-- Set to false to avoid saving data when robot is not moving -->
      <param name="Mem/NotLinkedNodesKept" type="string" value="false"/>

    </node> 
  </group>
</launch>
```
---
让我们分解一下这个启动文件：
```xml
      <param name="database_path"       type="string" value="$(arg database_path)"/>
```
参数-`--delete_db_on_start`将使rtabmap在启动时删除数据库（默认位于`~/.ros/rtabmap.db`）。如果想让机器人继续从之前的映射会话中进行映射，应该删除 `--delete_db_on_start`。

---
```xml
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="base_footprint"/>
      <param name="odom_frame_id"       type="string" value="/odom"/>
```
设置database_path,fixed frame和odom。

---
```xml
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>
```
默认情况下， subscribe_depth为true。但是，在这个设置中，我们将使用RGB-D图像输入，所以将subscribe_depth设置为false，将subscribe_rgbd设置为true。因为我们有一个2D lidar，所以将 subscribe_scan 设置为 true。如果我们有一个3D lidar发布sensor_msgs/PointCloud2消息，则将sensemble_scan_cloud设置为true，并重映射相应的scan_cloud主题而不是scan。
- 当 subscribe_rgbd=true时，应设置 rgbd_image 输入主题。
- 当 subscribe_scan=true时，需要设置扫描输入主题。

---
```xml
      <remap from="scan"            to="/slam_bot/laser/scan"/>
      <remap from="rgb/image"       to="$(arg rgb_topic)"/>
      <remap from="depth/image"     to="$(arg depth_topic)"/>
      <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>
```
设置所需的输入主题。若话题没有/，则意味着在其命名空间中订阅了相应话题，例如“rgbd_image” 订阅/rtabmap/rgbd_image。


---
```xml
      <!-- 2D SLAM -->
      <param name="Reg/Force3DoF" type="string" value="true"/>  

      <!-- Loop Closure Constraint -->
      <!-- 0=Visual, 1=ICP (1 requires scan)-->
      <param name="Reg/Strategy" type="string" value="0"/>
```
- `Reg/Force3DoF`：强制3DoF配准，不会估计roll、pitch和z。

- `Reg/Strategy`：使用ICP来细化使用激光扫描发现的ICP的全局闭环。

---
以下是mapping.launch中没有提到的参数的简要概述。
```xml
<!-- RTAB-Map's parameters -->
<param name="RGBD/NeighborLinkRefining" type="string" value="true"/>
<param name="RGBD/ProximityBySpace"     type="string" value="true"/>
<param name="RGBD/AngularUpdate"        type="string" value="0.01"/>
<param name="RGBD/LinearUpdate"         type="string" value="0.01"/>
<param name="RGBD/OptimizeFromGraphEnd" type="string" value="false"/>
<param name="Grid/FromDepth"            type="string" value="false"/> 

<!-- ICP parameters -->
<param name="Icp/VoxelSize"                 type="string" value="0.05"/>
<param name="Icp/MaxCorrespondenceDistance" type="string" value="0.1"/>
```

- `RGBD/NeighborLinkRefining`：使用ICP对输入lidar主题进行正确的里程测量。

- `RGBD/ProximityBySpace`：根据机器人在地图中的位置，寻找局部环形闭合。这在机器人朝反方向回来时非常有用。由于摄像头朝后，无法找到全局环形闭合点。所以利用位置和之前添加的激光扫描到地图上，我们用ICP找到变换。

- `RGBD/AngularUpdate`：机器人应该移动更新地图（如果不是0，则为0）。

- `RGBD/LinearUpdate`：机器人应该移动来更新地图（如果不是0的话）。

- `RGBD/OptimizeFromGraphEnd`：设置为false（默认为false），在循环关闭时，图形将从地图中的第一个姿势开始优化。TF /map -> /odom会在这种情况下改变。当设置为false时，图形将从添加到地图中的最新节点开始优化，而不是第一个节点。通过从最后一个节点开始优化，最后一个姿势保持它的值，而所有之前的姿势都会根据它来修正（所以/odom和/map总是匹配在一起）。

- `Grid/FromDepth`: 如果为true，则从深度相机生成的云层中创建占用网格。如果为false，则从激光扫描中生成占领网格。

- `Icp/VoxelSize`：扫描在做ICP之前被过滤到5厘米的体元。

- `Icp/MaxCorrespondenceDistance`：ICP注册时点之间的最大距离。

> 更多可以查看[SetupOnYourRobot](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot)。

### 4.配置控制程序teleop
使用以下python程序来控制模型在gazebo中的移动。
```
$ cd ~/catkin_ws/slam_bot/launch
$ nano teleop
$ sudo chmod 777 teleop
```
```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
msg = """
Control Your  SLAM-Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop')
    pub = rospy.Publisher('~/cmd_vel', Twist, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```

### 5.运行和测试
从[Github](https://github.com/Xiangyu-Fu/slam_bot/blob/master/worlds/kitchen_dining.world)中获取环境将其置于worlds文件夹中。

更改slam.launch
```xml
    <arg name="world_name" value="$(find slam_bot)/worlds/kitchen_dining.world"/>
```
运行程序
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch slam_bot slam.luaunch
```
启动Teleop节点
```
$ rosrun slam_bot teleop
```
启动建图节点
```
roslaunch slam_project mapping.launch
```

使用teleop控制模型四处移动，使用ctrl+c停止建图节点。

使用rtabmap-databaseViewer打开建图数据库
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

打开后，我们将需要添加一些窗口以更好地查看相关信息，因此：
- 同意使用数据库参数
- View -> Constraint View
- View -> Graph View
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529042826758.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center =800x)

<center><font face="楷体">图3-2 rtabmap-databaseViewer</font></center>

可以在左下方看到循环关闭的数量。这些代码代表以下各项：
邻居(Neighbor)、邻居合并(Neighbor Merged)、全局环路闭合、按空间划分的局部环路闭合、按时间划分的局部环路闭合、用户环路闭合、优先链接。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529003017489.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center =800x)
<center><font face="楷体">图3-3 3d地图</font></center>


可以使用的另一个工具是**rtabmapviz**，它是用于实时可视化特征映射，循环闭合等功能的附加节点。由于计算量大，不建议在仿真中进行映射时使用此工具。**rtabmapviz**非常适合在实时映射期间部署在真实的机器人上，以确保获得完成循环闭合所必需的功能。

要启动，将以下代码添加到mapping.launch文件中：
```
<!-- visualization with rtabmapviz -->
    <node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" args="-d $(find rtabmap_ros)/launch/config/rgbd_gui.ini" output="screen">
        <param name="subscribe_depth"             type="bool" value="true"/>
      <param name="subscribe_scan"              type="bool" value="true"/>
      <param name="frame_id"                    type="string" value="base_footprint"/>

      <remap from="rgb/image"       to="$(arg rgb_topic)"/>
      <remap from="depth/image"     to="$(arg depth_topic)"/>
      <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>
      <remap from="scan"            to="/scan"/>
    </node>
```
### 6. 导航
在导航中我们重新使用jackal_race.world。

首先对整个地图经行建图。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529073136225.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM3MjY2OTE3,size_16,color_FFFFFF,t_70#pic_center =600x)
<center><font face="楷体">图3-4 jackal_race.world的3d地图</font></center>

#### localization.launch定位节点
复制mapping.launch文件并更名localization.launch。

还需要对localization.launch文件进行以下更改：

1. args="--delete_db_on_start"从节点启动器中删除，因为您还将需要数据库进行本地化。

2. 删除Mem / NotLinkedNodesKept参数

3. 最后，添加字符串类型的Mem / IncrementalMemory参数并将其设置为false，以完成使机器人进入本地化模式所需的更改。

#### move.launch导航节点
如之前一样，使用move_base包
```
<?xml version="1.0"?>
<launch>

  <!-- Move base -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find slam_bot)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find slam_bot)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find slam_bot)/config/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find slam_bot)/config/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find slam_bot)/config/base_local_planner_params.yaml" command="load" />

    <remap from="cmd_vel" to="/cmd_vel"/>
    <remap from="odom" to="/odom"/>
    <remap from="scan" to="/slam_bot/laser/scan"/>

    <param name="base_global_planner" type="string" value="navfn/NavfnROS" />
    <param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS"/>

  </node>
</launch>
```

#### rivz节点
我们将rviz节点从slam.launch中移除，等各个节点建立完成后我们再打开rviz，这样需要为rviz专门编写一个launch文件。
```xml
<launch>
  <!--launch rviz-->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find slam_bot)/launch/config/robot_slam.rviz"/>
</launch>
```

其中`args="-d $(find slam_bot)/launch/config/robot_slam.rviz"`引入了rviz的设置文件，这样就不需要一一添加各个节点了。

#### 批量运行
由于需要使用的运行命令太多，提供如下的程序
```
#! /bin/bash

gnome-terminal -x bash -c "killall gzserver" &
sleep 1 &&

echo ' '
read -p 'Would you like to clear the previous map database? (y/n): ' ansinput

if [ “$ansinput” = “y” ]
then
 printf '\n Map deleted \n'
 rm -f ~/.ros/rtabmap.db

elif [ “$ansinput” = “n” ]
then
 printf '\n Map kept \n'

else
 echo 'Warning: Not an acceptable option. Choose (y/n).
         '
fi

echo ' '

read -p 'Enter target world destination or d for default: ' input_choice

if [ “$input_choice” = “d” ]
then
gnome-terminal -x bash -c " roslaunch slam_bot slam.launch "&

else
gnome-terminal -x bash -c "roslaunch slam_bot slam.launch world_file:=$input_choice"  &
fi

sleep 3 &&

gnome-terminal -x bash -c " rosrun slam_bot teleop" &

sleep 3 &&

echo ' '
read -p 'mapping or localization (m/l): ' input

if [ “$input” = “m” ]
then
gnome-terminal -x bash -c "  roslaunch slam_bot mapping.launch simulation:=true" 


elif [ “$input” = “l” ]
then
gnome-terminal -x bash -c "  roslaunch slam_bot localization.launch" 
gnome-terminal -x bash -c "  roslaunch slam_bot move.launch" 


else
 echo 'Warning: Not an acceptable option. Choose (m/l).'
fi
sleep 3 &&

gnome-terminal -x bash -c "  roslaunch slam_bot rviz.launch"

echo ' '
echo 'Script Completed'
echo ' '
```

在[【SLAM建图和导航仿真实例】（二）](https://blog.csdn.net/qq_37266917/article/details/106415436)中提供了一个测试程序navigation_goal，我们在这里还是借助这个程序。

打开一个新的terminal
```
$ cd ~/catkin_ws
$ rosrun slam_bot navigation_goal
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200529075334133.gif#pic_center)
<center><font face="楷体">图3-5 建图和导航</font></center>