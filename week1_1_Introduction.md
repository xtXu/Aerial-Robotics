# Robotics:Aerial Robotics (空中机器人) (一)

# week1.1 Introduction 引言



> 该专栏为Coursera上宾夕法尼亚大学Kumar教授的课程——Robotics:Aerial Robotics的课程笔记和整理
>
> 原课程链接：https://www.coursera.org/learn/robotics-flight?specialization=robotics



##  Quadrotor (四旋翼)

![image-20220314162723464](https://gitee.com/xxtRoche/images/raw/master/img/20220314162723.png)

几何结构十分简单，包括**4**个可独立控制的**转子**，挂载在刚体机架上。

其中，电机1,3转向相同，2,4转向相同。

通过改变电机转速，即可控制飞行的位置与旋转。（**position** and **orientation**)

四旋翼共有6个自由度，分别是**三自由度的位移（x-y-z）**和**三自由度的旋转（roll-pitch-yaw)**。



## Key Components of Autonomous Flight

+ **State Estimation**
  + estimate the position and velocity (including rotation and angular velocity of the robot)
+ **Control**
  + command motors and produce desired actions in order to navigate to desired state
+ **Mapping**
+ **Planning**



## State Estimation(状态估计)

![image-20220314164523325](https://gitee.com/xxtRoche/images/raw/master/img/20220314164523.png)

实验室环境下，可使用**motion capture**，精度高(below one millimeter)，实时性好(100-200 times a second)。

> **How do we navigate without GPS,  without external motion capture cameras or any other kinds of external sensors?**

在GPS拒止，又没有motion capture等外部传感器的情况下，常使用SLAM技术(**S**imultaneous **L**ocalization **A**nd **M**apping)。

![image-20220314165721605](https://gitee.com/xxtRoche/images/raw/master/img/20220314165721.png)





















