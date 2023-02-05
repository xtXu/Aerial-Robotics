# Robotics:Aerial Robotics (空中机器人) (二)

# week1.2 Energetics and System Design



> 该专栏为Coursera上宾夕法尼亚大学Kumar教授的课程——Robotics:Aerial Robotics的课程笔记和整理
>
> 原课程链接：https://www.coursera.org/learn/robotics-flight?specialization=robotics

## Basic Mechanics (基本力学)

首先，我们来看四旋翼的基本力学原理。

<img src="https://gitee.com/xxtRoche/images/raw/master/img/20220314205223.png" alt="image-20220314205223192" style="zoom:67%;" />

每个rotor（旋翼？）产生的**推力（Thrust）**与转速成二次关系（approximately quadratic），即
$$
F = k_F\omega^2
$$
rotor 旋转时，需要克服**阻力矩（Drag moment）**，阻力矩与转速也近似成二次关系，即
$$
M = k_M\omega^2
$$
因此，对于四旋翼来说，当处于悬停状态时，每个rotor都需要克服$\frac{1}{4}$的重力。通过推力与转速的关系曲线，我们就可以确定产生这些力所需的转速$\omega_0$。

当然，rotor要达到指定的转速，就需要克服阻力，这就需要引入motor了，根据motor的torque-speed 特性来选择合适的电机，以产生足够的扭矩来克服阻力矩。

综上，四旋翼悬停时，rotor通过转速补偿自身重力，根据重力即可确定rotor转速，这同时也给出了每个motor所需施加的扭矩大小。

<img src="https://gitee.com/xxtRoche/images/raw/master/img/20220314213427.png" alt="image-20220314213427221" style="zoom: 80%;" />

四旋翼的受力分析如图所示。合力为四个rotor的推力和自身的重力，合力矩为rotor施加的力矩和对应的阻力矩。
$$
F_i = k_F\omega_i^2\\
M_i =k_M\omega_i^2\\
\text{Resultant Force:}\ \ \ \mathbf{F}=\mathbf{F}_1+\mathbf{F}_2+\mathbf{F}_3+\mathbf{F}_4-mg\mathbf{a}_3\\
\text{Resultant Moment:}\ \ \ \mathbf{M}=\mathbf{r}_1\cross\mathbf{F}_1+\mathbf{r}_2\cross\mathbf{F}_2+\mathbf{r}_3\cross\mathbf{F}_3+\mathbf{r}_4\cross\mathbf{F}_4\\ \qquad \qquad \qquad \qquad +\mathbf{M}_1+\mathbf{M}_2+\mathbf{M}_3+\mathbf{M}_4
$$
当处于平衡状态时，合力和合力矩均为0。而当合力不为0时，就产生了加速度。因此，推力和重力决定了无人机的运动方式。

<img src="https://gitee.com/xxtRoche/images/raw/master/img/20220314215423.png" alt="image-20220314215422637" style="zoom:80%;" />

## Dynamics and 1-D Linear Control (一维控制)

本节仅讨论一维方向——高度的控制。

首先考虑垂直方向的动力学方程：
$$
\sum_{i=1}^{4} k_{F} \omega_{i}^{2}+m \mathbf{g}=m \mathbf{a}
$$
我们选择加速度作为系统输入量 $u$，系统状态为 $x$，则
$$
\text { Input } \quad u=\frac{1}{m}\left[\sum_{i=1}^{4} k_{F} \omega_{i}^{2}+m \mathbf{g}\right]
$$
由此，构建了一个**二阶动态系统（Second order dynamic system）**。

![image-20220315144934927](https://gitee.com/xxtRoche/images/raw/master/img/20220315144935.png)

整个控制问题可以表示为：

+ **State, input**：	$x, u \in \mathbb{R}$
+ **Plant model**:      $\ddot{x}=u$
+ 我们想要确定 $u(t)$，使得 $x(t)$ 能跟随期望轨迹 $x^{des}(t)$

为了求解该问题，首先定义error：
$$
e(t)=x^{des}(t)-x(t)
$$
我们的目标是让$x(t)$跟随期望$x^{des}(t)$，即使得 $e(t)$ 为0；更具体地，我们希望 error 能尽量快的下降到0。而指数下降速度较快，因此我们可以希望 error 能**指数下降至0**（go exponentially to 0）。而**二阶微分方程**的解可以满足指数下降，所以，我们想要确定 $u(t)$，使得 $e(t)$ 能满足一个二阶微分方程，即
$$
\text{Find} \quad u \quad \text{such that} \\
\ddot{e}+K_v\dot{e}+K_pe=0\quad K_p,K_v>0
$$
有如下推导
$$
\begin{aligned}
e(t) &=x^{des}(t)-x(t)\\
\ddot{e}(t)&=\ddot{x}^{des}(t)-\ddot{x}(t)=\ddot{x}^{des}(t)-u(t)\\
\ddot{x}^{des}(t)&-u(t)+K_v\dot{e}+K_pe=0\\
u(t)&=\ddot{x}^{des}(t)+K_v\dot{e}+K_pe
\end{aligned}
$$
由此就得到了 **PD** 控制方程
$$
u(t)=\ddot{x}^{des}(t)+K_v\dot{e}+K_pe
$$
当 $K_v,K_p>0$ 时，就可保证 $e(t)$ 指数下降至0，从而保证 $x(t)$ 趋向于 $x^{des}(t)$。其中，$K_p$ 称为 proportional gain，$K_v$ 称为 derivative gain，而 $\ddot{x}^{des}(t)$ 称为 feedforward term（前馈项）。 

+ Proportional control acts like a spring (capacitance) response 
+ Derivative control is a viscous dashpot (resistance) response
+ 过大的微分增益会导致系统过阻尼，收敛缓慢

当存在外界扰动（disturbance）或模型误差（modeling error）时，通常都会引入一个积分量进行修正，即耳熟能详的**PID控制**，即
$$
u(t)=\ddot{x}^{\mathrm{des}}(t)+K_{v} \dot{e}(t)+K_{p} e(t)+K_{i} \int_{0}^{t} e(\tau) d \tau
$$

+ PID控制构建了一个三阶闭环系统，通过 Integral control 使稳态误差趋于0。
+ ![image-20220315155316960](https://gitee.com/xxtRoche/images/raw/master/img/20220315155317.png)

## Design Consideration(推重比)

上节中，我们使用了一个简单的理想模型进行了控制，但其中有一个很大的问题：我们假设电机能产生controller所需的任意大的推力。但现实中，推力受限于电机的最大扭矩(peak motor torque)。

![image-20220317170543971](https://gitee.com/xxtRoche/images/raw/master/img/20220317170544.png)

设最大推力为 $T_{max}$，根据
$$
\begin{aligned}
u &=\frac{1}{m}\left[\sum_{i=1}^{4} k_{F} \omega_{i}^{2}+m \mathbf{g}\right] \\
&=\frac{1}{m}[T +m \mathbf{g}]
\end{aligned}
$$
得到最大输入
$$
\begin{aligned}
u_{max}&=\frac{1}{m}[T_{max} +m \mathbf{g}]\\
				&=\frac{T_{max}}{m}+g
\end{aligned}
$$
因此，先前的**PD control**可以改为
$$
u(t)=\min \left(\ddot{x}^{\text {des }}(t)+K_{V} \dot{e}(t)+K_{P} e(t), u_{\max }\right)
$$
**PID control**可改为
$$
u(t)=\min \left(\ddot{x}^{\text {des }}(t)+K_{V} \dot{e}(t)+K_{P} e(t)+K_{I} \int_{0}^{t} e(\tau) d \tau, u_{\max }\right)
$$
由于四旋翼受到的合力为重力和推力的合力，因此，推力和重力都会影响控制系统的最大输入，从而影响控制的效果，我们可以将其统一为推重比（thrust to weight ratio），从而通过调整推重比来调整四旋翼的性能。

## Design Consideration(能耗)

![image-20220317180111827](https://gitee.com/xxtRoche/images/raw/master/img/20220317180112.png)

四旋翼的基本功耗为：200 W/kg

锂电池的基本功耗：>200 W/kg

锂电池的容量约为：200 Whr/kg

四旋翼的元件的质量占比：电池～33%，电机~25%

对于激光雷达、相机、机载电脑来说，其增加的功耗主要来源于他们增加的负载(payload)。

## Agility and Maneuverability（机动性）

考虑四旋翼的Agility时，可以考虑两方面：

+ Maximum Velocity to Rest (Minimize stopping distance) 最大化加速度 <img src="https://gitee.com/xxtRoche/images/raw/master/img/20220317184930.png" alt="image-20220317184929845" style="zoom:50%;" />
+ Turn Quickly without slowing down (Minimize turning radius) 最小化转弯半径<img src="https://gitee.com/xxtRoche/images/raw/master/img/20220317185131.png" alt="image-20220317185131058" style="zoom:50%;" />

考虑一个垂直平面内的简单四旋翼，

![image-20220317185545451](https://gitee.com/xxtRoche/images/raw/master/img/20220317185546.png)

通过受力分析，可得
$$
\begin{aligned}
\ddot{y}&=-\frac{u_1sin\phi}{m}\\
\ddot{z}&=\frac{u_1cos\phi-mg}{m}\\
\ddot{\phi}&=\frac{u_2}{I_{xx}}
\end{aligned}
$$
写成矩阵形式，则
$$
\left[\begin{array}{c}
\ddot{y} \\
\ddot{z} \\
\ddot{\phi}
\end{array}\right]=\left[\begin{array}{c}
0 \\
-g \\
0
\end{array}\right]+\left[\begin{array}{cc}
-\frac{1}{m} \sin \phi & 0 \\
\frac{1}{m} \cos \phi & 0 \\
0 & \frac{1}{I_{x x}}
\end{array}\right]\left[\begin{array}{l}
u_{1} \\
u_{2}
\end{array}\right]
$$
我们想要更快地加减速，更快地旋转，因此，

+ 最大化线加速度 $a_{max}$，即最大化 $\frac{u_{1,max}}{m}$
+ 最大化角加速度 $\alpha_{max}$，即最大化 $\frac{u_{2,max}}{I_{xx}}$

## Effective of Size(尺寸的影响)

四旋翼的尺寸对其性能也会产生影响：

![image-20220317192031026](https://gitee.com/xxtRoche/images/raw/master/img/20220317192031.png)

+ mass, inertia
  $$
  m\sim l^3,I\sim l^5
  $$

+ thrust
  $$
  F\sim\pi r^2\cross(\omega r)^2\rightarrow F\sim r^2v^2\\
  r\sim l \rightarrow F\sim l^2v^2\\
  a\sim \frac{F}{m},m\sim l^3\rightarrow a\sim\frac{v^2}{l}
  $$

+ moment
  $$
  M\sim Fl\rightarrow M\sim l^3v^2\\
  
  \alpha\sim \frac{M}{I},I\sim l^5\rightarrow \alpha\sim\frac{v^2}{l^2}
  $$

其中 $v$ 成为 blade tip speed，$v$ 与 $l$ 的关系有两种范式（似乎是经实验测得？）：

+ Froud scaling
  $$
  v\sim \sqrt{l} \rightarrow a\sim 1,\alpha\sim\frac{1}{l}
  $$

+ Mach scaling
  $$
  v\sim 1 \rightarrow a\sim \frac{1}{l},\alpha\sim\frac{1}{l^2}
  $$

不论哪种范式，减小 $l$，及减小四旋翼尺寸，都能提高 agility。

