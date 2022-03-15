# Robotics:Aerial Robotics (空中机器人) (二)

# week1.2 Energetics and System Design



> 该专栏为Coursera上宾夕法尼亚大学Kumar教授的课程——Robotics:Aerial Robotics的课程笔记和整理
>
> 原课程链接：https://www.coursera.org/learn/robotics-flight?specialization=robotics

## Basic Mechanics

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

## Dynamics and 1-D Linear Control

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















