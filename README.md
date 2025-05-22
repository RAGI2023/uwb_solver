# 安装教程
1. 安装依赖
```shell
    sudo apt install ros-<ros-distro>-serial ros-<ros-distro>-geometry_msgs libeigen3-dev
```
2. 切换到对应的ros工作区
```shell
    cd ros_ws/src
```
3. git clone
```shell
    git clone https://github.com/RAGI2023/uwb_solver.git uwb_solber/
```
4. 修改串口号与基站位置
- 基站位置:[修改](https://github.com/RAGI2023/uwb_solver/blob/main/src/uwb_solve_node.cpp#L141)
- 串口号:[修改](https://github.com/RAGI2023/uwb_solver/blob/main/src/uwb_solve_node.cpp#L149)

# UWB定位结算原理
## 1. 问题描述

给定四个基站，其三维空间坐标分别为：

$$
\text{基站}_i: (x_i, y_i, z_i), \quad i = 1, 2, 3, 4
$$

目标点 $Ro$ 的坐标为 $(x_o, y_o, z_o)$，目标与各基站的距离为 $l_i$，为已知常量，通过标签输出。

---

## 2. 距离模型

根据欧几里得距离公式：

$$
\sqrt{(x_o - x_i)^2 + (y_o - y_i)^2 + (z_o - z_i)^2} = l_i
$$

平方两边，得到：

$$
(x_o - x_i)^2 + (y_o - y_i)^2 + (z_o - z_i)^2 = l_i^2 
$$

共有四个方程，但未知数为三个 $(x_o, y_o, z_o)$，故可通过三个差分线性化方程唯一求解。

---

## 3. 差分线性化

选第一个基站作为参考基站，将其他三个方程分别减去第一个方程，消除二次项中的未知平方：

$$
(x_2^2 - x_1^2) + (y_2^2 - y_1^2) + (z_2^2 - z_1^2) - (l_2^2 - l_1^2) = 2x_o(x_2 - x_1) + 2y_o(y_2 - y_1) + 2z_o(z_2 - z_1)
$$

$$
(x_3^2 - x_1^2) + (y_3^2 - y_1^2) + (z_3^2 - z_1^2) - (l_3^2 - l_1^2) = 2x_o(x_3 - x_1) + 2y_o(y_3 - y_1) + 2z_o(z_3 - z_1)
$$

$$
(x_4^2 - x_1^2) + (y_4^2 - y_1^2) + (z_4^2 - z_1^2) - (l_4^2 - l_1^2) = 2x_o(x_4 - x_1) + 2y_o(y_4 - y_1) + 2z_o(z_4 - z_1)
$$

两边同时除以2，得到标准线性方程组

---

## 4. 矩阵表示

设：

- 系数矩阵：

$$
A = \begin{bmatrix}
x_2 - x_1 & y_2 - y_1 & z_2 - z_1 \\
x_3 - x_1 & y_3 - y_1 & z_3 - z_1 \\
x_4 - x_1 & y_4 - y_1 & z_4 - z_1
\end{bmatrix}
$$

- 常数列向量：

$$
b = \frac{1}{2}
\begin{bmatrix}
(x_2^2 - x_1^2) + (y_2^2 - y_1^2) + (z_2^2 - z_1^2) - (l_2^2 - l_1^2) \\
(x_3^2 - x_1^2) + (y_3^2 - y_1^2) + (z_3^2 - z_1^2) - (l_3^2 - l_1^2) \\
(x_4^2 - x_1^2) + (y_4^2 - y_1^2) + (z_4^2 - z_1^2) - (l_4^2 - l_1^2)
\end{bmatrix}
$$

- 未知向量：

$$
X = \begin{bmatrix} x_o \\ y_o \\ z_o \end{bmatrix}
$$

最终线性方程组为：

$$
A X = b
$$

---

## 5. 求解方法

### 方法一：克拉默法则（适用于 3×3 系数矩阵）（MCU）

计算三个行列式：

$$
x_o = \frac{\det(A_x)}{\det(A)}, \quad
y_o = \frac{\det(A_y)}{\det(A)}, \quad
z_o = \frac{\det(A_z)}{\det(A)}
$$

### 方法二：矩阵求逆（ROS）

$$
X = A^{-1} b
$$

---

## 注意事项

- 至少需要 4 个**不共面**的基站。

---
