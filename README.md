# MPC控制器说明

### 参数含义：

详见MPC.h

### 输入：

1. planner规划后的一段路径点

2. 发布的消息格式定义在PathPlanner格式下，包含路径中包含点个数以及x,y坐标，具体如下;

```
uint8 length
float64[] x
float64[] y
```



3. 为了更好的根据路径点拟合曲线，在计算允许的情况下，讲尽量多的点发布出来，目前一次发布10个点，每两个点之间间隔0.6米

### 依赖：

1. ipopt-3.12.1
2. Eigen-3.3(包含在了MPC工程中
3. CppAD

### 使用步骤：

1. 使用ipopt-3.12.1里面的install文件安装
   ./install_ipopt.sh

   (github下载的ipopt库缺少东西，无法使用上述方法，需要去官网下载)

2. CppAD安装
   sudo apt-get install cppad

### 内容：

1. `src/main.cpp`主cpp文件，根据上层传来的path信息进行MPC控制器的求解
2. `src/MPC.cpp` MPC算法主文件
3. `src/MPC.h` MPC算法参数

### 使用步骤：

1. Make a build directory: 
`mkdir build && cd build`
2. Compile: 
`cmake .. && make`
3. source it and Run it

