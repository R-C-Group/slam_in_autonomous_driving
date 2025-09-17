<div align="center">
<h1>《自动驾驶中的SLAM技术》开源代码的学习与注释</h1>
</div>


# 安装与配置

## 编译

```sh
#依赖库
sudo apt install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev

#编译安装thirdparty/pangolin.zip
#编译thirdparty/g2o：最后加sudo make install

mkdir build
cd build
cmake ..
make -j8
```

* 下载数据集：百度云链接: https://pan.baidu.com/s/1ELOcF1UTKdfiKBAaXnE8sQ?pwd=feky 提取码: feky
* 数据路径放在文件夹`data`中

## 运行

* 编译后各章的可执行文件位于`bin`目录下

```sh
#到主目录下

#测试icp匹配
/bin/test_2d_icp_s2s --bag_path ~/datast/2dmapping/floor1.bag --method point2point

```


# 基础知识mark

## 坐标系定义
书中坐标系的定义(form body frame to the world frame)：

$p_w=R_{wb} p_{b}+t_{wb}$

阅读顺序从右到左

## 李群李代数

```py
# 关键点总结
三维旋转构成了三维旋转群SO(3),其对应的李代数为so(3)；
三维变换构成了三维变换矩阵SE(3),其对应的李代数为se(3)；

李群--->李代数：对数映射
李代数--->李群：指数映射
```

旋转矩阵和乘法就构成了旋转矩阵群(`SO(3)特殊正交群`)，变换矩阵和乘法也构成了变换矩阵群（`SE（3）特殊欧氏群`）。
对于群而言，满足乘法的封闭性但是不满足加法的封闭性。

<div align="center">
  <img src="https://kwanwaipang.github.io/SLAM_DEMO/SLAM_Introduction/微信截图_20240810185225.png" width="30%" />
<figcaption>  
直观理解：乘法的封闭性，就是两个矩阵相乘R1*R2还满足上面公式。但是是R1+R2不满足
</figcaption>
</div>

SLAM目的就是优化求解相机的这个最佳的位姿T（变换矩阵），优化方法一般都采用迭代优化的方法，每次迭代都更新一个位姿的增量delta，使得目标函数最小。
这个delta就是通过误差函数对T矩阵进行微分得到的。

对于变换矩阵T（包含旋转R与平移t），所在的空间为SE（3）（称呼为李群，也就是特殊欧氏群），进行映射为se(3),也就是李代数。
李代数对应李群的正切空间，它描述了李群局部的导数。见下面公式：

<div align="center">
  <img src="https://kwanwaipang.github.io/SLAM_DEMO/SLAM_Introduction/微信截图_20240810185704.png" width="50%" />
<figcaption>  
  对于某个时刻的R(t)（李群空间），存在一个三维向量φ=（φ1，φ2，φ3）（李代数空间），用来描述R在t时刻的局部的导数。
  而旋转矩阵的微分是一个反对称(也叫斜对称)矩阵左乘它本身。
</figcaption>
</div>

## 反对称矩阵 （skew symmetric matrix）

反对称矩阵其实是将三维向量和三维矩阵建立对应关系。它是这样定义的：如果一个3 X 3的矩阵A满足如下式子,那么A就是反对称矩阵。
<div align="center">
  <img src="https://kwanwaipang.github.io/SLAM_DEMO/SLAM_Introduction/微信截图_20240810190203.png" width="30%" />
<figcaption>  
</figcaption>
</div>


反对称矩阵对角线元素都为0。形如下公式：
<div align="center">
  <img src="https://kwanwaipang.github.io/SLAM_DEMO/SLAM_Introduction/微信截图_20240810190337.png" width="30%" />
<figcaption>  
</figcaption>
</div>


因此可以定义一个三维向量，用一个上三角符号来表示这个向量α和反对称矩阵A的对应关系。进而实现了向量和矩阵的对应关系

## 指数映射
对于下公式。向量φ=（φ1，φ2，φ3）反应了R的导数性质，故称它在SO(3)上的原点 φ0 附近的正切空间上。这个φ正是李群大SO(3)对应的李代数小so(3)。
<div align="center">
  <img src="https://kwanwaipang.github.io/SLAM_DEMO/SLAM_Introduction/微信截图_20240810190755.png" width="30%" />
<figcaption> 
指数映射 
</figcaption>
</div>

李代数小so(3)是三维向量φ的集合，每个向量φi的反对称矩阵都可以表达李群(大SO(3))上旋转矩阵R的导数，而R和φ是一个指数映射关系。也就是说，李群空间的任意一个旋转矩阵R都可以用李代数空间的一个向量的反对称矩阵指数来近似。

而李代数是由向量组成的，向量对加法运算是封闭的。

<details>
<summary>结合下面手写的笔记理解🙂</summary>

<div align="center">
  <img src="https://kwanwaipang.github.io/SLAM_DEMO/SLAM_Introduction/微信截图_20240810172202.png" width="100%" />
<figcaption>  
</figcaption>
</div>

</details>



# 参考资料
* 自动驾驶与机器人中的SLAM技术
* [视觉SLAM](https://kwanwaipang.github.io/File/Blogs/Poster/%E8%A7%86%E8%A7%89SLAM.html)
