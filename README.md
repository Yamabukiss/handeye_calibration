```c++
/*
	modified by yamabuki 2023/8/25
	在 `551e7f92d3ae50c3ea943ed0b14ad25170b59223` commit中已优化界面逻辑，文档中部分图片的按键显示情况不符合软件实际情况，不影响使用。
*/
```



## V1.0手眼标定软件使用文档

### 概述：

​	本软件适配所有深视`SRxxxx`线阵相机型号，旨在提出一种线阵相机的`ETH`手眼标定方法，本软件以`SR7900`作为示例。

### 界面介绍：

​	主界面如下图

​	![image-20230825112110903](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825112110903.png)

1. **连接**——连接相机

2. **扫图**——进行线阵相机批处理

3. **抛弃样本**

   如图

   ![image-20230825113006003](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825113006003.png)

   如果你不想要本次扫图采集的图像和特征点，点击**抛弃样本**即可，该操作会删除图像的显示和特征点如下图

   ![image-20230825113038295](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825113038295.png)

4. **更新参数**——请看参数调整章节

5. **保存参数**——会将参数保存在与可执行文件在同一目录下的`/config/handeye.json`文件中

6. **完成采集**——请看主要流程章节

7. **精度测试**——请看主要流程章节

### 主要流程：

1. 连接并且**点击扫图**处理完成后显示如下图，文本框中显示的是**点在相机物理坐标系下的表示**。

   ![image-20230825102250878](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825102250878.png)

   请**根据图像上点的顺序依次**将机械臂末端指向圆的圆心并输入坐标**然后点击完成本次输入**，**至少需要5个点对**才能够点击完成采集进行矩阵计算。

2. **你不需要输入所有点**，只要满足共计至少5个点对就可以进行矩阵计算，如上图输入了4个点，在下图中只输入了1个点，也可以解锁**完成采集按钮**。

   ![image-20230825105422759](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825105422759.png)

3. 点击**完成采集**即可计算手眼标定矩阵，显示在文本框中。

   ![image-20230825110503075](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825110503075.png)

4. 如果你要判断本次标定的优劣，点击**精度测试**。示例中**以下图点4**作为测试点

   ![image-20230825110902053](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825110902053.png)

   分别在**PBase处输入机械臂末端的点**，**在PCam中输入文本框中相机物理坐标系下的点**后点击**完成输入**即可以查看三轴绝对误差。

   ![image-20230825111220042](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825111220042.png)

   **清空输入**按钮则是清除PBase和PCam表格中的内容。

### 参数调整：

​	需要修改的参数基本上只有**投票阈值、最大半径和最小半径**，其他参数可以保留或者根据需求修改，**修改参数完成后点击更新参数按钮**。

​	以下图作为例子：

![image-20230825103515035](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825103515035.png)		

​		没有获得足够多的点，可以通过调小**投票阈值**来获取更多的点。

​		最小/大半径则对应圆的半径大小范围，没有捕获到小点，可以通过调小**最小半径**来获取更多的点，**最大半径**亦然。

​		示例中一番修改后如下图。

![image-20230825104538386](https://raw.githubusercontent.com/Yamabukiss/handeye_calibration/V1.0/Image/image-20230825104538386.png)



### 注意事项：

​	标定时请保证**原图像尺寸**与qt中的`label_gray`控件尺寸（默认是800x500）成**整数比例**。

​	此软件**依赖的标定板需要存在圆形特征**，可以使用圆形标定板或者其他**带有圆形特征**的工具。

​	配置文件中的参数是在**批处理点数为2000**下获得的，在标定时可以使用该值作为批处理点数。**请保证标定板在机械臂的工作空间内，若无法保证，请自行修改标定矩阵相应距离的偏移量**。

### 开发环境依赖：

1. `opencv 4.5.x`及以上
2. `eigen 3.4.0`及以上
3. 深视线阵相机二次开发库`SR_SdkDllx64`

### 运行环境：

​	Windows 10/11 64位操作系统