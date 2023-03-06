# ViBeTracker

使用 ViBe 作为运动目标检测器，使用 SORT 作为跟踪器，实现多目标跟踪。

## Install

下载项目到本地

```git
git clone https://github.com/kuaizhiyan/VibeTracker.git
```

## Requirements

本项目依赖环境：

- opencv4.x
- opencv-contrib
- dlib

本地编译完成的 `dlib` `opencv(+opencv-contrib)`  （Debug/Release）放置在：[Google Drive](https://drive.google.com/drive/folders/1iE0ArSiQGJjrxogbej2nU_wGOUXGcum1?usp=share_link)

自行编译可以参考的博客：

[C++ opencv-contrib配置过程]([[常用工具\] OpenCV_contrib库在windows下编译使用指南_落痕的寒假的博客-CSDN博客](https://blog.csdn.net/LuohenYJ/article/details/107944236))

[C++ dlib 配置过程](https://mo.zju.edu.cn/my_courses
https://blog.csdn.net/dawnfox/article/details/77282246)



1. 配置VC++目录->包含目录

   ```
   D:\opencv420\opencv-build\install\include\opencv2
   D:\opencv420\opencv-build\install\include
   D:\dlib-19.24\install\include
   ```

2. 配置VC++目录->库目录

```
D:\opencv420\opencv-build\install\x64\vc16\lib
D:\dlib-19.24\install\lib
```

3. 配置 链接器->输入->附加依赖项

```
opencv_world420.lib
opencv_img_hash420.lib
dlib19.24.0_release_64bit_msvc1929.lib
```

这里使用的是 Release 版本的库，使用 Debug 则自行更换，Google Drive 里提供了两个版本。



常见问题：

1. opencv 找不到 `opencv_worldxxx.dll`

> 将 ./opencv-build/install/x64/vc16/bin 下的 dll 文件复制到：c:/Windows/System32 中



## Usage

修改代码第17行，修改为本地视频路径，运行即可。

```c++
capture.open("C:\\Users\\dell\\Desktop\\testvideo\\car.avi");//输入端口
```




## Example
![image](https://user-images.githubusercontent.com/54351405/222967914-de21e767-79d8-4008-a6fa-e9d05fb6f793.png)
![image](https://user-images.githubusercontent.com/54351405/222967920-1e1c2ea3-e5ad-4543-92c4-86895aac0350.png)
![image](https://user-images.githubusercontent.com/54351405/222967925-17972a7f-c2d2-4b15-a03a-fefe13e5457e.png)

