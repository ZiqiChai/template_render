# 摘要

`template_render`用于使用三维模型生成彩色图和深度图的模板。

## 代码组织

1. UV Sphere Render：用于通过UV对球面划分，进行渲染视点信息采样，进而在每个视点渲染6D自由度模板（彩色图和深度图）。模板视点参数为：r,alpha,beta,gamma。

2. ICO Sphere Render：用于通过ICO对球面划分，进行渲染视点信息采样，进而在每个视点渲染6D自由度模板（彩色图和深度图）。模板视点参数为：r,number,gamma。

3. OCT Sphere Render：用于通过OCT对球面划分，进行渲染视点信息采样，进而在每个视点渲染6D自由度模板（彩色图和深度图）。模板视点参数为：r,number,gamma。

4. Random Generation：用于生成随机分布的测试样例。

以正二十面体递归细分举例：

1. ICO_Sphere_Render_6D_original：生成640x480分辨率模板。
2. ICO_Sphere_Render_6D_WxH：生成WxH分辨率模板。
3. ICO_Sphere_Render_6D_minimum：生成最小包围框分辨率模板。

## 程序依赖项

1. OpenGL
2. OpenCV
3. clang (optianal:sudo apt install clang)

## 程序运行

### 生成r,alpgha,beta,gamma参数化模板

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roscd template_render/
rosrun template_render uv_render_6d_original parameter-pvc-uv.yml 
```

### 生成20面体递归的均匀分布模板

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roscd template_render/
rosrun template_render ico_render_6d_original parameter-pvc-ico.yml 
```

## 节点的输入和输出

渲染模板的参数在`parameter-xxx.yml`文件内。
可以通过参数调整，完成5D或者6D渲染。

1. uv_render_6d_original
    - 输入:
        - 命令行参数
    - 输出:
        - 彩色图,格式:bmp,8位3通道
        - 深度图,格式:png,16位单通道
        - 深度图,格式:dpt,二进制

2. uv_render_6d_WxH
    - 输入:
        - 命令行参数,渲染图片的宽度和高度
    - 输出:
        - 彩色图,格式:bmp,8位3通道
        - 深度图,格式:png,16位单通道
        - 深度图,格式:dpt,二进制
        - txt文件,TXT file used to store "img_Name, label, r, num, gamma, xyz

3. ico_render_6d_original
    - 输入:
        - 命令行参数
    - 输出:
        - 彩色图,格式:bmp,8位3通道
        - 深度图,格式:png,16位单通道
        - 深度图,格式:dpt,二进制
        - xml文件,XML used to store coordinates of  icosohedron
        - txt文件,TXT file used to store "img_Name, label, r, num, gamma, xyz

4. ico_render_6d_WxH
    - 输入:
        - 命令行参数,渲染图片的宽度和高度
    - 输出:
        - 彩色图,格式:bmp,8位3通道
        - 深度图,格式:png,16位单通道
        - 深度图,格式:dpt,二进制
        - xml文件,XML used to store coordinates of  icosohedron
        - txt文件,TXT file used to store "img_Name, label, r, num, gamma, xyz

5. oct_render_6d
    - 输入:
        - 命令行参数
    - 输出:
        - 彩色图,格式:bmp,8位3通道
        - 深度图,格式:png,16位单通道
        - 深度图,格式:dpt,二进制
        - xml文件,XML used to store coordinates of  icosohedron
        - txt文件,TXT file used to store "img_Name, label, r, num, gamma, xyz
    - 备注：
        - 目前仅支持5d渲染，尚未调整gamma角度遍历

6. random_generation
    - 输入(可选):
        - xml文件，input_xml_file_name_for_random_generater，用于读取XML文件内的视点坐标，并渲染图片。
    - 输出:
        - 彩色图,格式:bmp,8位3通道
        - 深度图,格式:png,16位单通道
        - 深度图,格式:dpt,二进制
        - xml文件,random_scene_xml_file_name，保存渲染相机视点位置XYZ坐标，可选（当generate_model=2的时候，gamma角度）

    - 备注：
        - 目前支持liner，random，ico三种采样模式进行渲染，尚不支持oct模式，头文件使用的是icosphere.h

## 其他注意事项

1. 在渲染模板原始图片，提取特征，匹配过程中三个过程需要已知文件名的格式：

    1. 渲染模板，保存原图片，需要文件名
    2. 提取特征，读取原图像，需要文件名
    3. 查看结果，可视化匹配，需要文件名

    因此必须统一格式，避免不必要的麻烦，文件名规范约定如下：

    - ICO：Radius*+Number*+Gamma*.bmp/dpt/png

    - UV: Radius*+Alpha*+Beta*+Gamma*.bmp/dpt/png

2. 坐标系方向规定：

    gamma沿着相机坐标系，面向Z轴，逆时针转动相机坐标系为正，也就是gamma为正数的时候，相片在图像平面逆时针旋转。
