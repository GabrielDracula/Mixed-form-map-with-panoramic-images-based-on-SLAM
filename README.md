# Mixed-form-map-with-panoramic-images-based-on-SLAM
The program can construct a mixed form map with panoramic images captured by a stereo vision system which consists of two ladybug5.
视觉里程计部分的数据结构参考了高翔博士的《视觉SLAM十四讲》，将其改为了适用于实际的全景双目立体视觉系统中应用的算法结构
稠密建图部分使用的是opencv自带的SGBM以及深度滤波器
二维球形图部分是直接使用标定信息将每一个像素点投影到球面上

本程序使用实际采集的全景双目图像。由于图像集过大，未上传至此。
