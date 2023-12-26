## 文件结构更新说明（黄色修改，红色删除，绿色新增）：
### 版本v1.0：
  0.images: 保存生成的图片等相关文件
  1.imgSave：保存现有的标定图和对应tcp位姿信息（自动生成）
  2.collect_cal_img_and_tcp:获取标定图片和对应tcp位姿信息
  3.realsense2：定义realsense类
  4.capture：获取realsense实时图像
  5.iter_capture:定义realsense实时（并行线程）图像
  6.utils:获取三维点云函数
  7.calibrate：定义calibrate类
  8.eye-in-hand:眼在手标定
  9.eye-to-hand：眼在外标定

### 版本v1.1：
  0.images: 保存生成的图片等相关文件
   <font color="#FF0000">1.~~imgSave：保存现有的标定图和对应tcp位姿信息（自动生成）~~</font>
  2.collect_cal_img_and_tcp:获取标定图片和对应tcp位姿信息
  3.realsense2：定义realsense类
  4.capture：获取realsense实时图像
  5.iter_capture:定义realsense实时（并行线程）图像
  <font color="#FF9912">6.utils:工具类</font>
  7.calibrate：定义calibrate类
  8.eye-in-hand:眼在手标定
  9.eye-to-hand：眼在外标定
  <font color="#00BB00">
  10.hand_eye_calibration：自动标定（realsense+xarm6）
  11.camera：realsense或者其他相机函数
  12.wrap_xarm：定义xarm类
  </font>