# UaV-NeRF依赖ROS参数说明
## 字段说明
```
{drone_id}: 无人机编号
```

## sensor node
### **彩色图** 
**topic**: /airsim_node/uav{drone_id}/uav{drone_id}_scene_camera/Scene   
**type**: sensor_msgs/Image   
**note**: 彩色相机图像

### **深度图**  
**topic**: /airsim_node/uav{drone_id}/uav{drone_id}_depth_camera_vis/DepthVis   
**type**: sensor_msgs/Image   
**note**: 深度相机图像

### **相机位姿**   
**topic**: /airsim_node/uav{drone_id}/uav{drone_id}_depth_camera/pose  
**type**: geometry_msgs/PoseStamped   
**note**: 深度相机在世界坐标系下的坐标信息，当中pose.position字段表示深度相机所处位置三维空间坐标，pose.orientation字段表示深度相机视见姿态（z轴向内）。该值作为传感器估计结果，并不是真实值。（深度相机与彩色相机处于同一位姿，读哪一个都是一样的）

### **基础相机数据**   
**topic**: /airsim_node/uav{drone_id}/uav{drone_id}_depth_camera_vis/DepthVis/camera_info   
**type**: sensor_msgs/CameraInfo   
**note**:
- principal_point, focal_length, k1, k2, p1, p2: 从D,K中读出
- resolution: width, height

### **更多相机数据**
- depth_range_meter: ROS参数服务器中`/airsim_node/depth_range_meter`，默认100，也可以直接写死为100
- depth_bit_depth: 写死为8
