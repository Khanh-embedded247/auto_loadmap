# **Auto Loadmap for Vehicle** 😊
## 🎯 **Main features **
- ✅ Load maps on demand based on current location   
- ✅ Lazy loading và caching thông minh 
- ✅ Automatically update map when location changes

---

## 🚀 **Requirements**

Divide the large map into small maps with the form:
```bash
GRID_SIZE = 200.0 meters
MIN_POINTS_THRESHOLD = 10000

*Merge grids with too few points
```

```bash
<!--base_path=/root/simulation/src/pnkx_map/config-->
base_path/
├── hadong/
│   ├── pcd/
│   │   ├── file1.pcd
│   │   ├── file2.pcd
│   │   └── ...
│   ├── map_boundaries.yaml/
|
├── thanhxuan/
│   ├── pcd/
│   │   ├── file1.pcd
│   │   ├── file2.pcd
│   │   └── ...
│   ├── map_boundaries.yaml/
|
├── namtuliem/
│   ├── pcd/
│   │   ├── file1.pcd
│   │   ├── file2.pcd
│   │   └── ...
│   ├── map_boundaries.yaml/
|.............
```
***Prerequisite: enter the boundaries of major areas: Ha Dong, Thanh Hoa, Xuan, Hanoi***

**Input** : 
```bash
- map_path: .../config/
- Topic: 
-── /gnss_pose_cov : geometry_msgs::PoseWithCovarianceStamped
├── /point_ref : sensor_msgs::NavSatFix
├── /odom_MGRS : geometry_msgs::PoseStamped
```
```bash
- Topic: 
-── /map/current_map : sensor_msgs::PointCloud2 (only view)
├── /tf_map_enu : geometry_msgs::TransformStamped
├── /odom_MGRS : geometry_msgs::PoseStamped
```

```bash
- Service: 
-── points_update_service : Send pointclouds of maps to add or remove
├── monte_align_srv : Find the initial starting point
```


## 📞 **Contact**
```bash
📧 Email: khanh191600560@gmail.com  
🔗 GitHub: https://github.com/Khanh-embedded247 
💬 Facebook: https://www.facebook.com/K.h.a.n.h.24.7

```