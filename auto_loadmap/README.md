# **Auto Loadmap for Vehicle** 😊
## 🎯 **Main features **
- ✅ Load maps on demand based on current location   
- ✅ Smart loading and caching
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
├── district_boundaries.yaml         
├── Hungyen/
│   ├── areas_boundaries.yaml       
│   ├── Area_A/
│   │   ├── map_boundaries.yaml     
│   │   ├── hungyen_a1.pcd
│   │   ├── hungyen_a2.pcd
│   │   └── ...
│   ├── Area_B/
│   │   ├── map_boundaries.yaml     
│   │   ├── hungyen_b1.pcd
│   │   ├── hungyen_b2.pcd
│   │   └── ...
│   └── Area_C/
│       ├── map_boundaries.yaml     
│       ├── hungyen_c1.pcd
│       └── ...
├── Hadong/
│   ├── areas_boundaries.yaml       
│   ├── Area_A/
│   │   ├── map_boundaries.yaml      
│   │   ├── hadong_a1.pcd
│   │   ├── hadong_a2.pcd
│   │   └── ...
│   ├── Area_B/
│   │   ├── map_boundaries.yaml      
│   │   ├── hadong_b1.pcd
│   │   ├── hadong_b2.pcd
│   │   └── ...
│   └── Area_C/
│       ├── map_boundaries.yaml     
│       ├── hadong_c1.pcd
│       └── ...
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

```


## 📞 **Contact**
```bash
📧 Email: khanh191600560@gmail.com  
🔗 GitHub: https://github.com/Khanh-embedded247 
💬 Facebook: https://www.facebook.com/K.h.a.n.h.24.7

```