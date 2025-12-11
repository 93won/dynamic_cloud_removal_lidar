# Dynamic Cloud Removal - Design Document

A benchmark framework for dynamic object removal from LiDAR point clouds.

## Overview

This project provides a ROS-free framework to compare various dynamic object detection 
algorithms (Dynablox, DUFOMap, etc.) for LiDAR point cloud processing.

## Repository Structure

```
dynamic_cloud_removal_lidar/
│
├── docs/
│   └── DESIGN.md                  # This document
│
├── util/                          # Shared utility library
│   ├── include/util/
│   │   ├── point_cloud.h          # Point3D, PointCloud
│   │   ├── kdtree.h               # KdTree (nanoflann wrapper)
│   │   └── io.h                   # PLY/BIN/Pose loading
│   ├── src/
│   │   ├── point_cloud.cpp
│   │   └── io.cpp
│   └── CMakeLists.txt
│
├── method/                        # Algorithm implementations (libraries)
│   ├── dynablox/                  # Original Dynablox (reference)
│   ├── dynablox_standalone/       # Dynablox core library
│   │   ├── include/dynablox/
│   │   │   ├── core/
│   │   │   │   ├── types.h
│   │   │   │   ├── preprocessor.h
│   │   │   │   ├── ever_free_integrator.h
│   │   │   │   ├── clustering.h
│   │   │   │   └── tracking.h
│   │   │   └── dynablox_detector.h
│   │   ├── src/
│   │   │   ├── core/
│   │   │   │   ├── preprocessor.cpp
│   │   │   │   ├── ever_free_integrator.cpp
│   │   │   │   ├── clustering.cpp
│   │   │   │   └── tracking.cpp
│   │   │   └── dynablox_detector.cpp
│   │   └── CMakeLists.txt
│   │
│   └── dufomap_standalone/        # (Future) DUFOMap core library
│       └── ...
│
├── evaluation_tool/               # Evaluation and visualization app
│   ├── include/
│   │   └── viewer.h               # Pangolin 3D viewer
│   ├── src/
│   │   └── viewer.cpp
│   ├── app/
│   │   └── main.cpp               # Main executable
│   ├── config/
│   │   └── default.yaml           # Configuration file
│   └── CMakeLists.txt
│
├── thirdparty/                    # Shared external libraries
│   ├── nanoflann/                 # header-only KD-Tree
│   ├── pangolin/                  # 3D viewer
│   └── voxblox/                   # TSDF mapping
│
├── CMakeLists.txt                 # Root CMake
└── README.md
```

## Dependencies

### Thirdparty Libraries
| Library | Purpose | Type |
|---------|---------|------|
| nanoflann | KD-Tree (nearest neighbor search) | header-only |
| pangolin | 3D visualization | build required |
| voxblox | TSDF mapping | build required |
| Eigen3 | Linear algebra | system package |
| yaml-cpp | Configuration parsing | system package |

---

# Part 1: Util Library

Shared utilities used by all methods.

## util/point_cloud.h

```cpp
namespace util {

struct Point3D {
    float x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Eigen::Vector3f toEigen() const { return {x, y, z}; }
    static Point3D fromEigen(const Eigen::Vector3f& v) { return {v.x(), v.y(), v.z()}; }
    
    float norm() const { return std::sqrt(x*x + y*y + z*z); }
    float squaredNorm() const { return x*x + y*y + z*z; }
    
    Point3D operator+(const Point3D& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Point3D operator-(const Point3D& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Point3D operator*(float s) const { return {x*s, y*s, z*s}; }
};

class PointCloud {
public:
    using Ptr = std::shared_ptr<PointCloud>;
    using ConstPtr = std::shared_ptr<const PointCloud>;
    
    // Basic operations
    void push_back(const Point3D& p);
    void push_back(float x, float y, float z);
    size_t size() const;
    bool empty() const;
    void clear();
    void reserve(size_t n);
    
    // Access
    Point3D& operator[](size_t i);
    const Point3D& operator[](size_t i) const;
    
    // Transformations
    void transform(const Eigen::Matrix4f& T);
    PointCloud::Ptr transformedCopy(const Eigen::Matrix4f& T) const;
    
    // Iterators
    auto begin() { return points_.begin(); }
    auto end() { return points_.end(); }
    auto begin() const { return points_.begin(); }
    auto end() const { return points_.end(); }
    
    // Internal data access (for Voxblox compatibility)
    std::vector<Point3D>& points() { return points_; }
    const std::vector<Point3D>& points() const { return points_; }

private:
    std::vector<Point3D> points_;
};

}  // namespace util
```

## util/kdtree.h

```cpp
namespace util {

// nanoflann adapter
struct PointCloudAdapter {
    const PointCloud& cloud;
    
    explicit PointCloudAdapter(const PointCloud& c) : cloud(c) {}
    
    size_t kdtree_get_point_count() const { return cloud.size(); }
    
    float kdtree_get_pt(size_t idx, size_t dim) const {
        const auto& p = cloud[idx];
        return dim == 0 ? p.x : (dim == 1 ? p.y : p.z);
    }
    
    template<class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

class KdTree {
public:
    void setInputCloud(PointCloud::ConstPtr cloud);
    
    // K-nearest neighbors
    size_t knnSearch(const Point3D& query, size_t k,
                     std::vector<size_t>& indices,
                     std::vector<float>& distances) const;
    
    // Radius search
    size_t radiusSearch(const Point3D& query, float radius,
                        std::vector<size_t>& indices,
                        std::vector<float>& distances) const;

private:
    using Tree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdapter>,
        PointCloudAdapter, 3>;
    
    std::unique_ptr<PointCloudAdapter> adapter_;
    std::unique_ptr<Tree> tree_;
};

}  // namespace util
```

## util/io.h

```cpp
namespace util {

// Point cloud I/O
PointCloud::Ptr loadPointCloudPLY(const std::string& filename);
PointCloud::Ptr loadPointCloudBIN(const std::string& filename);  // KITTI format
bool savePointCloudPLY(const std::string& filename, const PointCloud& cloud);

// Pose I/O
std::vector<Eigen::Matrix4f> loadPosesKITTI(const std::string& filename);
std::vector<Eigen::Matrix4f> loadPosesTUM(const std::string& filename);

// Scan file listing
std::vector<std::string> listScanFiles(const std::string& directory, 
                                        const std::string& extension = ".bin");

}  // namespace util
```

---

# Part 2: Dynablox Standalone

Dynablox core library - Voxblox-based dynamic object detection.

## Algorithm Overview

Dynablox detects dynamic objects in real-time using the **Ever-Free Space** concept:
1. Estimate high-confidence free space by modeling sensor noise, sparsity, and drift
2. Mark voxels as "ever-free" if they remain unoccupied for a burn-in period
3. Detect dynamics when ever-free voxels suddenly become occupied
4. Cluster and track detected dynamic points

## dynablox/core/types.h

```cpp
namespace dynablox {

// Voxblox types (re-export)
using TsdfVoxel = voxblox::TsdfVoxel;
using TsdfBlock = voxblox::Block<TsdfVoxel>;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;
using BlockIndex = voxblox::BlockIndex;
using VoxelIndex = voxblox::VoxelIndex;
using VoxelKey = std::pair<BlockIndex, VoxelIndex>;

// Point info for each point
struct PointInfo {
    bool ready_for_evaluation = false;
    bool ever_free_level_dynamic = false;
    bool cluster_level_dynamic = false;
    bool object_level_dynamic = false;
    float distance_to_sensor = -1.0f;
    bool ground_truth_dynamic = false;
};

// Cloud metadata
struct CloudInfo {
    uint64_t timestamp = 0;
    util::Point3D sensor_position;
    std::vector<PointInfo> points;
};

// Cluster representation
struct Cluster {
    int id = -1;
    int track_length = 0;
    bool valid = false;
    std::vector<size_t> point_indices;
    util::Point3D centroid;
    float extent = 0.0f;
};

using Clusters = std::vector<Cluster>;

// Voxel to point mapping
using VoxelToPointMap = std::unordered_map<VoxelIndex, std::vector<size_t>>;
using BlockToPointMap = std::unordered_map<BlockIndex, VoxelToPointMap>;

}  // namespace dynablox
```

## dynablox/core/preprocessor.h

```cpp
namespace dynablox {

class Preprocessor {
public:
    struct Config {
        float min_range = 0.5f;   // Minimum range [m]
        float max_range = 20.0f;  // Maximum range [m]
    };

    explicit Preprocessor(const Config& config);

    // Preprocess point cloud: range filtering + coordinate transformation
    void process(const util::PointCloud& input,
                 const Eigen::Matrix4f& pose,
                 util::PointCloud& output,
                 CloudInfo& cloud_info);

private:
    Config config_;
};

}  // namespace dynablox
```

## dynablox/core/ever_free_integrator.h

```cpp
namespace dynablox {

class EverFreeIntegrator {
public:
    struct Config {
        int neighbor_connectivity = 18;     // Neighbor connectivity (6, 18, 26)
        int counter_to_reset = 150;         // Observations to un-free an ever-free voxel
        int temporal_buffer = 2;            // Sparsity compensation buffer
        int burn_in_period = 5;             // Frames before becoming ever-free
        float tsdf_occupancy_threshold = 0.3f;  // Occupancy threshold
        int num_threads = 4;                // Number of parallel threads
    };

    EverFreeIntegrator(const Config& config, TsdfLayer::Ptr tsdf_layer);

    // Update ever-free state for current frame
    void updateEverFreeVoxels(int frame_counter);

private:
    // Block-wise processing
    bool blockWiseUpdateEverFree(const BlockIndex& block_index, 
                                  int frame_counter,
                                  std::vector<VoxelKey>& voxels_to_remove);
    
    // Update occupancy counter
    void updateOccupancyCounter(TsdfVoxel& voxel, int frame_counter);
    
    // Remove ever-free status (including neighbors)
    std::vector<VoxelKey> removeEverFree(TsdfBlock& block, 
                                          TsdfVoxel& voxel,
                                          const BlockIndex& block_idx, 
                                          const VoxelIndex& voxel_idx);
    
    // Mark voxel as ever-free
    void blockWiseMakeEverFree(const BlockIndex& block_index, int frame_counter);

    Config config_;
    TsdfLayer::Ptr tsdf_layer_;
    float voxel_size_;
    size_t voxels_per_side_;
};

}  // namespace dynablox
```

## dynablox/core/clustering.h

```cpp
namespace dynablox {

class Clustering {
public:
    struct Config {
        int min_cluster_size = 20;
        int max_cluster_size = 200000;
        float min_extent = 0.0f;
        float max_extent = 200000.0f;
        int neighbor_connectivity = 6;
        bool grow_clusters_twice = false;
        float min_cluster_separation = 0.2f;
    };

    Clustering(const Config& config, TsdfLayer::Ptr tsdf_layer);

    // Extract clusters from occupied ever-free voxels
    Clusters performClustering(const BlockToPointMap& point_map,
                               const std::vector<VoxelKey>& occupied_ever_free_voxels,
                               int frame_counter,
                               const util::PointCloud& cloud,
                               CloudInfo& cloud_info);

private:
    // Voxel-based clustering
    std::vector<std::vector<VoxelKey>> voxelClustering(
        const std::vector<VoxelKey>& seeds, int frame_counter);
    
    // Grow single cluster from seed
    void growCluster(const VoxelKey& seed, 
                     int frame_counter,
                     std::vector<VoxelKey>& result);

    Config config_;
    TsdfLayer::Ptr tsdf_layer_;
};

}  // namespace dynablox
```

## dynablox/core/tracking.h

```cpp
namespace dynablox {

class Tracking {
public:
    struct Config {
        int min_track_duration = 0;         // Minimum tracking frames
        float max_tracking_distance = 1.0f; // Maximum tracking distance [m]
    };

    explicit Tracking(const Config& config);

    // Track clusters across frames
    void track(const util::PointCloud& cloud, 
               Clusters& clusters, 
               CloudInfo& cloud_info);

private:
    Config config_;
    Clusters previous_clusters_;
    int next_cluster_id_ = 0;
};

}  // namespace dynablox
```

## dynablox/dynablox_detector.h

```cpp
namespace dynablox {

class DynabloxDetector {
public:
    struct Config {
        // Voxblox
        float voxel_size = 0.2f;
        float truncation_distance = 0.4f;
        int voxels_per_side = 16;
        
        // Sub-module configs
        Preprocessor::Config preprocessor;
        EverFreeIntegrator::Config ever_free;
        Clustering::Config clustering;
        Tracking::Config tracking;
    };

    explicit DynabloxDetector(const Config& config);
    
    // Main processing function
    // Returns: CloudInfo with dynamic/static labels for each point
    CloudInfo processPointCloud(const util::PointCloud& cloud, 
                                const Eigen::Matrix4f& pose);
    
    // Access TSDF layer (for visualization)
    TsdfLayer::Ptr getTsdfLayer() const { return tsdf_layer_; }
    
    // Get last detected clusters
    const Clusters& getLastClusters() const { return last_clusters_; }

private:
    Config config_;
    int frame_counter_ = 0;
    
    // Voxblox
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    TsdfLayer::Ptr tsdf_layer_;
    
    // Sub-modules
    std::unique_ptr<Preprocessor> preprocessor_;
    std::unique_ptr<EverFreeIntegrator> ever_free_integrator_;
    std::unique_ptr<Clustering> clustering_;
    std::unique_ptr<Tracking> tracking_;
    
    // Results
    Clusters last_clusters_;
    
    // Internal methods
    void buildPointMap(const util::PointCloud& cloud,
                       BlockToPointMap& point_map,
                       std::vector<VoxelKey>& occupied_ever_free_voxels,
                       CloudInfo& cloud_info);
                       
    void integrateTsdf(const util::PointCloud& cloud,
                       const Eigen::Matrix4f& pose);
};

}  // namespace dynablox
```

## Data Flow (Pipeline)

```
1. Load point cloud & pose
   │
   ▼
2. Preprocessor::process()
   - Range filtering (min_range ~ max_range)
   - Transform to world frame (apply pose)
   - Initialize CloudInfo
   │
   ▼
3. Build point-to-voxel map
   - Map each point to its voxel
   - Collect occupied ever-free voxels
   │
   ▼
4. Clustering::performClustering()
   - Seed from occupied ever-free voxels
   - Expand clusters to neighboring voxels
   - Filter by cluster size/extent
   │
   ▼
5. Tracking::track()
   - Match with previous frame clusters
   - Assign tracking IDs
   - Check track duration
   │
   ▼
6. EverFreeIntegrator::updateEverFreeVoxels()
   - Update occupancy counters
   - Set/unset ever-free status based on conditions
   │
   ▼
7. TsdfIntegrator::integratePointCloud()
   - Update TSDF map
   │
   ▼
8. Return CloudInfo with labels
   - ever_free_level_dynamic
   - cluster_level_dynamic
   - object_level_dynamic
```

---

# Part 3: Evaluation Tool

Evaluation and visualization application.

## viewer.h

```cpp
namespace evaluation {

class Viewer {
public:
    Viewer();
    ~Viewer();
    
    bool initialize(int width = 1280, int height = 960);
    void shutdown();
    bool shouldClose() const;
    
    // Cloud updates
    void setStaticCloud(util::PointCloud::ConstPtr cloud);   // Gray
    void setDynamicCloud(util::PointCloud::ConstPtr cloud);  // Red
    void setMapCloud(util::PointCloud::ConstPtr cloud);      // Blue
    
    // Trajectory
    void addPose(const Eigen::Matrix4f& pose);
    void clearTrajectory();
    
    // Camera control
    void followPose(const Eigen::Matrix4f& pose);
    void resetCamera();

private:
    void renderLoop();
    
    std::thread render_thread_;
    std::atomic<bool> should_close_{false};
    std::mutex data_mutex_;
    
    // Render data
    util::PointCloud::ConstPtr static_cloud_;
    util::PointCloud::ConstPtr dynamic_cloud_;
    util::PointCloud::ConstPtr map_cloud_;
    std::vector<Eigen::Matrix4f> trajectory_;
};

}  // namespace evaluation
```

## app/main.cpp Structure

```cpp
int main(int argc, char** argv) {
    // 1. Parse arguments
    std::string config_path = "config/default.yaml";
    std::string data_path = "/path/to/dataset";
    
    // 2. Load config
    YAML::Node config = YAML::LoadFile(config_path);
    
    // 3. Setup detector
    dynablox::DynabloxDetector::Config detector_config;
    // ... load from yaml
    dynablox::DynabloxDetector detector(detector_config);
    
    // 4. Setup viewer
    evaluation::Viewer viewer;
    viewer.initialize();
    
    // 5. Load data
    auto scan_files = util::listScanFiles(data_path + "/scans");
    auto poses = util::loadPosesKITTI(data_path + "/poses.txt");
    
    // 6. Process loop
    for (size_t i = 0; i < scan_files.size() && !viewer.shouldClose(); ++i) {
        // Load point cloud
        auto cloud = util::loadPointCloudBIN(scan_files[i]);
        
        // Process
        auto cloud_info = detector.processPointCloud(*cloud, poses[i]);
        
        // Separate static/dynamic
        auto static_cloud = std::make_shared<util::PointCloud>();
        auto dynamic_cloud = std::make_shared<util::PointCloud>();
        
        for (size_t j = 0; j < cloud->size(); ++j) {
            if (cloud_info.points[j].object_level_dynamic) {
                dynamic_cloud->push_back((*cloud)[j]);
            } else {
                static_cloud->push_back((*cloud)[j]);
            }
        }
        
        // Update viewer
        viewer.setStaticCloud(static_cloud);
        viewer.setDynamicCloud(dynamic_cloud);
        viewer.addPose(poses[i]);
        viewer.followPose(poses[i]);
        
        // Delay for visualization
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    viewer.shutdown();
    return 0;
}
```

---

# Part 4: Build System

## Root CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(dynamic_cloud_removal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# System dependencies
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Threads REQUIRED)

# Thirdparty
set(THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/thirdparty)

# Build Pangolin
set(BUILD_PANGOLIN_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PANGOLIN_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(${THIRDPARTY_DIR}/pangolin ${CMAKE_BINARY_DIR}/pangolin)

# Build Voxblox (standalone mode)
add_subdirectory(${THIRDPARTY_DIR}/voxblox ${CMAKE_BINARY_DIR}/voxblox)

# Include nanoflann (header-only)
include_directories(${THIRDPARTY_DIR}/nanoflann/include)

# Build util library
add_subdirectory(util)

# Build method libraries
add_subdirectory(method/dynablox_standalone)
# add_subdirectory(method/dufomap_standalone)  # Future

# Build evaluation tool
add_subdirectory(evaluation_tool)
```

## util/CMakeLists.txt

```cmake
add_library(util
    src/point_cloud.cpp
    src/io.cpp
)

target_include_directories(util PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(util
    Eigen3::Eigen
)
```

## method/dynablox_standalone/CMakeLists.txt

```cmake
add_library(dynablox_standalone
    src/core/preprocessor.cpp
    src/core/ever_free_integrator.cpp
    src/core/clustering.cpp
    src/core/tracking.cpp
    src/dynablox_detector.cpp
)

target_include_directories(dynablox_standalone PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dynablox_standalone
    util
    voxblox
    Eigen3::Eigen
)
```

## evaluation_tool/CMakeLists.txt

```cmake
add_library(viewer
    src/viewer.cpp
)

target_include_directories(viewer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(viewer
    util
    pangolin
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    Threads::Threads
)

add_executable(evaluate app/main.cpp)

target_link_libraries(evaluate
    dynablox_standalone
    # dufomap_standalone  # Future
    viewer
    yaml-cpp
)
```

---

# Configuration (default.yaml)

```yaml
dynablox:
  # Voxblox
  voxel_size: 0.2
  truncation_distance: 0.4
  voxels_per_side: 16
  
  # Preprocessing
  preprocessor:
    min_range: 0.5
    max_range: 20.0
  
  # Ever-Free Integration
  ever_free:
    counter_to_reset: 150
    temporal_buffer: 2
    burn_in_period: 5
    tsdf_occupancy_threshold: 0.3
    neighbor_connectivity: 26
  
  # Clustering
  clustering:
    min_cluster_size: 20
    max_cluster_size: 200000
    min_extent: 0
    max_extent: 200000
    neighbor_connectivity: 6
    grow_clusters_twice: false
    min_cluster_separation: 0.2
  
  # Tracking
  tracking:
    min_track_duration: 0
    max_tracking_distance: 1.0
```

---

# Implementation Order

## Phase 1: Foundation
1. Create project folder structure
2. Setup thirdparty (clone nanoflann, pangolin, voxblox)
3. Write root CMakeLists.txt

## Phase 2: Util Library
4. Write util/point_cloud.h/cpp
5. Write util/kdtree.h (header-only)
6. Write util/io.h/cpp
7. Test util build

## Phase 3: Dynablox Core
8. Write dynablox/core/types.h
9. Implement dynablox/core/preprocessor
10. Implement dynablox/core/ever_free_integrator (core algorithm!)
11. Implement dynablox/core/clustering
12. Implement dynablox/core/tracking
13. Integrate dynablox/dynablox_detector
14. Test dynablox_standalone build

## Phase 4: Evaluation Tool
15. Write viewer.h/cpp (Pangolin)
16. Write app/main.cpp
17. Full build and test

## Phase 5: (Future)
18. Implement DUFOMap standalone
19. Add evaluation metrics
20. Benchmark scripts

---

# References

- Original Dynablox: https://github.com/ethz-asl/dynablox
- Voxblox: https://github.com/ethz-asl/voxblox
- DUFOMap: https://github.com/KTH-RPL/dufomap
- Dynablox Paper: "Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments" (RA-L 2023)
