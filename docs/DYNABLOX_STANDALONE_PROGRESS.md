# Dynablox Standalone Implementation Progress

## Overview

This document tracks the progress of creating a **ROS-free Dynablox implementation** that uses **Voxblox TSDF** exactly like the original Dynablox paper and code.

## Key Decision: Using Modified Voxblox Branch

The original Dynablox uses a **modified Voxblox branch** (`dynablox/release`) that extends `TsdfVoxel` with additional fields for dynamic detection:

```cpp
// Extended TsdfVoxel fields in dynablox/release branch
struct TsdfVoxel {
    // Standard fields
    float distance;
    float weight;
    Color color;
    
    // Dynablox extensions
    bool ever_free = false;           // Voxel confirmed as free space
    uint32_t last_lidar_occupied = 0; // Last frame a LiDAR point fell here
    uint32_t last_occupied = 0;       // Last frame voxel was occupied
    uint8_t occ_counter = 0;          // Consecutive occupied frame counter
    bool dynamic = false;             // Contains dynamic object
    bool clustering_processed = false; // Processing flag for clustering
};
```

**Repository**: `https://github.com/ethz-asl/voxblox.git`  
**Branch**: `dynablox/release`

## Completed Tasks

### 1. ✅ Voxblox Branch Switch
- Switched `/thirdparty/voxblox` to `dynablox/release` branch
- Verified extended `TsdfVoxel` fields are available
- Voxblox builds successfully

### 2. ✅ Core Types (`types.h`)
Location: `/method/dynablox_standalone/include/dynablox/types.h`

Defines ROS-free types using Voxblox:
- `VoxelIndex`, `BlockIndex`, `TsdfVoxel`, `TsdfBlock`, `TsdfLayer`
- `Pointcloud`, `Point`, `Transformation` (from Voxblox/kindr)
- `BlockToPointMap` - Maps blocks to voxels to point indices
- `CloudInfo`, `CloudInfoPoint` - Per-point processing info
- `Cluster` - Dynamic cluster with bounding box and point indices
- `DetectionResult` - Frame detection output

### 3. ✅ Configuration (`config.h`)
Location: `/method/dynablox_standalone/include/dynablox/config.h`

Configuration structures matching original Dynablox params:
- `PreprocessingConfig`: min_range=0.5, max_range=20.0
- `EverFreeConfig`: counter_to_reset=150, temporal_buffer=2, burn_in_period=5
- `TsdfConfig`: voxel_size=0.2, voxels_per_side=16
- `ClusteringConfig`: min_cluster_size=25, max_cluster_size=2500
- `TrackingConfig`: min_track_duration=0, max_tracking_distance=1.0
- YAML loading support via `DynabloxConfig::fromYaml()`

### 4. ✅ Neighborhood Search (`neighborhood_search.h`)
Location: `/method/dynablox_standalone/include/dynablox/neighborhood_search.h`

Header-only implementation using Voxblox's `Neighborhood<Connectivity>`:
- Supports 6, 18, 26 connectivity
- `search()` returns neighbor voxel keys across block boundaries
- Used by both EverFreeIntegrator and Clustering

### 5. ✅ Ever-Free Integrator
Location: 
- `/method/dynablox_standalone/include/dynablox/ever_free_integrator.h`
- `/method/dynablox_standalone/src/ever_free_integrator.cpp`

Based on original Dynablox logic:
- `updateEverFreeVoxels()` - Main entry point, processes all updated blocks
- `blockWiseUpdateEverFree()` - Per-block processing with occupancy counter
- `updateOccupancyCounter()` - Increment/reset based on TSDF distance
- `removeEverFree()` - Remove ever-free status from voxel and neighbors
- `blockWiseMakeEverFree()` - Label voxels as ever-free after burn-in
- Multi-threaded using `std::async`

### 6. ✅ Clustering (Partial)
Location:
- `/method/dynablox_standalone/include/dynablox/clustering.h`
- `/method/dynablox_standalone/src/clustering.cpp`

Voxel-based region growing from ever-free seeds:
- `performClustering()` - Main entry point
- `voxelClustering()` - Cluster voxels from occupied ever-free seeds
- `growCluster()` - Region growing using neighborhood search
- `inducePointClusters()` - Map voxel clusters to point indices
- `applyClusterLevelFilters()` - Size and extent filtering
- `setClusterLevelDynamicFlag()` - Mark points as dynamic

### 7. ✅ Motion Detector (Main Class)
Location:
- `/method/dynablox_standalone/include/dynablox/motion_detector.h`
- `/method/dynablox_standalone/src/motion_detector.cpp`

Main detector combining TSDF + EverFree + Clustering:
- Uses `voxblox::TsdfMap` and `voxblox::FastTsdfIntegrator`
- `processPointcloud()` - Main processing pipeline
- `buildBlockToPointMap()` - Build voxel-to-point mapping AND set `last_lidar_occupied`
- `findOccupiedEverFreeVoxels()` - Find seeds for clustering
- `preprocessPointcloud()` - Range filtering

### 8. ✅ Dynablox Runner Updated
Location: `/evaluation_tool/src/dynablox_runner.cpp`

Updated to use new `MotionDetector` class:
- Uses `dynablox::Transformation` (kindr-based)
- Visualization with Pangolin
- Batch mode for processing all frames

### 9. ✅ CMakeLists.txt Updated
Location: `/method/dynablox_standalone/CMakeLists.txt`

- Links against `voxblox` library
- Uses pkg-config for glog
- Includes Voxblox headers

## Build Status

```bash
# Current build commands
cd /home/eugene/dynamic_cloud_removal_lidar/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make dynablox_standalone -j$(nproc)  # ✅ Builds successfully
make dynablox_runner -j$(nproc)       # ✅ Builds successfully
```

## Current Issue: No Dynamic Points Detected

### Problem
Running `dynablox_runner` shows 0 dynamic points for all frames.

### Root Cause Analysis
The `last_lidar_occupied` field must be set **before** TSDF integration for each point that hits a voxel. The original Dynablox does this in its preprocessing step.

### Solution (Implemented but needs verification)
Modified `buildBlockToPointMap()` to set `last_lidar_occupied = frame_counter_` for each voxel that contains points. This should happen AFTER TSDF integration but the timing needs to be verified.

The correct order should be:
1. Preprocess point cloud (range filter)
2. Transform to global frame
3. Build block-to-point map AND set `last_lidar_occupied`
4. Integrate into TSDF
5. Update ever-free voxels
6. Find occupied ever-free seeds (check `ever_free && last_lidar_occupied == frame`)
7. Perform clustering

## Remaining Tasks

### 1. 🔄 Debug Dynamic Detection
- Verify `last_lidar_occupied` is being set correctly
- Verify ever-free voxels are being labeled after burn-in period
- Add debug logging to trace the pipeline

### 2. ⬜ Verify Detection Results
- Compare with ground truth labels
- Check if dynamic ratio is reasonable (should be ~1-5% for urban scenes)

### 3. ⬜ Performance Optimization
- Profile the pipeline
- Consider parallelizing more operations

### 4. ⬜ Evaluation Metrics
- Implement precision/recall calculation
- Compare with ground truth dynamic labels

## File Structure

```
method/dynablox_standalone/
├── CMakeLists.txt
├── include/dynablox/
│   ├── types.h               ✅
│   ├── config.h              ✅
│   ├── neighborhood_search.h ✅
│   ├── ever_free_integrator.h ✅
│   ├── clustering.h          ✅
│   └── motion_detector.h     ✅
└── src/
    ├── ever_free_integrator.cpp ✅
    ├── clustering.cpp           ✅
    └── motion_detector.cpp      ✅

evaluation_tool/src/
└── dynablox_runner.cpp       ✅

thirdparty/voxblox/           ✅ (dynablox/release branch)

config/
└── dynablox_default.yaml     ✅
```

## Test Data

- **Dataset**: Hauptgebäude sequence 1
- **Location**: `/home/eugene/data/Dynamic/hauptgebaeude/sequence_1/extracted/`
- **Frames**: 100 extracted PLY files with poses and labels
- **Labels**: Ground truth dynamic point indices

## Key Parameters (from original Dynablox)

```yaml
ever_free_integrator:
  counter_to_reset: 150      # Frames before ever-free is revoked
  temporal_buffer: 2         # Frames tolerance for recent occupation
  burn_in_period: 5          # Frames before labeling ever-free
  tsdf_occupancy_threshold: 0.3

preprocessing:
  max_range: 20.0            # Max LiDAR range in meters

clustering:
  min_cluster_size: 25       # Min points per cluster
  max_cluster_size: 2500     # Max points per cluster
  min_extent: 0.2            # Min bounding box diagonal
  max_extent: 5.0            # Max bounding box diagonal
  neighbor_connectivity: 6   # 6, 18, or 26

tsdf:
  voxel_size: 0.2            # TSDF voxel size in meters
```

## Next Steps (Priority Order)

1. **Debug Detection**: Add logging to verify each stage of the pipeline
2. **Verify Timing**: Ensure `last_lidar_occupied` is set at the right time
3. **Test with Visualization**: Use the Pangolin viewer to see if clusters appear
4. **Compare with GT**: Calculate precision/recall against ground truth

---

*Last Updated: December 11, 2025*
