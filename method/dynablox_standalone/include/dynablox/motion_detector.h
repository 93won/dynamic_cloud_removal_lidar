/**
 * @file motion_detector.h
 * @brief Main Dynablox motion detector (ROS-free, Voxblox-based)
 */

#ifndef DYNABLOX_MOTION_DETECTOR_H_
#define DYNABLOX_MOTION_DETECTOR_H_

#include <memory>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include "dynablox/types.h"
#include "dynablox/config.h"
#include "dynablox/ever_free_integrator.h"
#include "dynablox/clustering.h"

namespace dynablox {

/**
 * @brief Main motion detector combining TSDF integration + EverFree + Clustering
 */
class MotionDetector {
public:
    using Ptr = std::shared_ptr<MotionDetector>;
    
    explicit MotionDetector(const DynabloxConfig& config);
    
    /**
     * @brief Process a point cloud frame
     * @param cloud Point cloud in sensor frame
     * @param T_G_S Transformation from sensor to global frame
     * @param frame_id Frame number
     * @param timestamp Timestamp (optional)
     * @return Detection result
     */
    DetectionResult processPointcloud(
        const Pointcloud& cloud,
        const Transformation& T_G_S,
        uint32_t frame_id,
        double timestamp = 0.0);
    
    /**
     * @brief Get the TSDF layer (for visualization)
     */
    TsdfLayer* getTsdfLayer() { return tsdf_layer_; }
    
    /**
     * @brief Reset all state
     */
    void reset();
    
private:
    /**
     * @brief Build block-to-point map for clustering and update voxel state
     */
    BlockToPointMap buildBlockToPointMap(
        const Pointcloud& cloud,
        const Point& sensor_origin);
    
    /**
     * @brief Find occupied ever-free voxels
     */
    std::vector<voxblox::VoxelKey> findOccupiedEverFreeVoxels(
        const BlockToPointMap& point_map,
        int frame_counter,
        CloudInfo& cloud_info) const;
    
    /**
     * @brief Preprocess point cloud (range filter)
     */
    void preprocessPointcloud(
        const Pointcloud& cloud,
        const Point& sensor_origin,
        Pointcloud& filtered_cloud,
        std::vector<size_t>& original_indices) const;

private:
    DynabloxConfig config_;
    
    // Voxblox
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    TsdfLayer* tsdf_layer_;
    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
    
    // Dynablox components
    std::shared_ptr<EverFreeIntegrator> ever_free_integrator_;
    std::shared_ptr<Clustering> clustering_;
    
    // State
    int frame_counter_ = 0;
    
    // Cached values
    float voxel_size_;
    size_t voxels_per_side_;
    float max_range_;
};

}  // namespace dynablox

#endif  // DYNABLOX_MOTION_DETECTOR_H_
