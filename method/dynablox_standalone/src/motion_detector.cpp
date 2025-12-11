/**
 * @file motion_detector.cpp
 * @brief Main Dynablox motion detector implementation
 */

#include "dynablox/motion_detector.h"
#include <iostream>
#include <algorithm>

namespace dynablox {

MotionDetector::MotionDetector(const DynabloxConfig& config)
    : config_(config) {
    
    voxel_size_ = config_.tsdf.voxel_size;
    voxels_per_side_ = config_.tsdf.voxels_per_side;
    max_range_ = config_.preprocessing.max_range;
    
    // Create TSDF map/layer
    voxblox::TsdfMap::Config map_config;
    map_config.tsdf_voxel_size = voxel_size_;
    map_config.tsdf_voxels_per_side = voxels_per_side_;
    
    tsdf_map_ = std::make_shared<voxblox::TsdfMap>(map_config);
    tsdf_layer_ = tsdf_map_->getTsdfLayerPtr();
    
    // Create TSDF integrator
    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.voxel_carving_enabled = true;
    integrator_config.max_ray_length_m = max_range_;
    integrator_config.min_ray_length_m = 0.5f;
    integrator_config.default_truncation_distance = voxel_size_ * 4.0f;
    integrator_config.allow_clear = true;
    
    tsdf_integrator_ = std::make_shared<voxblox::FastTsdfIntegrator>(
        integrator_config, tsdf_layer_);
    
    // Create Dynablox components
    ever_free_integrator_ = std::make_shared<EverFreeIntegrator>(
        config_.ever_free, tsdf_layer_);
    
    clustering_ = std::make_shared<Clustering>(
        config_.clustering, tsdf_layer_);
    
    frame_counter_ = 0;
    
    std::cout << "[MotionDetector] Initialized with voxel_size=" << voxel_size_
              << ", max_range=" << max_range_ << std::endl;
}

DetectionResult MotionDetector::processPointcloud(
    const Pointcloud& cloud,
    const Transformation& T_G_S,
    uint32_t frame_id,
    double timestamp) {
    
    DetectionResult result;
    result.frame_id = frame_id;
    result.timestamp = timestamp;
    
    if (cloud.empty()) {
        std::cerr << "[MotionDetector] Empty point cloud!" << std::endl;
        return result;
    }
    
    // Sensor origin in global frame
    Point sensor_origin = T_G_S.getPosition();
    
    // Preprocess: range filtering
    Pointcloud filtered_cloud;
    std::vector<size_t> original_indices;
    preprocessPointcloud(cloud, sensor_origin, filtered_cloud, original_indices);
    
    if (filtered_cloud.empty()) {
        return result;
    }
    
    // Transform to global frame
    Pointcloud global_cloud;
    global_cloud.reserve(filtered_cloud.size());
    for (const auto& pt : filtered_cloud) {
        Point global_pt = T_G_S.transform(pt);
        global_cloud.push_back(global_pt);
    }
    
    // Integrate into TSDF
    voxblox::Colors colors(global_cloud.size(), voxblox::Color::Gray());
    
    tsdf_integrator_->integratePointCloud(T_G_S, global_cloud, colors, false);
    
    // Build block-to-point map
    BlockToPointMap point_map = buildBlockToPointMap(global_cloud, sensor_origin);
    
    // Initialize cloud info
    CloudInfo cloud_info;
    cloud_info.points.resize(global_cloud.size());
    
    for (size_t i = 0; i < global_cloud.size(); ++i) {
        auto& info = cloud_info.points[i];
        info.point_index = i;
        info.original_index = original_indices[i];
        info.is_dynamic = false;
        info.cluster_id = -1;
        
        // Calculate distance from sensor
        info.distance = (global_cloud[i] - sensor_origin).norm();
    }
    
    // Find occupied ever-free voxels (seeds)
    std::vector<voxblox::VoxelKey> ever_free_seeds =
        findOccupiedEverFreeVoxels(point_map, frame_counter_, cloud_info);
    
    // Update ever-free voxels
    ever_free_integrator_->updateEverFreeVoxels(frame_counter_);
    
    // Perform clustering if past burn-in
    std::vector<Cluster> clusters;
    if (frame_counter_ >= config_.ever_free.burn_in_period) {
        clusters = clustering_->performClustering(
            global_cloud, point_map, ever_free_seeds, cloud_info);
        
        // Mark dynamic points
        for (const auto& cluster : clusters) {
            for (size_t pt_idx : cluster.point_indices) {
                if (pt_idx < cloud_info.points.size()) {
                    cloud_info.points[pt_idx].is_dynamic = true;
                    cloud_info.points[pt_idx].cluster_id = cluster.id;
                }
            }
        }
    }
    
    // Build result
    result.clusters = clusters;
    
    for (size_t i = 0; i < cloud_info.points.size(); ++i) {
        size_t orig_idx = cloud_info.points[i].original_index;
        if (cloud_info.points[i].is_dynamic) {
            result.dynamic_indices.push_back(orig_idx);
        } else {
            result.static_indices.push_back(orig_idx);
        }
    }
    
    // Count statistics
    result.num_total = cloud.size();
    result.num_filtered = filtered_cloud.size();
    result.num_dynamic = result.dynamic_indices.size();
    result.num_clusters = clusters.size();
    result.num_ever_free_seeds = ever_free_seeds.size();
    
    frame_counter_++;
    
    return result;
}

void MotionDetector::preprocessPointcloud(
    const Pointcloud& cloud,
    const Point& /* sensor_origin */,
    Pointcloud& filtered_cloud,
    std::vector<size_t>& original_indices) const {
    
    filtered_cloud.clear();
    original_indices.clear();
    filtered_cloud.reserve(cloud.size());
    original_indices.reserve(cloud.size());
    
    for (size_t i = 0; i < cloud.size(); ++i) {
        const Point& pt = cloud[i];
        
        // Skip NaN points
        if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z())) {
            continue;
        }
        
        // Range check (already in sensor frame)
        float range = pt.norm();
        if (range < 0.5f || range > max_range_) {
            continue;
        }
        
        filtered_cloud.push_back(pt);
        original_indices.push_back(i);
    }
}

BlockToPointMap MotionDetector::buildBlockToPointMap(
    const Pointcloud& cloud,
    const Point& /* sensor_origin */) {
    
    BlockToPointMap point_map;
    
    // First pass: collect points into voxels
    for (size_t i = 0; i < cloud.size(); ++i) {
        const Point& pt = cloud[i];
        
        // Get block index
        voxblox::BlockIndex block_idx = tsdf_layer_->computeBlockIndexFromCoordinates(pt);
        
        // Get voxel index
        voxblox::VoxelIndex voxel_idx;
        if (tsdf_layer_->hasBlock(block_idx)) {
            const auto& block = tsdf_layer_->getBlockByIndex(block_idx);
            voxel_idx = block.computeVoxelIndexFromCoordinates(pt);
        } else {
            // Compute voxel index manually
            Point pt_in_block = pt - voxblox::getOriginPointFromGridIndex(
                block_idx, tsdf_layer_->block_size());
            voxel_idx = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                pt_in_block, 1.0f / voxel_size_);
        }
        
        point_map[block_idx][voxel_idx].push_back(i);
    }
    
    // Second pass: update voxel state for occupied voxels
    for (const auto& block_pair : point_map) {
        const voxblox::BlockIndex& block_idx = block_pair.first;
        
        if (!tsdf_layer_->hasBlock(block_idx)) {
            continue;
        }
        
        auto block = tsdf_layer_->getBlockPtrByIndex(block_idx);
        if (!block) continue;
        
        for (const auto& voxel_pair : block_pair.second) {
            const voxblox::VoxelIndex& voxel_idx = voxel_pair.first;
            
            TsdfVoxel& voxel = block->getVoxelByVoxelIndex(voxel_idx);
            voxel.last_lidar_occupied = static_cast<uint32_t>(frame_counter_);
            voxel.last_occupied = static_cast<uint32_t>(frame_counter_);
            voxel.clustering_processed = false;
        }
        
        // Mark block as updated for EverFreeIntegrator
        block->updated().set(voxblox::Update::kEsdf);
    }
    
    return point_map;
}

std::vector<voxblox::VoxelKey> MotionDetector::findOccupiedEverFreeVoxels(
    const BlockToPointMap& point_map,
    int frame_counter,
    CloudInfo& cloud_info) const {
    
    std::vector<voxblox::VoxelKey> seeds;
    
    if (frame_counter < config_.ever_free.burn_in_period) {
        return seeds;  // No seeds during burn-in
    }
    
    for (const auto& block_pair : point_map) {
        const voxblox::BlockIndex& block_idx = block_pair.first;
        
        if (!tsdf_layer_->hasBlock(block_idx)) {
            continue;
        }
        
        const auto& block = tsdf_layer_->getBlockByIndex(block_idx);
        
        for (const auto& voxel_pair : block_pair.second) {
            const voxblox::VoxelIndex& voxel_idx = voxel_pair.first;
            
            const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(voxel_idx);
            
            // Check if this ever-free voxel is currently occupied
            // ever_free and last occupied check
            if (voxel.ever_free && 
                static_cast<int>(voxel.last_occupied) == frame_counter) {
                
                seeds.push_back({block_idx, voxel_idx});
                
                // Mark points in this voxel as potential dynamic
                for (int pt_idx : voxel_pair.second) {
                    if (static_cast<size_t>(pt_idx) < cloud_info.points.size()) {
                        cloud_info.points[pt_idx].is_dynamic = true;
                    }
                }
            }
        }
    }
    
    return seeds;
}

void MotionDetector::reset() {
    voxblox::TsdfMap::Config map_config;
    map_config.tsdf_voxel_size = voxel_size_;
    map_config.tsdf_voxels_per_side = voxels_per_side_;
    
    tsdf_map_ = std::make_shared<voxblox::TsdfMap>(map_config);
    tsdf_layer_ = tsdf_map_->getTsdfLayerPtr();
    
    voxblox::TsdfIntegratorBase::Config integrator_config;
    integrator_config.voxel_carving_enabled = true;
    integrator_config.max_ray_length_m = max_range_;
    integrator_config.min_ray_length_m = 0.5f;
    integrator_config.default_truncation_distance = voxel_size_ * 4.0f;
    integrator_config.allow_clear = true;
    
    tsdf_integrator_ = std::make_shared<voxblox::FastTsdfIntegrator>(
        integrator_config, tsdf_layer_);
    
    ever_free_integrator_ = std::make_shared<EverFreeIntegrator>(
        config_.ever_free, tsdf_layer_);
    
    clustering_ = std::make_shared<Clustering>(
        config_.clustering, tsdf_layer_);
    
    frame_counter_ = 0;
}

}  // namespace dynablox
