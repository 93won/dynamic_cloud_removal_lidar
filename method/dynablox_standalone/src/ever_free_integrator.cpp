/**
 * @file ever_free_integrator.cpp
 * @brief Ever-Free Integrator implementation (based on original Dynablox)
 */

#include "dynablox/ever_free_integrator.h"

#include <future>
#include <thread>
#include <vector>

namespace dynablox {

EverFreeIntegrator::EverFreeIntegrator(
    const EverFreeConfig& config,
    TsdfLayer* tsdf_layer)
    : config_(config),
      tsdf_layer_(tsdf_layer),
      neighborhood_search_(config.neighbor_connectivity),
      voxel_size_(tsdf_layer->voxel_size()),
      voxels_per_side_(tsdf_layer->voxels_per_side()),
      voxels_per_block_(voxels_per_side_ * voxels_per_side_ * voxels_per_side_) {
}

void EverFreeIntegrator::updateEverFreeVoxels(int frame_counter) const {
    // Get all updated blocks (we use kEsdf flag for ever-free tracking)
    voxblox::BlockIndexList updated_blocks;
    tsdf_layer_->getAllUpdatedBlocks(voxblox::Update::kEsdf, &updated_blocks);
    
    std::vector<BlockIndex> indices(updated_blocks.begin(), updated_blocks.end());
    
    // Phase 1: Update occupancy counter and collect voxels to remove
    voxblox::AlignedVector<voxblox::VoxelKey> voxels_to_remove;
    std::mutex result_mutex;
    
    // Simple thread pool using async
    std::vector<std::future<void>> futures;
    std::atomic<size_t> block_idx(0);
    
    auto worker = [&]() {
        voxblox::AlignedVector<voxblox::VoxelKey> local_to_remove;
        
        while (true) {
            size_t idx = block_idx.fetch_add(1);
            if (idx >= indices.size()) break;
            
            voxblox::AlignedVector<voxblox::VoxelKey> voxels;
            if (blockWiseUpdateEverFree(indices[idx], frame_counter, voxels)) {
                local_to_remove.insert(local_to_remove.end(), 
                                       voxels.begin(), voxels.end());
            }
        }
        
        if (!local_to_remove.empty()) {
            std::lock_guard<std::mutex> lock(result_mutex);
            voxels_to_remove.insert(voxels_to_remove.end(),
                                    local_to_remove.begin(), 
                                    local_to_remove.end());
        }
    };
    
    int num_threads = std::min(config_.num_threads, (int)indices.size());
    for (int i = 0; i < num_threads; ++i) {
        futures.push_back(std::async(std::launch::async, worker));
    }
    for (auto& f : futures) {
        f.get();
    }
    
    // Remove voxels that were flagged for removal
    for (const auto& voxel_key : voxels_to_remove) {
        auto tsdf_block = tsdf_layer_->getBlockPtrByIndex(voxel_key.first);
        if (!tsdf_block) continue;
        
        TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByVoxelIndex(voxel_key.second);
        tsdf_voxel.ever_free = false;
        tsdf_voxel.dynamic = false;
    }
    
    // Phase 2: Label free voxels as ever-free
    futures.clear();
    block_idx = 0;
    
    auto label_worker = [&]() {
        while (true) {
            size_t idx = block_idx.fetch_add(1);
            if (idx >= indices.size()) break;
            blockWiseMakeEverFree(indices[idx], frame_counter);
        }
    };
    
    for (int i = 0; i < num_threads; ++i) {
        futures.push_back(std::async(std::launch::async, label_worker));
    }
    for (auto& f : futures) {
        f.get();
    }
}

bool EverFreeIntegrator::blockWiseUpdateEverFree(
    const BlockIndex& block_index,
    int frame_counter,
    voxblox::AlignedVector<voxblox::VoxelKey>& voxels_to_remove) const {
    
    auto tsdf_block = tsdf_layer_->getBlockPtrByIndex(block_index);
    if (!tsdf_block) return false;
    
    for (size_t linear_idx = 0; linear_idx < voxels_per_block_; ++linear_idx) {
        TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(linear_idx);
        
        // Update occupancy counter if voxel is occupied
        if (tsdf_voxel.distance < config_.tsdf_occupancy_threshold ||
            tsdf_voxel.last_lidar_occupied == frame_counter) {
            updateOccupancyCounter(tsdf_voxel, frame_counter);
        }
        
        // Reset dynamic flag if not recently occupied by LiDAR
        if (tsdf_voxel.last_lidar_occupied < frame_counter - config_.temporal_buffer) {
            tsdf_voxel.dynamic = false;
        }
        
        // Check if ever-free should be removed
        if (tsdf_voxel.occ_counter >= config_.counter_to_reset && 
            tsdf_voxel.ever_free) {
            const VoxelIndex voxel_index = 
                tsdf_block->computeVoxelIndexFromLinearIndex(linear_idx);
            
            auto voxels = removeEverFree(*tsdf_block, tsdf_voxel, 
                                         block_index, voxel_index);
            voxels_to_remove.insert(voxels_to_remove.end(), 
                                    voxels.begin(), voxels.end());
        }
    }
    
    return !voxels_to_remove.empty();
}

void EverFreeIntegrator::updateOccupancyCounter(
    TsdfVoxel& voxel, int frame_counter) const {
    
    // Allow breaks of temporal_buffer between occupied observations
    if (voxel.last_occupied >= frame_counter - config_.temporal_buffer) {
        voxel.occ_counter++;
    } else {
        voxel.occ_counter = 1;
    }
    voxel.last_occupied = frame_counter;
}

voxblox::AlignedVector<voxblox::VoxelKey> EverFreeIntegrator::removeEverFree(
    TsdfBlock& block,
    TsdfVoxel& voxel,
    const BlockIndex& block_index,
    const VoxelIndex& voxel_index) const {
    
    // Remove ever-free from this voxel
    voxel.ever_free = false;
    voxel.dynamic = false;
    
    // Also remove from neighbors
    auto neighbors = neighborhood_search_.search(
        block_index, voxel_index, voxels_per_side_);
    
    voxblox::AlignedVector<voxblox::VoxelKey> voxels_to_remove;
    
    for (const auto& neighbor_key : neighbors) {
        if (neighbor_key.first == block_index) {
            // Same block - modify directly
            TsdfVoxel& neighbor_voxel = 
                block.getVoxelByVoxelIndex(neighbor_key.second);
            neighbor_voxel.ever_free = false;
            neighbor_voxel.dynamic = false;
        } else {
            // Different block - mark for later cleanup
            voxels_to_remove.push_back(neighbor_key);
        }
    }
    
    return voxels_to_remove;
}

void EverFreeIntegrator::blockWiseMakeEverFree(
    const BlockIndex& block_index,
    int frame_counter) const {
    
    auto tsdf_block = tsdf_layer_->getBlockPtrByIndex(block_index);
    if (!tsdf_block) return;
    
    for (size_t linear_idx = 0; linear_idx < voxels_per_block_; ++linear_idx) {
        TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(linear_idx);
        
        // Skip if already ever-free
        if (tsdf_voxel.ever_free) continue;
        
        // Must have been observed (have weight)
        if (tsdf_voxel.weight <= 1e-6) continue;
        
        // Must not have been occupied recently (burn-in period)
        if (tsdf_voxel.last_occupied > frame_counter - config_.burn_in_period) {
            continue;
        }
        
        // Check neighborhood for occupied or unobserved voxels
        const VoxelIndex voxel_index = 
            tsdf_block->computeVoxelIndexFromLinearIndex(linear_idx);
        
        auto neighbors = neighborhood_search_.search(
            block_index, voxel_index, voxels_per_side_);
        
        bool neighbor_occupied_or_unobserved = false;
        
        for (const auto& neighbor_key : neighbors) {
            const TsdfBlock* neighbor_block;
            
            if (neighbor_key.first == block_index) {
                neighbor_block = tsdf_block.get();
            } else {
                neighbor_block = tsdf_layer_->getBlockPtrByIndex(
                    neighbor_key.first).get();
                if (!neighbor_block) {
                    // Block doesn't exist - unobserved
                    neighbor_occupied_or_unobserved = true;
                    break;
                }
            }
            
            const TsdfVoxel& neighbor_voxel = 
                neighbor_block->getVoxelByVoxelIndex(neighbor_key.second);
            
            // Check if unobserved or recently occupied
            if (neighbor_voxel.weight < 1e-6 ||
                neighbor_voxel.last_occupied > frame_counter - config_.burn_in_period) {
                neighbor_occupied_or_unobserved = true;
                break;
            }
        }
        
        // Only observed free space can be labeled ever-free
        if (!neighbor_occupied_or_unobserved) {
            tsdf_voxel.ever_free = true;
        }
    }
    
    // Reset the update flag for this block
    tsdf_block->updated().reset(voxblox::Update::kEsdf);
}

}  // namespace dynablox
