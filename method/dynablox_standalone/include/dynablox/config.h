/**
 * @file config.h
 * @brief Configuration for Dynablox (based on original ROS params)
 */

#ifndef DYNABLOX_CONFIG_H_
#define DYNABLOX_CONFIG_H_

#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace dynablox {

/**
 * @brief Preprocessing configuration
 */
struct PreprocessingConfig {
    float min_range = 0.5f;   // Minimum point range [m]
    float max_range = 20.0f;  // Maximum point range [m]
};

/**
 * @brief Ever-free integrator configuration (from original)
 */
struct EverFreeConfig {
    // Neighborhood connectivity for removing ever-free (6, 18, or 26)
    int neighbor_connectivity = 26;
    
    // After this many occupied observations, ever-free becomes never-free
    int counter_to_reset = 150;
    
    // Frames a voxel can be free between occupancy (compensate sparsity)
    int temporal_buffer = 2;
    
    // Consecutive frames a voxel must be free to become ever-free
    int burn_in_period = 5;
    
    // TSDF distance below which voxel is considered occupied [m]
    float tsdf_occupancy_threshold = 0.3f;
    
    // Number of threads
    int num_threads = 4;
};

/**
 * @brief TSDF integrator configuration
 */
struct TsdfConfig {
    float voxel_size = 0.2f;
    int voxels_per_side = 16;
    float truncation_distance = 0.4f;  // 2 * voxel_size
    float max_weight = 1000.0f;
    
    // Projective integrator settings
    int sensor_horizontal_resolution = 2048;
    int sensor_vertical_resolution = 64;
    float sensor_vertical_fov_degrees = 33.2f;
    
    float min_ray_length = 0.5f;
    float max_ray_length = 20.0f;
};

/**
 * @brief Clustering configuration (from original)
 */
struct ClusteringConfig {
    int min_cluster_size = 20;
    int max_cluster_size = 200000;
    float min_extent = 0.0f;
    float max_extent = 200000.0f;
    int neighbor_connectivity = 6;
    bool grow_clusters_twice = false;
    float min_cluster_separation = 0.2f;
};

/**
 * @brief Tracking configuration
 */
struct TrackingConfig {
    int min_track_duration = 0;
    float max_tracking_distance = 1.0f;
};

/**
 * @brief Main Dynablox configuration
 */
struct DynabloxConfig {
    PreprocessingConfig preprocessing;
    EverFreeConfig ever_free;
    TsdfConfig tsdf;
    ClusteringConfig clustering;
    TrackingConfig tracking;
    
    bool verbose = false;
    int num_threads = 4;
    
    /**
     * @brief Load from YAML file
     */
    static DynabloxConfig fromYaml(const std::string& filepath) {
        DynabloxConfig config;
        
        try {
            YAML::Node yaml = YAML::LoadFile(filepath);
            
            // Preprocessing
            if (yaml["preprocessing"]) {
                YAML::Node pp = yaml["preprocessing"];
                if (pp["min_range"]) config.preprocessing.min_range = pp["min_range"].as<float>();
                if (pp["max_range"]) config.preprocessing.max_range = pp["max_range"].as<float>();
            }
            
            // Ever-free
            if (yaml["ever_free_integrator"] || yaml["ever_free"]) {
                YAML::Node ef = yaml["ever_free_integrator"] ? yaml["ever_free_integrator"] : yaml["ever_free"];
                if (ef["counter_to_reset"]) config.ever_free.counter_to_reset = ef["counter_to_reset"].as<int>();
                if (ef["temporal_buffer"]) config.ever_free.temporal_buffer = ef["temporal_buffer"].as<int>();
                if (ef["burn_in_period"]) config.ever_free.burn_in_period = ef["burn_in_period"].as<int>();
                if (ef["tsdf_occupancy_threshold"]) config.ever_free.tsdf_occupancy_threshold = ef["tsdf_occupancy_threshold"].as<float>();
                if (ef["neighbor_connectivity"]) config.ever_free.neighbor_connectivity = ef["neighbor_connectivity"].as<int>();
            }
            
            // TSDF / Voxblox
            if (yaml["voxblox"] || yaml["tsdf"]) {
                YAML::Node vb = yaml["voxblox"] ? yaml["voxblox"] : yaml["tsdf"];
                if (vb["tsdf_voxel_size"] || vb["voxel_size"]) {
                    config.tsdf.voxel_size = (vb["tsdf_voxel_size"] ? vb["tsdf_voxel_size"] : vb["voxel_size"]).as<float>();
                }
                if (vb["truncation_distance"]) config.tsdf.truncation_distance = vb["truncation_distance"].as<float>();
                if (vb["max_ray_length_m"]) config.tsdf.max_ray_length = vb["max_ray_length_m"].as<float>();
                if (vb["min_ray_length_m"]) config.tsdf.min_ray_length = vb["min_ray_length_m"].as<float>();
            }
            
            // Clustering
            if (yaml["clustering"]) {
                YAML::Node cl = yaml["clustering"];
                if (cl["min_cluster_size"]) config.clustering.min_cluster_size = cl["min_cluster_size"].as<int>();
                if (cl["max_cluster_size"]) config.clustering.max_cluster_size = cl["max_cluster_size"].as<int>();
                if (cl["min_extent"]) config.clustering.min_extent = cl["min_extent"].as<float>();
                if (cl["max_extent"]) config.clustering.max_extent = cl["max_extent"].as<float>();
                if (cl["neighbor_connectivity"]) config.clustering.neighbor_connectivity = cl["neighbor_connectivity"].as<int>();
            }
            
            // Tracking
            if (yaml["tracking"]) {
                YAML::Node tr = yaml["tracking"];
                if (tr["min_track_duration"]) config.tracking.min_track_duration = tr["min_track_duration"].as<int>();
                if (tr["max_tracking_distance"]) config.tracking.max_tracking_distance = tr["max_tracking_distance"].as<float>();
            }
            
            // General
            if (yaml["verbose"]) config.verbose = yaml["verbose"].as<bool>();
            if (yaml["num_threads"]) config.num_threads = yaml["num_threads"].as<int>();
            
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to load config from " << filepath << ": " << e.what() << std::endl;
        }
        
        return config;
    }
    
    /**
     * @brief Get default config matching original Dynablox
     */
    static DynabloxConfig getDefault() {
        DynabloxConfig config;
        // Already initialized with original defaults
        return config;
    }
};

}  // namespace dynablox

#endif  // DYNABLOX_CONFIG_H_
