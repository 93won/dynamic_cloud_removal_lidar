/**
 * @file main.cpp
 * @brief Evaluation tool for dynamic object removal methods
 */

#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "util/point_cloud.h"
#include "util/io.h"
#include "dynablox/dynablox_detector.h"
#include "dynablox/config.h"

#ifdef USE_PANGOLIN
#include "viewer/pangolin_viewer.h"
#endif

// ==================== Configuration ====================

struct AppConfig {
    // Input paths
    std::string data_dir;
    std::string poses_file;
    std::string output_dir;
    std::string method_config;
    
    // Data format
    std::string point_cloud_format = "bin";  // "bin" or "ply"
    std::string poses_format = "kitti";       // "kitti" or "tum"
    
    // Processing
    int start_frame = 0;
    int end_frame = -1;  // -1 = all frames
    int frame_step = 1;
    
    // Output
    bool save_results = false;
    bool visualize = true;
    bool verbose = false;
    
    static AppConfig fromYaml(const std::string& filename) {
        AppConfig config;
        YAML::Node yaml = YAML::LoadFile(filename);
        
        if (yaml["data_dir"]) config.data_dir = yaml["data_dir"].as<std::string>();
        if (yaml["poses_file"]) config.poses_file = yaml["poses_file"].as<std::string>();
        if (yaml["output_dir"]) config.output_dir = yaml["output_dir"].as<std::string>();
        if (yaml["method_config"]) config.method_config = yaml["method_config"].as<std::string>();
        if (yaml["point_cloud_format"]) config.point_cloud_format = yaml["point_cloud_format"].as<std::string>();
        if (yaml["poses_format"]) config.poses_format = yaml["poses_format"].as<std::string>();
        if (yaml["start_frame"]) config.start_frame = yaml["start_frame"].as<int>();
        if (yaml["end_frame"]) config.end_frame = yaml["end_frame"].as<int>();
        if (yaml["frame_step"]) config.frame_step = yaml["frame_step"].as<int>();
        if (yaml["save_results"]) config.save_results = yaml["save_results"].as<bool>();
        if (yaml["visualize"]) config.visualize = yaml["visualize"].as<bool>();
        if (yaml["verbose"]) config.verbose = yaml["verbose"].as<bool>();
        
        return config;
    }
};

// ==================== Helper Functions ====================

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n"
              << "\nOptions:\n"
              << "  -c, --config <file>     Configuration file (YAML)\n"
              << "  -d, --data <dir>        Point cloud data directory\n"
              << "  -p, --poses <file>      Poses file\n"
              << "  -o, --output <dir>      Output directory\n"
              << "  -m, --method <file>     Method configuration file\n"
              << "  --start <n>             Start frame index\n"
              << "  --end <n>               End frame index\n"
              << "  --no-vis                Disable visualization\n"
              << "  --save                  Save filtered point clouds\n"
              << "  -v, --verbose           Verbose output\n"
              << "  -h, --help              Show this help\n";
}

AppConfig parseArgs(int argc, char** argv) {
    AppConfig config;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config = AppConfig::fromYaml(argv[++i]);
        } else if ((arg == "-d" || arg == "--data") && i + 1 < argc) {
            config.data_dir = argv[++i];
        } else if ((arg == "-p" || arg == "--poses") && i + 1 < argc) {
            config.poses_file = argv[++i];
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            config.output_dir = argv[++i];
        } else if ((arg == "-m" || arg == "--method") && i + 1 < argc) {
            config.method_config = argv[++i];
        } else if (arg == "--start" && i + 1 < argc) {
            config.start_frame = std::stoi(argv[++i]);
        } else if (arg == "--end" && i + 1 < argc) {
            config.end_frame = std::stoi(argv[++i]);
        } else if (arg == "--no-vis") {
            config.visualize = false;
        } else if (arg == "--save") {
            config.save_results = true;
        } else if (arg == "-v" || arg == "--verbose") {
            config.verbose = true;
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            exit(0);
        }
    }
    
    return config;
}

// ==================== Main ====================

int main(int argc, char** argv) {
    std::cout << "=== Dynamic Object Removal Evaluation Tool ===\n\n";
    
    // Parse arguments
    AppConfig app_config = parseArgs(argc, argv);
    
    // Validate input
    if (app_config.data_dir.empty()) {
        std::cerr << "Error: Data directory not specified.\n";
        printUsage(argv[0]);
        return 1;
    }
    
    // List point cloud files
    std::string extension = "." + app_config.point_cloud_format;
    std::vector<std::string> cloud_files = util::listFiles(app_config.data_dir, extension);
    
    if (cloud_files.empty()) {
        std::cerr << "Error: No point cloud files found in " << app_config.data_dir << "\n";
        return 1;
    }
    
    std::cout << "Found " << cloud_files.size() << " point cloud files.\n";
    
    // Load poses
    std::vector<Eigen::Matrix4f> poses;
    if (!app_config.poses_file.empty()) {
        if (app_config.poses_format == "kitti") {
            poses = util::loadPosesKITTI(app_config.poses_file);
        } else if (app_config.poses_format == "tum") {
            poses = util::loadPosesTUM(app_config.poses_file);
        }
        std::cout << "Loaded " << poses.size() << " poses.\n";
    }
    
    // Initialize detector
    dynablox::DynabloxConfig method_config;
    if (!app_config.method_config.empty() && util::fileExists(app_config.method_config)) {
        method_config = dynablox::DynabloxConfig::fromYaml(app_config.method_config);
    }
    method_config.verbose = app_config.verbose;
    
    dynablox::DynabloxDetector detector(method_config);
    
    // Initialize viewer
#ifdef USE_PANGOLIN
    std::unique_ptr<viewer::PangolinViewer> viewer;
    if (app_config.visualize) {
        viewer = std::make_unique<viewer::PangolinViewer>();
        viewer->start();
    }
#endif
    
    // Determine frame range
    int start_frame = app_config.start_frame;
    int end_frame = app_config.end_frame >= 0 ? app_config.end_frame : static_cast<int>(cloud_files.size());
    end_frame = std::min(end_frame, static_cast<int>(cloud_files.size()));
    
    // Statistics
    size_t total_points = 0;
    size_t total_dynamic = 0;
    double total_time = 0.0;
    
    std::cout << "\nProcessing frames " << start_frame << " to " << end_frame - 1 << "...\n\n";
    
    // Process frames
    for (int frame_idx = start_frame; frame_idx < end_frame; frame_idx += app_config.frame_step) {
        // Load point cloud
        util::PointCloud::Ptr cloud;
        if (app_config.point_cloud_format == "bin") {
            cloud = util::loadPointCloudBIN(cloud_files[frame_idx]);
        } else {
            cloud = util::loadPointCloudPLY(cloud_files[frame_idx]);
        }
        
        if (!cloud || cloud->empty()) {
            std::cerr << "Warning: Failed to load " << cloud_files[frame_idx] << "\n";
            continue;
        }
        
        // Get pose
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        if (frame_idx < static_cast<int>(poses.size())) {
            pose = poses[frame_idx];
        }
        
        // Process
        auto start_time = std::chrono::high_resolution_clock::now();
        
        dynablox::DetectionResult result = detector.processWithPose(
            *cloud, pose, static_cast<uint32_t>(frame_idx));
        
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        // Update statistics
        total_points += cloud->size();
        total_dynamic += result.numDynamicPoints();
        total_time += elapsed;
        
        // Print progress
        if (app_config.verbose || frame_idx % 10 == 0) {
            std::cout << "Frame " << std::setw(4) << frame_idx 
                      << ": " << std::setw(6) << result.numDynamicPoints() 
                      << "/" << std::setw(6) << cloud->size() << " dynamic"
                      << " (" << std::fixed << std::setprecision(1) << 100.0f * result.dynamicRatio() << "%)"
                      << ", " << std::setw(6) << std::setprecision(2) << elapsed << " ms"
                      << ", clusters: " << result.numClusters()
                      << ", tracks: " << result.tracked_objects.size()
                      << "\n";
        }
        
        // Visualize
#ifdef USE_PANGOLIN
        if (viewer && viewer->isRunning()) {
            // Get static and dynamic clouds for visualization
            auto static_cloud = detector.getStaticCloud(*cloud, result);
            auto dynamic_cloud = detector.getDynamicCloud(*cloud, result);
            
            viewer->updateCloud(*static_cloud, *dynamic_cloud, pose);
            viewer->updateClusters(result.clusters);
            viewer->updateTracks(result.tracked_objects);
            
            // Check for pause/quit
            if (viewer->shouldQuit()) {
                break;
            }
            
            // Wait for next frame (or user input)
            viewer->waitKey(10);
        }
#endif
        
        // Save results
        if (app_config.save_results && !app_config.output_dir.empty()) {
            auto static_cloud = detector.getStaticCloud(*cloud, result);
            
            std::string basename = util::getBasename(cloud_files[frame_idx]);
            std::string output_path = app_config.output_dir + "/" + basename + "_static.ply";
            
            util::savePointCloudPLY(output_path, *static_cloud);
        }
    }
    
    // Print summary
    std::cout << "\n=== Summary ===\n"
              << "Total frames processed: " << (end_frame - start_frame) / app_config.frame_step << "\n"
              << "Total points: " << total_points << "\n"
              << "Total dynamic points: " << total_dynamic 
              << " (" << std::fixed << std::setprecision(2) 
              << 100.0 * total_dynamic / total_points << "%)\n"
              << "Average processing time: " << std::setprecision(2) 
              << total_time / ((end_frame - start_frame) / app_config.frame_step) << " ms/frame\n";
    
#ifdef USE_PANGOLIN
    if (viewer) {
        std::cout << "\nPress ESC in viewer to exit...\n";
        viewer->join();
    }
#endif
    
    return 0;
}
