/**
 * @file dynablox_runner.cpp
 * @brief Run Dynablox detector on extracted point cloud sequence
 * 
 * Usage: dynablox_runner <data_directory> [config.yaml]
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <unordered_set>
#include <unordered_map>

#include <Eigen/Dense>

// Pangolin for visualization
#include <pangolin/pangolin.h>

// Dynablox (refactored with Voxblox)
#include "dynablox/motion_detector.h"
#include "dynablox/config.h"
#include "dynablox/types.h"

// Util
#include "util/point_cloud.h"

namespace fs = std::filesystem;

// Helper function for C++17 compatibility
bool endsWith(const std::string& str, const std::string& suffix) {
    if (suffix.size() > str.size()) return false;
    return str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Load PLY point cloud
util::PointCloud::Ptr loadPLY(const std::string& filepath) {
    auto cloud = std::make_shared<util::PointCloud>();
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return cloud;
    }
    
    std::string line;
    int num_points = 0;
    bool in_header = true;
    
    // Parse header
    while (std::getline(file, line)) {
        if (line.find("element vertex") != std::string::npos) {
            std::istringstream iss(line);
            std::string tmp1, tmp2;
            iss >> tmp1 >> tmp2 >> num_points;
        }
        if (line == "end_header") {
            in_header = false;
            break;
        }
    }
    
    cloud->reserve(num_points);
    
    // Read points
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (iss >> x >> y >> z) {
            cloud->push_back(x, y, z);
        }
    }
    
    return cloud;
}

// Load BIN point cloud (KITTI format)
util::PointCloud::Ptr loadBIN(const std::string& filepath) {
    auto cloud = std::make_shared<util::PointCloud>();
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return cloud;
    }
    
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    size_t num_points = file_size / (4 * sizeof(float));
    cloud->reserve(num_points);
    
    for (size_t i = 0; i < num_points; ++i) {
        float data[4];
        file.read(reinterpret_cast<char*>(data), 4 * sizeof(float));
        cloud->push_back(data[0], data[1], data[2]);
    }
    
    return cloud;
}

// Load pose from poses.txt line (3x4 or 4x4 matrix)
Eigen::Matrix4f parsePose(const std::string& line) {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    std::istringstream iss(line);
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            iss >> pose(i, j);
        }
    }
    
    return pose;
}

// Load all poses
std::vector<Eigen::Matrix4f> loadPoses(const std::string& filepath) {
    std::vector<Eigen::Matrix4f> poses;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open poses file: " << filepath << std::endl;
        return poses;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            poses.push_back(parsePose(line));
        }
    }
    
    return poses;
}

// Simple point structure for rendering
struct RenderPoint {
    float x, y, z;
    float r, g, b;
};

class DynabloxRunner {
public:
    DynabloxRunner(const std::string& data_dir, const dynablox::DynabloxConfig& config)
        : data_dir_(data_dir), config_(config) {
        
        pointcloud_dir_ = data_dir + "/pointclouds";
        output_dir_ = data_dir + "/dynablox_output";
        
        // Create output directory
        fs::create_directories(output_dir_);
        
        // Find point cloud files
        for (const auto& entry : fs::directory_iterator(pointcloud_dir_)) {
            std::string path = entry.path().string();
            if (endsWith(path, ".ply") || endsWith(path, ".bin")) {
                pc_files_.push_back(path);
            }
        }
        std::sort(pc_files_.begin(), pc_files_.end());
        
        if (pc_files_.empty()) {
            throw std::runtime_error("No point cloud files found in " + pointcloud_dir_);
        }
        
        format_ = endsWith(pc_files_[0], ".ply") ? "ply" : "bin";
        total_frames_ = pc_files_.size();
        
        std::cout << "Found " << total_frames_ << " point cloud files" << std::endl;
        
        // Load poses
        std::string poses_file = data_dir + "/poses.txt";
        poses_ = loadPoses(poses_file);
        std::cout << "Loaded " << poses_.size() << " poses" << std::endl;
        
        // Initialize detector (new MotionDetector with Voxblox)
        detector_ = std::make_shared<dynablox::MotionDetector>(config_);
        std::cout << "Dynablox MotionDetector initialized (Voxblox-based)" << std::endl;
    }
    
    void runWithVisualization() {
        std::cout << "\n=== Dynablox Runner with Visualization ===" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  Space: Play/Pause" << std::endl;
        std::cout << "  N: Next frame (when paused)" << std::endl;
        std::cout << "  R: Reset to frame 0" << std::endl;
        std::cout << "  Mouse: Rotate/zoom/pan" << std::endl;
        std::cout << "==========================================\n" << std::endl;
        
        // Create Pangolin window
        pangolin::CreateWindowAndBind("Dynablox Runner", 1280, 720);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Camera
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1280, 720, 500, 500, 640, 360, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -50, 30, 0, 0, 0, 0, 0, 1)
        );
        
        // Interactive view
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1280.0f/720.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
        
        // UI Panel
        pangolin::CreatePanel("ui")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200));
        
        pangolin::Var<int> ui_frame("ui.Frame", 0, 0, total_frames_ - 1);
        pangolin::Var<bool> ui_playing("ui.Playing", false, true);
        pangolin::Var<bool> ui_show_static("ui.Show Static", true, true);
        pangolin::Var<bool> ui_show_dynamic("ui.Show Dynamic", true, true);
        pangolin::Var<bool> ui_show_trajectory("ui.Show Trajectory", true, true);
        pangolin::Var<float> ui_point_size("ui.Point Size", 2.0f, 1.0f, 10.0f);
        pangolin::Var<int> ui_num_static("ui.Static Points", 0);
        pangolin::Var<int> ui_num_dynamic("ui.Dynamic Points", 0);
        pangolin::Var<int> ui_num_clusters("ui.Clusters", 0);
        pangolin::Var<double> ui_proc_time("ui.Proc Time (ms)", 0.0);
        
        int last_frame = -1;
        
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            
            // Auto-play
            if (ui_playing.Get()) {
                int next_frame = (static_cast<int>(ui_frame.Get()) + 1) % total_frames_;
                ui_frame = next_frame;
            }
            
            // Process frame if changed
            int current_frame = ui_frame.Get();
            if (current_frame != last_frame) {
                processFrame(current_frame);
                last_frame = current_frame;
                
                ui_num_static = static_points_.size();
                ui_num_dynamic = dynamic_points_.size();
                ui_num_clusters = last_result_.clusters.size();
                ui_proc_time = last_proc_time_ms_;
            }
            
            d_cam.Activate(s_cam);
            
            // Draw trajectory
            if (ui_show_trajectory.Get() && poses_.size() > 1) {
                glLineWidth(2.0f);
                glColor3f(0.0f, 0.8f, 0.0f);
                glBegin(GL_LINE_STRIP);
                for (size_t i = 0; i <= std::min((size_t)current_frame, poses_.size() - 1); ++i) {
                    Eigen::Vector3f pos = poses_[i].block<3,1>(0,3);
                    glVertex3f(pos.x(), pos.y(), pos.z());
                }
                glEnd();
            }
            
            // Draw static points (gray)
            if (ui_show_static.Get() && !static_points_.empty()) {
                glPointSize(ui_point_size.Get());
                glColor3f(0.4f, 0.4f, 0.4f);
                glBegin(GL_POINTS);
                for (const auto& pt : static_points_) {
                    glVertex3f(pt.x, pt.y, pt.z);
                }
                glEnd();
            }
            
            // Draw dynamic points (magenta/cyan per cluster)
            if (ui_show_dynamic.Get() && !dynamic_points_.empty()) {
                glPointSize(ui_point_size.Get() * 2.5f);
                glBegin(GL_POINTS);
                for (const auto& pt : dynamic_points_) {
                    glColor3f(pt.r, pt.g, pt.b);
                    glVertex3f(pt.x, pt.y, pt.z);
                }
                glEnd();
            }
            
            // Draw cluster bounding boxes
            if (ui_show_dynamic.Get()) {
                glLineWidth(2.0f);
                for (const auto& cluster : last_result_.clusters) {
                    // Get cluster color
                    int cid = cluster.id % 6;
                    float r, g, b;
                    switch (cid) {
                        case 0: r = 1.0f; g = 0.2f; b = 0.8f; break;  // Magenta
                        case 1: r = 0.2f; g = 1.0f; b = 0.8f; break;  // Cyan
                        case 2: r = 1.0f; g = 0.8f; b = 0.2f; break;  // Yellow
                        case 3: r = 0.2f; g = 0.8f; b = 1.0f; break;  // Light blue
                        case 4: r = 1.0f; g = 0.5f; b = 0.2f; break;  // Orange
                        default: r = 0.8f; g = 0.2f; b = 1.0f; break; // Purple
                    }
                    glColor3f(r, g, b);
                    
                    // Draw AABB
                    Eigen::Vector3f min_pt = cluster.min_bound;
                    Eigen::Vector3f max_pt = cluster.max_bound;
                    
                    glBegin(GL_LINES);
                    // Bottom face
                    glVertex3f(min_pt.x(), min_pt.y(), min_pt.z()); glVertex3f(max_pt.x(), min_pt.y(), min_pt.z());
                    glVertex3f(max_pt.x(), min_pt.y(), min_pt.z()); glVertex3f(max_pt.x(), max_pt.y(), min_pt.z());
                    glVertex3f(max_pt.x(), max_pt.y(), min_pt.z()); glVertex3f(min_pt.x(), max_pt.y(), min_pt.z());
                    glVertex3f(min_pt.x(), max_pt.y(), min_pt.z()); glVertex3f(min_pt.x(), min_pt.y(), min_pt.z());
                    // Top face
                    glVertex3f(min_pt.x(), min_pt.y(), max_pt.z()); glVertex3f(max_pt.x(), min_pt.y(), max_pt.z());
                    glVertex3f(max_pt.x(), min_pt.y(), max_pt.z()); glVertex3f(max_pt.x(), max_pt.y(), max_pt.z());
                    glVertex3f(max_pt.x(), max_pt.y(), max_pt.z()); glVertex3f(min_pt.x(), max_pt.y(), max_pt.z());
                    glVertex3f(min_pt.x(), max_pt.y(), max_pt.z()); glVertex3f(min_pt.x(), min_pt.y(), max_pt.z());
                    // Verticals
                    glVertex3f(min_pt.x(), min_pt.y(), min_pt.z()); glVertex3f(min_pt.x(), min_pt.y(), max_pt.z());
                    glVertex3f(max_pt.x(), min_pt.y(), min_pt.z()); glVertex3f(max_pt.x(), min_pt.y(), max_pt.z());
                    glVertex3f(max_pt.x(), max_pt.y(), min_pt.z()); glVertex3f(max_pt.x(), max_pt.y(), max_pt.z());
                    glVertex3f(min_pt.x(), max_pt.y(), min_pt.z()); glVertex3f(min_pt.x(), max_pt.y(), max_pt.z());
                    glEnd();
                }
            }
            
            pangolin::FinishFrame();
        }
    }
    
    void runBatch() {
        std::cout << "\n=== Running Dynablox on all frames ===" << std::endl;
        
        double total_time = 0.0;
        int total_dynamic = 0;
        
        for (size_t i = 0; i < total_frames_; ++i) {
            processFrame(i);
            total_time += last_proc_time_ms_;
            total_dynamic += dynamic_points_.size();
            
            // Save results
            saveResults(i);
            
            if ((i + 1) % 10 == 0 || i == total_frames_ - 1) {
                std::cout << "Processed " << (i + 1) << "/" << total_frames_ 
                          << " frames, dynamic points: " << dynamic_points_.size()
                          << ", time: " << std::fixed << std::setprecision(1) << last_proc_time_ms_ << " ms" << std::endl;
            }
        }
        
        std::cout << "\n=== Summary ===" << std::endl;
        std::cout << "Total frames: " << total_frames_ << std::endl;
        std::cout << "Avg time per frame: " << std::fixed << std::setprecision(2) << (total_time / total_frames_) << " ms" << std::endl;
        std::cout << "Avg dynamic points: " << (total_dynamic / (int)total_frames_) << std::endl;
        std::cout << "Results saved to: " << output_dir_ << std::endl;
    }

private:
    void processFrame(size_t frame_idx) {
        if (frame_idx >= total_frames_) return;
        
        // Load point cloud
        util::PointCloud::Ptr cloud;
        if (format_ == "ply") {
            cloud = loadPLY(pc_files_[frame_idx]);
        } else {
            cloud = loadBIN(pc_files_[frame_idx]);
        }
        
        // Get pose
        Eigen::Matrix4f pose_f = Eigen::Matrix4f::Identity();
        if (frame_idx < poses_.size()) {
            pose_f = poses_[frame_idx];
        }
        dynablox::Transformation T_G_S(pose_f);
        
        // Convert to Pointcloud (vector<Eigen::Vector3f>)
        dynablox::Pointcloud pointcloud;
        pointcloud.reserve(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& pt = (*cloud)[i];
            pointcloud.emplace_back(pt.x, pt.y, pt.z);
        }
        
        // Run Dynablox
        auto start = std::chrono::high_resolution_clock::now();
        last_result_ = detector_->processPointcloud(pointcloud, T_G_S, frame_idx);
        auto end = std::chrono::high_resolution_clock::now();
        last_proc_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
        
        // Build dynamic index set for fast lookup
        std::unordered_set<size_t> dynamic_set(last_result_.dynamic_indices.begin(), 
                                                last_result_.dynamic_indices.end());
        
        // Build cluster ID map
        std::unordered_map<size_t, int> point_cluster_map;
        for (const auto& cluster : last_result_.clusters) {
            for (size_t idx : cluster.point_indices) {
                point_cluster_map[idx] = cluster.id;
            }
        }
        
        // Separate points for visualization (transform to world frame)
        static_points_.clear();
        dynamic_points_.clear();
        
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& pt = (*cloud)[i];
            Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f pw = pose_f * p;
            
            if (dynamic_set.count(i)) {
                // Dynamic point
                RenderPoint rp;
                rp.x = pw.x(); rp.y = pw.y(); rp.z = pw.z();
                
                // Color by cluster
                int cluster_id = point_cluster_map.count(i) ? point_cluster_map[i] : 0;
                int cid = cluster_id % 6;
                switch (cid) {
                    case 0: rp.r = 1.0f; rp.g = 0.2f; rp.b = 0.8f; break;
                    case 1: rp.r = 0.2f; rp.g = 1.0f; rp.b = 0.8f; break;
                    case 2: rp.r = 1.0f; rp.g = 0.8f; rp.b = 0.2f; break;
                    case 3: rp.r = 0.2f; rp.g = 0.8f; rp.b = 1.0f; break;
                    case 4: rp.r = 1.0f; rp.g = 0.5f; rp.b = 0.2f; break;
                    default: rp.r = 0.8f; rp.g = 0.2f; rp.b = 1.0f; break;
                }
                dynamic_points_.push_back(rp);
            } else {
                // Static point
                RenderPoint rp;
                rp.x = pw.x(); rp.y = pw.y(); rp.z = pw.z();
                rp.r = 0.4f; rp.g = 0.4f; rp.b = 0.4f;
                static_points_.push_back(rp);
            }
        }
    }
    
    void saveResults(size_t frame_idx) {
        // Save dynamic point indices
        std::ostringstream filename;
        filename << output_dir_ << "/" << std::setfill('0') << std::setw(6) << frame_idx << ".txt";
        
        std::ofstream file(filename.str());
        for (size_t idx : last_result_.dynamic_indices) {
            file << idx << "\n";
        }
    }
    
    std::string data_dir_;
    std::string pointcloud_dir_;
    std::string output_dir_;
    dynablox::DynabloxConfig config_;
    
    std::vector<std::string> pc_files_;
    std::vector<Eigen::Matrix4f> poses_;
    size_t total_frames_ = 0;
    std::string format_;
    
    dynablox::MotionDetector::Ptr detector_;
    dynablox::DetectionResult last_result_;
    double last_proc_time_ms_ = 0.0;
    
    std::vector<RenderPoint> static_points_;
    std::vector<RenderPoint> dynamic_points_;
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <data_directory> [config.yaml] [--batch]" << std::endl;
        std::cout << "\nData directory should contain:" << std::endl;
        std::cout << "  pointclouds/  - PLY or BIN files" << std::endl;
        std::cout << "  poses.txt     - Camera poses" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  config.yaml   - Dynablox configuration file (optional)" << std::endl;
        std::cout << "  --batch       - Run without visualization, save results" << std::endl;
        return 1;
    }
    
    std::string data_dir = argv[1];
    
    // Load config
    dynablox::DynabloxConfig config;
    bool batch_mode = false;
    
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--batch") {
            batch_mode = true;
        } else if (endsWith(arg, ".yaml") || endsWith(arg, ".yml")) {
            std::cout << "Loading config from: " << arg << std::endl;
            config = dynablox::DynabloxConfig::fromYaml(arg);
        }
    }
    
    // Set verbose mode for debugging
    config.verbose = true;
    
    try {
        DynabloxRunner runner(data_dir, config);
        
        if (batch_mode) {
            runner.runBatch();
        } else {
            runner.runWithVisualization();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
