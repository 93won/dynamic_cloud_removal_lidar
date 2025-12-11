/**
 * @file gt_viewer.cpp
 * @brief Ground Truth Point Cloud Viewer using Pangolin
 * 
 * Visualizes point clouds with dynamic objects colored differently
 * based on ground truth labels.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <string>
#include <algorithm>
#include <filesystem>

#include <Eigen/Core>
#include <pangolin/pangolin.h>

namespace fs = std::filesystem;

// Helper function for ends_with (C++17 may not have it in older GCC)
bool endsWith(const std::string& str, const std::string& suffix) {
    if (suffix.size() > str.size()) return false;
    return str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

struct Point3D {
    float x, y, z;
};

// Load PLY file
std::vector<Point3D> loadPLY(const std::string& filepath) {
    std::vector<Point3D> points;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return points;
    }
    
    std::string line;
    bool in_header = true;
    int num_vertices = 0;
    
    while (std::getline(file, line)) {
        if (in_header) {
            if (line.find("element vertex") != std::string::npos) {
                sscanf(line.c_str(), "element vertex %d", &num_vertices);
            }
            if (line == "end_header") {
                in_header = false;
                points.reserve(num_vertices);
            }
            continue;
        }
        
        Point3D pt;
        std::istringstream iss(line);
        if (iss >> pt.x >> pt.y >> pt.z) {
            points.push_back(pt);
        }
    }
    
    return points;
}

// Load BIN file (KITTI format)
std::vector<Point3D> loadBIN(const std::string& filepath) {
    std::vector<Point3D> points;
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return points;
    }
    
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    size_t num_points = file_size / (4 * sizeof(float));
    points.reserve(num_points);
    
    for (size_t i = 0; i < num_points; ++i) {
        float data[4];
        file.read(reinterpret_cast<char*>(data), 4 * sizeof(float));
        points.push_back({data[0], data[1], data[2]});
    }
    
    return points;
}

// Load dynamic indices from label file
std::set<int> loadLabels(const std::string& filepath) {
    std::set<int> indices;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return indices;
    }
    
    int idx;
    while (file >> idx) {
        indices.insert(idx);
    }
    
    return indices;
}

// Load pose from poses.txt line
Eigen::Matrix4f loadPose(const std::string& line) {
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
        return poses;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            poses.push_back(loadPose(line));
        }
    }
    
    return poses;
}

class GTViewer {
public:
    GTViewer(const std::string& data_dir) : data_dir_(data_dir) {
        pointcloud_dir_ = data_dir + "/pointclouds";
        labels_dir_ = data_dir + "/labels";
        
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
        
        // Find first frame with labels
        for (size_t i = 0; i < pc_files_.size(); ++i) {
            std::string basename = fs::path(pc_files_[i]).stem().string();
            std::string label_file = labels_dir_ + "/" + basename + ".txt";
            if (fs::exists(label_file)) {
                first_labeled_frame_ = i;
                std::cout << "First labeled frame: " << i << " (" << basename << ")" << std::endl;
                break;
            }
        }
    }
    
    void loadFrame(int frame_idx) {
        if (frame_idx < 0 || frame_idx >= (int)total_frames_) return;
        
        current_frame_ = frame_idx;
        
        // Load point cloud
        std::vector<Point3D> points;
        if (format_ == "ply") {
            points = loadPLY(pc_files_[frame_idx]);
        } else {
            points = loadBIN(pc_files_[frame_idx]);
        }
        
        // Load labels
        std::string basename = fs::path(pc_files_[frame_idx]).stem().string();
        std::string label_file = labels_dir_ + "/" + basename + ".txt";
        std::set<int> dynamic_indices = loadLabels(label_file);
        
        std::cout << "Frame " << frame_idx << " (" << basename << "): " 
                  << dynamic_indices.size() << " dynamic indices loaded from " << label_file << std::endl;
        
        // Transform to world frame
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        if (frame_idx < (int)poses_.size()) {
            pose = poses_[frame_idx];
        }
        
        // Separate static and dynamic points
        static_points_.clear();
        dynamic_points_.clear();
        
        for (size_t i = 0; i < points.size(); ++i) {
            Eigen::Vector4f pt(points[i].x, points[i].y, points[i].z, 1.0f);
            Eigen::Vector4f pt_world = pose * pt;
            
            Point3D p_world = {pt_world.x(), pt_world.y(), pt_world.z()};
            
            if (dynamic_indices.count(i)) {
                dynamic_points_.push_back(p_world);
            } else {
                static_points_.push_back(p_world);
            }
        }
        
        num_dynamic_ = dynamic_points_.size();
        num_static_ = static_points_.size();
    }
    
    void run() {
        std::cout << "\n=== Ground Truth Point Cloud Viewer ===" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  N: Next frame" << std::endl;
        std::cout << "  P: Previous frame" << std::endl;
        std::cout << "  Space: Play/Pause" << std::endl;
        std::cout << "  Mouse: Rotate/zoom/pan" << std::endl;
        std::cout << "========================================\n" << std::endl;
        
        // Create Pangolin window
        pangolin::CreateWindowAndBind("GT Point Cloud Viewer", 1280, 720);
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
        
        pangolin::Var<int> ui_frame("ui.Frame", first_labeled_frame_, 0, total_frames_ - 1);
        pangolin::Var<bool> ui_playing("ui.Playing", false, true);
        pangolin::Var<bool> ui_show_static("ui.Show_Static", true, true);
        pangolin::Var<bool> ui_show_dynamic("ui.Show_Dynamic", true, true);
        pangolin::Var<bool> ui_show_trajectory("ui.Show_Trajectory", true, true);
        pangolin::Var<float> ui_point_size("ui.Point_Size", 2.0, 1.0, 10.0);
        pangolin::Var<int> ui_num_static("ui.Static_Points", 0);
        pangolin::Var<int> ui_num_dynamic("ui.Dynamic_Points", 0);
        
        // Load first frame
        loadFrame(0);
        
        int frame_counter = 0;
        
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            
            // Handle frame slider
            if (ui_frame.Get() != current_frame_) {
                loadFrame(ui_frame.Get());
            }
            
            // Auto-play
            if (ui_playing.Get()) {
                frame_counter++;
                if (frame_counter >= 3) {
                    frame_counter = 0;
                    int next_frame = (current_frame_ + 1) % total_frames_;
                    loadFrame(next_frame);
                    ui_frame = next_frame;
                }
            }
            
            // Update stats
            ui_num_static = num_static_;
            ui_num_dynamic = num_dynamic_;
            
            d_cam.Activate(s_cam);
            
            // Draw coordinate axes
            drawAxes(2.0f);
            
            // Draw trajectory
            if (ui_show_trajectory.Get()) {
                drawTrajectory();
            }
            
            // Draw static points (dark gray)
            if (ui_show_static.Get() && !static_points_.empty()) {
                glPointSize(ui_point_size.Get());
                glColor3f(0.3f, 0.3f, 0.3f);
                glBegin(GL_POINTS);
                for (const auto& pt : static_points_) {
                    glVertex3f(pt.x, pt.y, pt.z);
                }
                glEnd();
            }
            
            // Draw dynamic points (bright cyan/magenta for visibility)
            if (ui_show_dynamic.Get() && !dynamic_points_.empty()) {
                glPointSize(ui_point_size.Get() * 3.0f);
                glColor3f(1.0f, 0.2f, 0.8f);  // Magenta/Pink
                glBegin(GL_POINTS);
                for (const auto& pt : dynamic_points_) {
                    glVertex3f(pt.x, pt.y, pt.z);
                }
                glEnd();
            }
            
            pangolin::FinishFrame();
        }
    }
    
private:
    void drawAxes(float length) {
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        // X - Red
        glColor3f(1, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(length, 0, 0);
        // Y - Green
        glColor3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, length, 0);
        // Z - Blue
        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, length);
        glEnd();
    }
    
    void drawTrajectory() {
        if (poses_.size() < 2) return;
        
        glLineWidth(2.0f);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINE_STRIP);
        for (const auto& pose : poses_) {
            glVertex3f(pose(0, 3), pose(1, 3), pose(2, 3));
        }
        glEnd();
        
        // Current position
        if (current_frame_ < (int)poses_.size()) {
            const auto& pose = poses_[current_frame_];
            glPointSize(10.0f);
            glColor3f(1.0f, 1.0f, 0.0f);
            glBegin(GL_POINTS);
            glVertex3f(pose(0, 3), pose(1, 3), pose(2, 3));
            glEnd();
        }
    }
    
    std::string data_dir_;
    std::string pointcloud_dir_;
    std::string labels_dir_;
    std::string format_;
    
    std::vector<std::string> pc_files_;
    std::vector<Eigen::Matrix4f> poses_;
    
    std::vector<Point3D> static_points_;
    std::vector<Point3D> dynamic_points_;
    
    size_t total_frames_ = 0;
    int current_frame_ = 0;
    int first_labeled_frame_ = 0;
    int num_static_ = 0;
    int num_dynamic_ = 0;
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <data_directory>" << std::endl;
        std::cout << "\nData directory should contain:" << std::endl;
        std::cout << "  pointclouds/  - PLY or BIN files" << std::endl;
        std::cout << "  labels/       - Dynamic point indices" << std::endl;
        std::cout << "  poses.txt     - Camera poses (optional)" << std::endl;
        return 1;
    }
    
    std::string data_dir = argv[1];
    
    try {
        GTViewer viewer(data_dir);
        viewer.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
