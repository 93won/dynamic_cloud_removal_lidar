/**
 * @file pangolin_viewer.cpp
 * @brief Pangolin viewer implementation
 */

#include "viewer/pangolin_viewer.h"

#include <chrono>

namespace viewer {

PangolinViewer::PangolinViewer(int width, int height)
    : width_(width), height_(height) {
}

PangolinViewer::~PangolinViewer() {
    stop();
}

void PangolinViewer::start() {
    if (running_.load()) return;
    
    running_.store(true);
    viewer_thread_ = std::thread(&PangolinViewer::run, this);
}

void PangolinViewer::stop() {
    running_.store(false);
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void PangolinViewer::join() {
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void PangolinViewer::waitKey(int timeout_ms) {
    if (timeout_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    }
}

void PangolinViewer::updateCloud(
    const util::PointCloud& static_cloud,
    const util::PointCloud& dynamic_cloud,
    const Eigen::Matrix4f& sensor_pose) {
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Update static points
    static_points_.clear();
    static_points_.reserve(static_cloud.size());
    for (size_t i = 0; i < static_cloud.size(); ++i) {
        const auto& pt = static_cloud[i];
        static_points_.emplace_back(pt.x, pt.y, pt.z);
    }
    
    // Update dynamic points
    dynamic_points_.clear();
    dynamic_points_.reserve(dynamic_cloud.size());
    for (size_t i = 0; i < dynamic_cloud.size(); ++i) {
        const auto& pt = dynamic_cloud[i];
        dynamic_points_.emplace_back(pt.x, pt.y, pt.z);
    }
    
    // Update pose and trajectory
    current_pose_ = sensor_pose;
    trajectory_.push_back(sensor_pose);
    
    // Limit trajectory length
    if (trajectory_.size() > 1000) {
        trajectory_.erase(trajectory_.begin());
    }
}

void PangolinViewer::updateClusters(const std::vector<dynablox::ClusterInfo>& clusters) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    clusters_ = clusters;
}

void PangolinViewer::updateTracks(const std::vector<dynablox::TrackedObject>& tracks) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    tracks_ = tracks;
}

void PangolinViewer::clear() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    static_points_.clear();
    dynamic_points_.clear();
    trajectory_.clear();
    clusters_.clear();
    tracks_.clear();
}

void PangolinViewer::run() {
    // Create window
    pangolin::CreateWindowAndBind("Dynamic Object Removal Viewer", width_, height_);
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Define camera projection and view
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(width_, height_, 500, 500, 
                                   width_/2, height_/2, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -50, 30, 0, 0, 0, pangolin::AxisZ)
    );
    
    // Create interactive view
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0, -float(width_)/float(height_))
        .SetHandler(&handler);
    
    // Create side panel for controls
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));
    
    // UI variables
    pangolin::Var<bool> show_static("ui.Show Static", true, true);
    pangolin::Var<bool> show_dynamic("ui.Show Dynamic", true, true);
    pangolin::Var<bool> show_clusters("ui.Show Clusters", true, true);
    pangolin::Var<bool> show_tracks("ui.Show Tracks", true, true);
    pangolin::Var<bool> show_trajectory("ui.Show Trajectory", true, true);
    pangolin::Var<bool> follow_camera("ui.Follow Camera", false, true);
    pangolin::Var<float> pt_size_static("ui.Static Pt Size", 1.0f, 0.5f, 5.0f);
    pangolin::Var<float> pt_size_dynamic("ui.Dynamic Pt Size", 3.0f, 1.0f, 10.0f);
    pangolin::Var<bool> reset_view("ui.Reset View", false, false);
    
    // Main loop
    while (!pangolin::ShouldQuit() && running_.load()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        
        // Update settings from UI
        show_static_ = show_static;
        show_dynamic_ = show_dynamic;
        show_clusters_ = show_clusters;
        show_tracks_ = show_tracks;
        show_trajectory_ = show_trajectory;
        follow_camera_ = follow_camera;
        point_size_static_ = pt_size_static;
        point_size_dynamic_ = pt_size_dynamic;
        
        // Reset view if requested
        if (pangolin::Pushed(reset_view)) {
            s_cam.SetModelViewMatrix(
                pangolin::ModelViewLookAt(0, -50, 30, 0, 0, 0, pangolin::AxisZ));
        }
        
        // Follow camera mode
        if (follow_camera_) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            Eigen::Vector3f pos = current_pose_.block<3,1>(0,3);
            Eigen::Vector3f forward = current_pose_.block<3,1>(0,0);
            
            s_cam.SetModelViewMatrix(
                pangolin::ModelViewLookAt(
                    pos.x() - 20*forward.x(), pos.y() - 20*forward.y(), pos.z() + 15,
                    pos.x() + 20*forward.x(), pos.y() + 20*forward.y(), pos.z(),
                    0, 0, 1));
        }
        
        d_cam.Activate(s_cam);
        
        // Draw content
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            drawAxes(3.0f);
            
            if (show_static_) {
                drawStaticCloud();
            }
            
            if (show_dynamic_) {
                drawDynamicCloud();
            }
            
            if (show_trajectory_) {
                drawTrajectory();
            }
            
            if (show_clusters_) {
                drawClusters();
            }
            
            if (show_tracks_) {
                drawTracks();
            }
        }
        
        pangolin::FinishFrame();
    }
    
    quit_requested_.store(true);
    pangolin::DestroyWindow("Dynamic Object Removal Viewer");
}

void PangolinViewer::drawStaticCloud() {
    if (static_points_.empty()) return;
    
    glPointSize(point_size_static_);
    glBegin(GL_POINTS);
    glColor3f(0.6f, 0.6f, 0.6f);  // Gray
    
    for (const auto& pt : static_points_) {
        glVertex3f(pt.x(), pt.y(), pt.z());
    }
    
    glEnd();
}

void PangolinViewer::drawDynamicCloud() {
    if (dynamic_points_.empty()) return;
    
    glPointSize(point_size_dynamic_);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.2f, 0.2f);  // Red
    
    for (const auto& pt : dynamic_points_) {
        glVertex3f(pt.x(), pt.y(), pt.z());
    }
    
    glEnd();
}

void PangolinViewer::drawTrajectory() {
    if (trajectory_.size() < 2) return;
    
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    glColor3f(0.2f, 1.0f, 0.2f);  // Green
    
    for (const auto& pose : trajectory_) {
        Eigen::Vector3f pos = pose.block<3,1>(0,3);
        glVertex3f(pos.x(), pos.y(), pos.z());
    }
    
    glEnd();
    
    // Draw current position as larger point
    if (!trajectory_.empty()) {
        Eigen::Vector3f pos = trajectory_.back().block<3,1>(0,3);
        glPointSize(10.0f);
        glBegin(GL_POINTS);
        glColor3f(0.2f, 1.0f, 0.2f);
        glVertex3f(pos.x(), pos.y(), pos.z());
        glEnd();
    }
}

void PangolinViewer::drawClusters() {
    for (const auto& cluster : clusters_) {
        Eigen::Vector3f color = Colors::byIndex(cluster.id);
        drawBox(cluster.min_bound, cluster.max_bound, color, 2.0f);
    }
}

void PangolinViewer::drawTracks() {
    for (const auto& track : tracks_) {
        Eigen::Vector3f color = Colors::byIndex(track.id);
        
        // Draw trajectory
        if (track.position_history.size() >= 2) {
            glLineWidth(2.0f);
            glBegin(GL_LINE_STRIP);
            glColor3f(color.x(), color.y(), color.z());
            
            for (const auto& pos : track.position_history) {
                glVertex3f(pos.x(), pos.y(), pos.z());
            }
            
            glEnd();
        }
        
        // Draw current position
        glPointSize(8.0f);
        glBegin(GL_POINTS);
        glColor3f(color.x(), color.y(), color.z());
        glVertex3f(track.position.x(), track.position.y(), track.position.z());
        glEnd();
        
        // Draw velocity vector
        if (track.velocity.norm() > 0.1f) {
            Eigen::Vector3f end = track.position + track.velocity;
            glLineWidth(3.0f);
            glBegin(GL_LINES);
            glColor3f(1.0f, 1.0f, 0.0f);  // Yellow
            glVertex3f(track.position.x(), track.position.y(), track.position.z());
            glVertex3f(end.x(), end.y(), end.z());
            glEnd();
        }
    }
}

void PangolinViewer::drawAxes(float length) {
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    
    // X axis - Red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(length, 0, 0);
    
    // Y axis - Green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, length, 0);
    
    // Z axis - Blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, length);
    
    glEnd();
}

void PangolinViewer::drawBox(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt,
                              const Eigen::Vector3f& color, float line_width) {
    glLineWidth(line_width);
    glColor3f(color.x(), color.y(), color.z());
    
    // 8 corners of the box
    float x0 = min_pt.x(), y0 = min_pt.y(), z0 = min_pt.z();
    float x1 = max_pt.x(), y1 = max_pt.y(), z1 = max_pt.z();
    
    glBegin(GL_LINES);
    
    // Bottom face
    glVertex3f(x0, y0, z0); glVertex3f(x1, y0, z0);
    glVertex3f(x1, y0, z0); glVertex3f(x1, y1, z0);
    glVertex3f(x1, y1, z0); glVertex3f(x0, y1, z0);
    glVertex3f(x0, y1, z0); glVertex3f(x0, y0, z0);
    
    // Top face
    glVertex3f(x0, y0, z1); glVertex3f(x1, y0, z1);
    glVertex3f(x1, y0, z1); glVertex3f(x1, y1, z1);
    glVertex3f(x1, y1, z1); glVertex3f(x0, y1, z1);
    glVertex3f(x0, y1, z1); glVertex3f(x0, y0, z1);
    
    // Vertical edges
    glVertex3f(x0, y0, z0); glVertex3f(x0, y0, z1);
    glVertex3f(x1, y0, z0); glVertex3f(x1, y0, z1);
    glVertex3f(x1, y1, z0); glVertex3f(x1, y1, z1);
    glVertex3f(x0, y1, z0); glVertex3f(x0, y1, z1);
    
    glEnd();
}

}  // namespace viewer
