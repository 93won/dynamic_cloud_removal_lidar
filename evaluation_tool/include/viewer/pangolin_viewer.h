/**
 * @file pangolin_viewer.h
 * @brief Pangolin-based 3D visualization for point clouds and detections
 */

#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

#include <Eigen/Dense>
#include <pangolin/pangolin.h>

#include "util/point_cloud.h"
#include "dynablox/types.h"

namespace viewer {

/**
 * @brief Color palette for visualization
 */
struct Colors {
    static Eigen::Vector3f White() { return {1.0f, 1.0f, 1.0f}; }
    static Eigen::Vector3f Gray() { return {0.5f, 0.5f, 0.5f}; }
    static Eigen::Vector3f Red() { return {1.0f, 0.2f, 0.2f}; }
    static Eigen::Vector3f Green() { return {0.2f, 1.0f, 0.2f}; }
    static Eigen::Vector3f Blue() { return {0.2f, 0.4f, 1.0f}; }
    static Eigen::Vector3f Yellow() { return {1.0f, 1.0f, 0.2f}; }
    static Eigen::Vector3f Cyan() { return {0.2f, 1.0f, 1.0f}; }
    static Eigen::Vector3f Magenta() { return {1.0f, 0.2f, 1.0f}; }
    static Eigen::Vector3f Orange() { return {1.0f, 0.5f, 0.0f}; }
    
    // Get color by index (for clusters/tracks)
    static Eigen::Vector3f byIndex(int idx) {
        static std::vector<Eigen::Vector3f> palette = {
            Red(), Green(), Blue(), Yellow(), Cyan(), Magenta(), Orange(),
            {0.5f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.5f}, {0.0f, 0.5f, 1.0f}
        };
        return palette[idx % palette.size()];
    }
};

/**
 * @brief Pangolin-based 3D viewer
 */
class PangolinViewer {
public:
    PangolinViewer(int width = 1280, int height = 720);
    ~PangolinViewer();
    
    /**
     * @brief Start the viewer thread
     */
    void start();
    
    /**
     * @brief Stop the viewer and wait for thread to finish
     */
    void stop();
    
    /**
     * @brief Wait for viewer thread to finish
     */
    void join();
    
    /**
     * @brief Check if viewer is running
     */
    bool isRunning() const { return running_.load(); }
    
    /**
     * @brief Check if user requested quit
     */
    bool shouldQuit() const { return quit_requested_.load(); }
    
    /**
     * @brief Wait for key press (non-blocking after timeout)
     */
    void waitKey(int timeout_ms = 0);
    
    /**
     * @brief Update point cloud display
     * @param static_cloud Static points (gray)
     * @param dynamic_cloud Dynamic points (red)
     * @param sensor_pose Current sensor pose (optional)
     */
    void updateCloud(const util::PointCloud& static_cloud,
                     const util::PointCloud& dynamic_cloud,
                     const Eigen::Matrix4f& sensor_pose = Eigen::Matrix4f::Identity());
    
    /**
     * @brief Update cluster display
     */
    void updateClusters(const std::vector<dynablox::ClusterInfo>& clusters);
    
    /**
     * @brief Update tracked object display
     */
    void updateTracks(const std::vector<dynablox::TrackedObject>& tracks);
    
    /**
     * @brief Clear all displays
     */
    void clear();

private:
    /**
     * @brief Main rendering loop (runs in separate thread)
     */
    void run();
    
    /**
     * @brief Draw static point cloud
     */
    void drawStaticCloud();
    
    /**
     * @brief Draw dynamic point cloud
     */
    void drawDynamicCloud();
    
    /**
     * @brief Draw sensor trajectory
     */
    void drawTrajectory();
    
    /**
     * @brief Draw cluster bounding boxes
     */
    void drawClusters();
    
    /**
     * @brief Draw tracked object trajectories
     */
    void drawTracks();
    
    /**
     * @brief Draw coordinate axes at origin
     */
    void drawAxes(float length = 1.0f);
    
    /**
     * @brief Draw a 3D box
     */
    void drawBox(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt,
                 const Eigen::Vector3f& color, float line_width = 2.0f);

private:
    int width_, height_;
    std::thread viewer_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> quit_requested_{false};
    
    // Data with mutex protection
    std::mutex data_mutex_;
    std::vector<Eigen::Vector3f> static_points_;
    std::vector<Eigen::Vector3f> dynamic_points_;
    std::vector<Eigen::Matrix4f> trajectory_;
    std::vector<dynablox::ClusterInfo> clusters_;
    std::vector<dynablox::TrackedObject> tracks_;
    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
    
    // Display settings
    float point_size_static_ = 1.0f;
    float point_size_dynamic_ = 3.0f;
    bool show_static_ = true;
    bool show_dynamic_ = true;
    bool show_clusters_ = true;
    bool show_tracks_ = true;
    bool show_trajectory_ = true;
    bool follow_camera_ = false;
};

}  // namespace viewer
