/**
 * @file io.h
 * @brief File I/O utilities for point clouds and poses
 */

#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "util/point_cloud.h"

namespace util {

// ==================== Point Cloud I/O ====================

/**
 * @brief Load point cloud from PLY file
 * @param filename Path to PLY file
 * @return Point cloud pointer, nullptr on failure
 */
PointCloud::Ptr loadPointCloudPLY(const std::string& filename);

/**
 * @brief Load point cloud from KITTI binary file (.bin)
 * @param filename Path to .bin file
 * @return Point cloud pointer, nullptr on failure
 */
PointCloud::Ptr loadPointCloudBIN(const std::string& filename);

/**
 * @brief Save point cloud to PLY file (binary format)
 * @param filename Output path
 * @param cloud Point cloud to save
 * @return true on success
 */
bool savePointCloudPLY(const std::string& filename, const PointCloud& cloud);

/**
 * @brief Save point cloud to PLY file with colors
 * @param filename Output path
 * @param cloud Point cloud to save
 * @param colors RGB colors for each point (0-255)
 * @return true on success
 */
bool savePointCloudPLYWithColors(const std::string& filename, 
                                  const PointCloud& cloud,
                                  const std::vector<Eigen::Vector3i>& colors);

// ==================== Pose I/O ====================

/**
 * @brief Load poses from KITTI format file
 * Each line: 12 values (3x4 matrix row-major)
 * @param filename Path to poses file
 * @return Vector of 4x4 transformation matrices
 */
std::vector<Eigen::Matrix4f> loadPosesKITTI(const std::string& filename);

/**
 * @brief Load poses from TUM format file
 * Each line: timestamp tx ty tz qx qy qz qw
 * @param filename Path to poses file
 * @return Vector of 4x4 transformation matrices
 */
std::vector<Eigen::Matrix4f> loadPosesTUM(const std::string& filename);

/**
 * @brief Save poses to KITTI format file
 * @param filename Output path
 * @param poses Vector of 4x4 transformation matrices
 * @return true on success
 */
bool savePosesKITTI(const std::string& filename, 
                    const std::vector<Eigen::Matrix4f>& poses);

// ==================== File Utilities ====================

/**
 * @brief List all files in directory with given extension
 * @param directory Directory path
 * @param extension File extension (e.g., ".bin", ".ply")
 * @return Sorted list of file paths
 */
std::vector<std::string> listFiles(const std::string& directory, 
                                    const std::string& extension);

/**
 * @brief Check if file exists
 */
bool fileExists(const std::string& filename);

/**
 * @brief Get filename without path and extension
 */
std::string getBasename(const std::string& filepath);

}  // namespace util
