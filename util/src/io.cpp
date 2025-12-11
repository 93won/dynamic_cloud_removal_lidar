/**
 * @file io.cpp
 * @brief File I/O utilities implementation
 */

#include "util/io.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <cstring>

namespace fs = std::filesystem;

namespace util {

// ==================== Point Cloud I/O ====================

PointCloud::Ptr loadPointCloudPLY(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return nullptr;
    }

    // Parse PLY header
    std::string line;
    size_t vertex_count = 0;
    bool is_binary = false;
    bool has_header_end = false;
    int x_idx = -1, y_idx = -1, z_idx = -1;
    int property_count = 0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "format") {
            std::string format;
            iss >> format;
            is_binary = (format == "binary_little_endian" || format == "binary_big_endian");
        } else if (token == "element") {
            std::string elem_type;
            iss >> elem_type;
            if (elem_type == "vertex") {
                iss >> vertex_count;
            }
        } else if (token == "property") {
            std::string type, name;
            iss >> type >> name;
            if (name == "x") x_idx = property_count;
            else if (name == "y") y_idx = property_count;
            else if (name == "z") z_idx = property_count;
            property_count++;
        } else if (token == "end_header") {
            has_header_end = true;
            break;
        }
    }

    if (!has_header_end || vertex_count == 0) {
        return nullptr;
    }

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(vertex_count);

    if (is_binary) {
        // Binary format - assume float x, y, z as first 3 properties
        for (size_t i = 0; i < vertex_count; ++i) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), sizeof(float));
            file.read(reinterpret_cast<char*>(&y), sizeof(float));
            file.read(reinterpret_cast<char*>(&z), sizeof(float));
            
            // Skip remaining properties
            for (int j = 3; j < property_count; ++j) {
                float dummy;
                file.read(reinterpret_cast<char*>(&dummy), sizeof(float));
            }
            
            cloud->push_back(x, y, z);
        }
    } else {
        // ASCII format
        for (size_t i = 0; i < vertex_count; ++i) {
            if (!std::getline(file, line)) break;
            std::istringstream iss(line);
            
            std::vector<float> values;
            float val;
            while (iss >> val) {
                values.push_back(val);
            }
            
            if (x_idx >= 0 && y_idx >= 0 && z_idx >= 0 &&
                x_idx < static_cast<int>(values.size()) &&
                y_idx < static_cast<int>(values.size()) &&
                z_idx < static_cast<int>(values.size())) {
                cloud->push_back(values[x_idx], values[y_idx], values[z_idx]);
            }
        }
    }

    return cloud;
}

PointCloud::Ptr loadPointCloudBIN(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return nullptr;
    }

    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // KITTI format: each point is 4 floats (x, y, z, intensity)
    size_t point_size = 4 * sizeof(float);
    size_t num_points = file_size / point_size;

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(num_points);

    std::vector<float> buffer(4);
    for (size_t i = 0; i < num_points; ++i) {
        file.read(reinterpret_cast<char*>(buffer.data()), point_size);
        cloud->push_back(buffer[0], buffer[1], buffer[2]);
        // buffer[3] is intensity, ignored for now
    }

    return cloud;
}

bool savePointCloudPLY(const std::string& filename, const PointCloud& cloud) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    // Write header
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    // Write binary data
    for (const auto& pt : cloud) {
        file.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));
    }

    return true;
}

bool savePointCloudPLYWithColors(const std::string& filename,
                                  const PointCloud& cloud,
                                  const std::vector<Eigen::Vector3i>& colors) {
    if (cloud.size() != colors.size()) {
        return false;
    }

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    // Write header
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";

    // Write binary data
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& pt = cloud[i];
        file.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));
        
        uint8_t r = static_cast<uint8_t>(std::clamp(colors[i].x(), 0, 255));
        uint8_t g = static_cast<uint8_t>(std::clamp(colors[i].y(), 0, 255));
        uint8_t b = static_cast<uint8_t>(std::clamp(colors[i].z(), 0, 255));
        file.write(reinterpret_cast<const char*>(&r), sizeof(uint8_t));
        file.write(reinterpret_cast<const char*>(&g), sizeof(uint8_t));
        file.write(reinterpret_cast<const char*>(&b), sizeof(uint8_t));
    }

    return true;
}

// ==================== Pose I/O ====================

std::vector<Eigen::Matrix4f> loadPosesKITTI(const std::string& filename) {
    std::vector<Eigen::Matrix4f> poses;
    std::ifstream file(filename);
    if (!file.is_open()) {
        return poses;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        
        // Read 12 values (3x4 matrix row-major)
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                iss >> pose(r, c);
            }
        }
        
        poses.push_back(pose);
    }

    return poses;
}

std::vector<Eigen::Matrix4f> loadPosesTUM(const std::string& filename) {
    std::vector<Eigen::Matrix4f> poses;
    std::ifstream file(filename);
    if (!file.is_open()) {
        return poses;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        Eigen::Quaternionf q(static_cast<float>(qw), 
                             static_cast<float>(qx), 
                             static_cast<float>(qy), 
                             static_cast<float>(qz));
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.toRotationMatrix();
        pose(0,3) = static_cast<float>(tx);
        pose(1,3) = static_cast<float>(ty);
        pose(2,3) = static_cast<float>(tz);
        
        poses.push_back(pose);
    }

    return poses;
}

bool savePosesKITTI(const std::string& filename,
                    const std::vector<Eigen::Matrix4f>& poses) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    file << std::fixed << std::setprecision(9);
    for (const auto& pose : poses) {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (r > 0 || c > 0) file << " ";
                file << pose(r, c);
            }
        }
        file << "\n";
    }

    return true;
}

// ==================== File Utilities ====================

std::vector<std::string> listFiles(const std::string& directory, 
                                    const std::string& extension) {
    std::vector<std::string> files;
    
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
        return files;
    }

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string path = entry.path().string();
            if (extension.empty() || entry.path().extension() == extension) {
                files.push_back(path);
            }
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

bool fileExists(const std::string& filename) {
    return fs::exists(filename);
}

std::string getBasename(const std::string& filepath) {
    return fs::path(filepath).stem().string();
}

}  // namespace util
