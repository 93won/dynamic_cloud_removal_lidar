/**
 * @file point_cloud.h
 * @brief Point cloud types and utilities
 */

#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>

namespace util {

/**
 * @brief Basic 3D point structure
 */
struct Point3D {
    float x, y, z;
    
    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    // Conversion to/from Eigen
    Eigen::Vector3f toEigen() const { 
        return Eigen::Vector3f(x, y, z); 
    }
    
    static Point3D fromEigen(const Eigen::Vector3f& v) { 
        return Point3D(v.x(), v.y(), v.z()); 
    }
    
    // Norm calculations
    float norm() const { 
        return std::sqrt(x*x + y*y + z*z); 
    }
    
    float squaredNorm() const { 
        return x*x + y*y + z*z; 
    }
    
    // Operators
    Point3D operator+(const Point3D& o) const { 
        return Point3D(x + o.x, y + o.y, z + o.z); 
    }
    
    Point3D operator-(const Point3D& o) const { 
        return Point3D(x - o.x, y - o.y, z - o.z); 
    }
    
    Point3D operator*(float s) const { 
        return Point3D(x * s, y * s, z * s); 
    }
    
    Point3D operator/(float s) const { 
        return Point3D(x / s, y / s, z / s); 
    }
    
    Point3D& operator+=(const Point3D& o) {
        x += o.x; y += o.y; z += o.z;
        return *this;
    }
    
    Point3D& operator-=(const Point3D& o) {
        x -= o.x; y -= o.y; z -= o.z;
        return *this;
    }
    
    Point3D& operator*=(float s) {
        x *= s; y *= s; z *= s;
        return *this;
    }
    
    // Distance
    float distanceTo(const Point3D& o) const {
        return (*this - o).norm();
    }
    
    float squaredDistanceTo(const Point3D& o) const {
        return (*this - o).squaredNorm();
    }
};

/**
 * @brief Point cloud container
 */
class PointCloud {
public:
    using Ptr = std::shared_ptr<PointCloud>;
    using ConstPtr = std::shared_ptr<const PointCloud>;
    
    PointCloud() = default;
    explicit PointCloud(size_t reserve_size) {
        points_.reserve(reserve_size);
    }
    
    // Basic operations
    void push_back(const Point3D& p) { 
        points_.push_back(p); 
    }
    
    void push_back(float x, float y, float z) { 
        points_.emplace_back(x, y, z); 
    }
    
    size_t size() const { 
        return points_.size(); 
    }
    
    bool empty() const { 
        return points_.empty(); 
    }
    
    void clear() { 
        points_.clear(); 
    }
    
    void reserve(size_t n) { 
        points_.reserve(n); 
    }
    
    void resize(size_t n) {
        points_.resize(n);
    }
    
    // Access
    Point3D& operator[](size_t i) { 
        return points_[i]; 
    }
    
    const Point3D& operator[](size_t i) const { 
        return points_[i]; 
    }
    
    Point3D& at(size_t i) {
        return points_.at(i);
    }
    
    const Point3D& at(size_t i) const {
        return points_.at(i);
    }
    
    // Transformations
    void transform(const Eigen::Matrix4f& T);
    PointCloud::Ptr transformedCopy(const Eigen::Matrix4f& T) const;
    
    // Copy
    PointCloud::Ptr copy() const;
    
    // Merge
    void append(const PointCloud& other);
    PointCloud& operator+=(const PointCloud& other);
    
    // Iterators
    auto begin() { return points_.begin(); }
    auto end() { return points_.end(); }
    auto begin() const { return points_.begin(); }
    auto end() const { return points_.end(); }
    
    // Internal data access
    std::vector<Point3D>& points() { return points_; }
    const std::vector<Point3D>& points() const { return points_; }

private:
    std::vector<Point3D> points_;
};

}  // namespace util
