/**
 * @file point_cloud.cpp
 * @brief Point cloud implementation
 */

#include "util/point_cloud.h"

namespace util {

void PointCloud::transform(const Eigen::Matrix4f& T) {
    for (auto& point : points_) {
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f transformed = T * p;
        point.x = transformed.x();
        point.y = transformed.y();
        point.z = transformed.z();
    }
}

PointCloud::Ptr PointCloud::transformedCopy(const Eigen::Matrix4f& T) const {
    auto result = std::make_shared<PointCloud>();
    result->reserve(points_.size());
    
    for (const auto& point : points_) {
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f transformed = T * p;
        result->push_back(transformed.x(), transformed.y(), transformed.z());
    }
    
    return result;
}

PointCloud::Ptr PointCloud::copy() const {
    auto result = std::make_shared<PointCloud>();
    result->points_ = this->points_;
    return result;
}

void PointCloud::append(const PointCloud& other) {
    points_.reserve(points_.size() + other.points_.size());
    for (const auto& p : other.points_) {
        points_.push_back(p);
    }
}

PointCloud& PointCloud::operator+=(const PointCloud& other) {
    append(other);
    return *this;
}

}  // namespace util
