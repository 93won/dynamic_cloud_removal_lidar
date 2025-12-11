/**
 * @file kdtree.h
 * @brief KD-Tree wrapper using nanoflann (header-only)
 */

#pragma once

#include <memory>
#include <vector>
#include <nanoflann.hpp>
#include "util/point_cloud.h"

namespace util {

/**
 * @brief Adapter for nanoflann to work with PointCloud
 */
struct PointCloudAdapter {
    const PointCloud& cloud;
    
    explicit PointCloudAdapter(const PointCloud& c) : cloud(c) {}
    
    inline size_t kdtree_get_point_count() const { 
        return cloud.size(); 
    }
    
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        const Point3D& p = cloud[idx];
        if (dim == 0) return p.x;
        if (dim == 1) return p.y;
        return p.z;
    }
    
    template<class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { 
        return false; 
    }
};

/**
 * @brief KD-Tree for fast nearest neighbor search
 */
class KdTree {
public:
    using TreeType = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdapter>,
        PointCloudAdapter,
        3  // dimensions
    >;
    
    KdTree() = default;
    ~KdTree() = default;
    
    /**
     * @brief Set input cloud and build the tree (shared_ptr version)
     */
    void setInputCloud(PointCloud::ConstPtr cloud) {
        if (!cloud || cloud->empty()) {
            tree_.reset();
            adapter_.reset();
            return;
        }
        
        cloud_ = cloud;
        adapter_ = std::make_unique<PointCloudAdapter>(*cloud);
        tree_ = std::make_unique<TreeType>(
            3, *adapter_, 
            nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */)
        );
        tree_->buildIndex();
    }
    
    /**
     * @brief Set input cloud and build the tree (const reference version)
     */
    void setInputCloud(const PointCloud& cloud) {
        if (cloud.empty()) {
            tree_.reset();
            adapter_.reset();
            return;
        }
        
        // Create a shared_ptr copy for internal storage
        cloud_ = std::make_shared<PointCloud>(cloud);
        adapter_ = std::make_unique<PointCloudAdapter>(*cloud_);
        tree_ = std::make_unique<TreeType>(
            3, *adapter_, 
            nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */)
        );
        tree_->buildIndex();
    }
    
    /**
     * @brief Find k nearest neighbors
     * @return Number of neighbors found
     */
    size_t knnSearch(const Point3D& query, size_t k,
                     std::vector<size_t>& indices,
                     std::vector<float>& distances) const {
        if (!tree_ || k == 0) {
            indices.clear();
            distances.clear();
            return 0;
        }
        
        indices.resize(k);
        distances.resize(k);
        
        std::vector<uint32_t> temp_indices(k);
        float query_pt[3] = {query.x, query.y, query.z};
        
        size_t num_found = tree_->knnSearch(
            query_pt, k, temp_indices.data(), distances.data()
        );
        
        indices.resize(num_found);
        distances.resize(num_found);
        
        for (size_t i = 0; i < num_found; ++i) {
            indices[i] = static_cast<size_t>(temp_indices[i]);
        }
        
        return num_found;
    }
    
    /**
     * @brief Find all neighbors within radius
     * @param radius Search radius
     * @return Number of neighbors found
     */
    size_t radiusSearch(const Point3D& query, float radius,
                        std::vector<size_t>& indices,
                        std::vector<float>& distances) const {
        if (!tree_ || radius <= 0) {
            indices.clear();
            distances.clear();
            return 0;
        }
        
        float query_pt[3] = {query.x, query.y, query.z};
        float squared_radius = radius * radius;
        
        std::vector<nanoflann::ResultItem<uint32_t, float>> matches;
        nanoflann::SearchParameters params;
        
        size_t num_found = tree_->radiusSearch(
            query_pt, squared_radius, matches, params
        );
        
        indices.resize(num_found);
        distances.resize(num_found);
        
        for (size_t i = 0; i < num_found; ++i) {
            indices[i] = static_cast<size_t>(matches[i].first);
            distances[i] = std::sqrt(matches[i].second);  // Convert squared to actual distance
        }
        
        return num_found;
    }
    
    /**
     * @brief Check if tree is built
     */
    bool isBuilt() const {
        return tree_ != nullptr;
    }

private:
    PointCloud::ConstPtr cloud_;
    std::unique_ptr<PointCloudAdapter> adapter_;
    std::unique_ptr<TreeType> tree_;
};

}  // namespace util
