/**
 * @file ObjectSegmenter.cpp
 * @brief Class for pipeline of segmenting object from a point cloud of scene using point indices
 */

#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>


namespace point_cloud_utils {

    /**
     * @brief Pipeline for segmenting object from a point cloud of scene using point indices
     */
    class ObjectSegmenter {
    public:
        using PointInT = pcl::PointXYZRGB;
        using PointInTPtr = pcl::PointCloud<PointInT>::Ptr;
        using PointIndicesT = pcl::PointIndices;
        using PointIndicesTPtr = pcl::PointIndices::Ptr;
        using IntVector = std::vector<int>;

        ObjectSegmenter();
        
        /**
         * @brief Start object segmentation pipeline
         * @param[in] cloud_file_path Path to input point cloud
         * @param[in] indices_file_path Path to file with indices data
         * @param[in] result_path Path to save results
         * */
        void run(const std::string& cloud_file_path, const std::string& indices_file_path, const std::string& result_path) const;
        
    private:
        /// Point cloud of scene
        PointInTPtr scene_cloud;
        /// Point cloud of object
        PointInTPtr object_cloud;

        /**
         * @brief Load point cloud from disk 
         * @param[in] cloud_file_path Path to file with point cloud
         * */
        void loadPointCloud(const std::string& cloud_file_path) const;
        /**
         * @brief Read raw point indices from file 
         * @param[in] file_path Path to file with indices data
         * @param[in] indices_data Vector with indices data
         * */
        void readIndicesFromFile(const std::string& file_path, IntVector& indices_data) const;
        /**
         * @brief Convert raw point indices to point cloud
         * @param[in] raw_indices Path to file with indices data
         * @param[in] point_indices_cloud Result point cloud of indices
         * */
        void convertPointIndices(const IntVector& raw_indices, PointIndicesTPtr& point_indices_cloud) const;
        /**
         * @brief Perform segmenting object from a point cloud 
         * @param[in] indices_data Point cloud of indices data
         * */
        void segment(const PointIndicesTPtr& indices_data) const;
        /**
         * @brief Save results
         * @param[in] result_path Path to save results
         * */
        void saveResults(const std::string& result_path) const;
    };

};  // namespace point_cloud_utils
