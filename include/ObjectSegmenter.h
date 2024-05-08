#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>


namespace point_cloud_utils {

    class ObjectSegmenter {
    public:
        using PointInT = pcl::PointXYZRGB;
        using PointInTPtr = pcl::PointCloud<PointInT>::Ptr;
        using PointIndicesT = pcl::PointIndices;
        using PointIndicesTPtr = pcl::PointIndices::Ptr;
        using IntVector = std::vector<int>;

        ObjectSegmenter();
        
        void run(const std::string& cloud_file_path, const std::string& indices_file_path, const std::string& result_path) const;
        
    private:
        PointInTPtr scene_cloud;
        PointInTPtr object_cloud;

        void loadPointCloud(const std::string& cloud_file_path) const;
        void readIndicesFromFile(const std::string& file_path, IntVector& indices_data) const;
        void convertPointIndices(const IntVector& raw_indices, PointIndicesTPtr& point_indices_cloud) const;
        void segment(const PointIndicesTPtr& indices_data) const;
        void saveResults(const std::string& result_path) const;
    };

};  // namespace point_cloud_utils
