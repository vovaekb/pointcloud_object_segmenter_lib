#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <fstream>

#include <pcl/io/pcd_io.h>


namespace point_cloud_utils {

    class ObjectSegmenter {
    public:
        using PointInT = pcl::PointXYZRGB;
        using PointInTPtr = pcl::PointCloud<PointInT>::Ptr;
        using IntVector = std::vector<int>;
        
        void run(const std::string& cloud_file_path, const std::string& indices_file_path, const std::string& result_path) const;
        void loadPointCloud(const std::string& cloud_file_path, PointInTPtr in) const;
        void readIndicesFromFile(const std::string& file_path, pcl::PointIndices::Ptr& indices_data) const;
        void segment(PointInTPtr in, PointInTPtr& out, pcl::PointIndices::Ptr& data) const;
        void saveResults(PointInTPtr& out, const std::string& result_path) const;
    };

};  // namespace point_cloud_utils