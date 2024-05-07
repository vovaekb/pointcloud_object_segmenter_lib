#include "ObjectSegmenter.h"

namespace point_cloud_utils {

    void ObjectSegmenter::run(const std::string& cloud_file_path, const std::string& indices_file_path, const std::string& result_path) const {
        PointInTPtr scene_cloud;
        PointInTPtr object_cloud;
        pcl::PointIndices::Ptr point_indices;
        loadPointCloud(cloud_file_path, scene_cloud);

        readIndicesFromFile(indices_file_path, point_indices);

        segment(scene_cloud, object_cloud, point_indices);

    }

    void ObjectSegmenter::loadPointCloud(const std::string& cloud_file_path, PointInTPtr in) const {}
    
    void ObjectSegmenter::readIndicesFromFile(const std::string& file_path, pcl::PointIndices::Ptr& indices_data) const {}
    
    void ObjectSegmenter::segment(PointInTPtr in, PointInTPtr& out, pcl::PointIndices::Ptr& data) const {}
    
    void ObjectSegmenter::saveResults(PointInTPtr& out, const std::string& result_path) const {}

}