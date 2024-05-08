#include <fstream>

#include "ObjectSegmenter.h"

namespace point_cloud_utils {

    ObjectSegmenter::ObjectSegmenter() : scene_cloud(new pcl::PointCloud<PointInT>()), object_cloud(new pcl::PointCloud<PointInT>()) {}

    void ObjectSegmenter::run(const std::string& cloud_file_path, const std::string& indices_file_path, const std::string& result_path) const {
        IntVector raw_indices;
        PointIndicesTPtr point_indices_cloud;
        
        loadPointCloud(cloud_file_path);
        
        readIndicesFromFile(indices_file_path, raw_indices);

        // Map point indices to cloud points
        convertPointIndices(raw_indices, point_indices_cloud);

        // for(size_t i = 0; i < raw_indices.size(); i++)
        // {
        //     PCL_INFO("point index at %d is %d\n", (int)i, point_indices_cloud->indices[i]);
        // }

        segment(point_indices_cloud);

        PCL_INFO("Object cloud has %d points\n", (int)object_cloud->points.size());

        saveResults(result_path);

    }

    void ObjectSegmenter::loadPointCloud(const std::string& cloud_file_path) const {
        std::cout << "load Point Cloud from file.\n";
        
        pcl::io::loadPCDFile(cloud_file_path, *scene_cloud);
        PCL_INFO("Scene cloud has %d points\n", (int)scene_cloud->points.size());
    }
    
    void ObjectSegmenter::readIndicesFromFile(const std::string& file_path, IntVector& indices_data) const {
        std::cout << "read indices from file.\n";
        std::ifstream fp;
        fp.open(file_path.c_str(), std::ifstream::in);

        if (!fp) {
            std::cout << "Cannot open file.\n";
            exit(0);
        }

        int index;
        while (fp >> index) {
            indices_data.push_back(index);
        }

        std::cout << "\n";

        fp.close();
    }

    void ObjectSegmenter::convertPointIndices(const IntVector& raw_indices, PointIndicesTPtr& point_indices_cloud) const {
        std::cout << "convert point indices\n";

        point_indices_cloud.reset(new PointIndicesT());
        point_indices_cloud->indices.resize(raw_indices.size());

        for (size_t i = 0; i < raw_indices.size(); ++i) {
            point_indices_cloud->indices[i] = raw_indices[i];
        }

        // for(size_t i = 0; i < raw_indices.size(); i++)
        // {
        //     PCL_INFO("point index at %d is %d\n", (int)i, point_indices_cloud->indices[i]);
        // }

    }
    
    void ObjectSegmenter::segment(const PointIndicesTPtr& indices_data) const {
        pcl::ExtractIndices<PointInT> ei_filter(true);
        ei_filter.setInputCloud(scene_cloud);
        ei_filter.setIndices(indices_data);
        ei_filter.filter(*object_cloud);
    }
    
    void ObjectSegmenter::saveResults(const std::string& result_path) const {
        std::stringstream ss;

        ss << result_path << "/object.pcd";
        std::string object_pcd = ss.str();

        pcl::io::savePCDFileBinary(object_pcd.c_str(), *object_cloud);

        PCL_INFO("Object cloud was saved in %s\n", object_pcd.c_str());
    }

}
