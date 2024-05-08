#include <string>
#include <iostream>
#include <filesystem>

#include <gtest/gtest.h>

#include "ObjectSegmenter.h"

using namespace std::filesystem;

namespace point_cloud_utils {
    TEST(ObjectSegmenterTest, Initial) {
        std::string indices_file = "indices.txt";
        std::string cloud_file = "cloud.pcd";
        std::string result_path = "result";
        ObjectSegmenter segmenter;
        segmenter.run(cloud_file, indices_file, result_path);
        // ASSERT_TRUE(true);

        path dir_path = result_path;

        int files_number = 0;
        std::string result_file_name;
        for (const auto& file : directory_iterator(dir_path))
        {
            ++files_number;
            result_file_name = file.path().filename();
        }
        ASSERT_EQ(files_number, 1);
        ASSERT_EQ(result_file_name, "object.pcd");
    }
};


