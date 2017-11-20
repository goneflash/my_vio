#ifndef MAV_DATASET_LOADER_HPP_
#define MAV_DATASET_LOADER_HPP_

#include <iostream>
#include <fstream>
#include <map>
#include <string>

using namespace std;

class MavDatasetLoader {
  public:
    typedef int64_t ImageTimeStamp;
    explicit MavDatasetLoader(const std::string &dataset_path);
    MavDatasetLoader() = delete;

    bool IsDatasetValid();

  private:
    bool LoadCalibration();
    bool LoadCameraImages() {
      const std::string camera_timestamp_file = "cam0.csv";
      ifstream camera_file(dataset_path_ + camera_timestamp_file);
      if (camera_file.is_open()) {
        std::string line;
        // Skip first line
        getline(camera_file, line);
        while (getline(camera_file, line)) {
          const size_t split_pos = line.find_first_of(',');
          const std::string timestamp_str = line.substr(0, split_pos + 1);
          const std::string image_name = line.substr(split_pos + 1, string::npos);

          std::cout << timestamp_str << std::endl;
        }
         
      }
      camera_file.close();
    }

    std::map<ImageTimeStamp, std::string> timestamp_to_image_name_;

    std::string dataset_path_;

};

#endif // MAV_DATASET_LOADER_HPP_

