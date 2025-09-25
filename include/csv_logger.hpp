#ifndef CSV_LOGGER_HPP
#define CSV_LOGGER_HPP

#include <string>
#include <fstream>
#include <mutex>
#include <memory>
#include <atomic>
#include <chrono>

#ifdef HAS_ROS
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#endif


namespace wds_snapshot {

    struct ImageMetadata {
        int batch_id = 0;                    // Add default initialization
        int frame_number = 0;
        int height = 0;
        int width = 0;
        std::string encoding = "";
        std::string image_file_path = "";
        std::string image_title = "";
        // QVIO position data
        double position_x = 0.0;
        double position_y = 0.0;
        double position_z = 0.0;
        double orientation_w = 1.0;          // Identity quaternion
        double orientation_x = 0.0;
        double orientation_y = 0.0;
        double orientation_z = 0.0;
    };

class CsvLogger {
public:
    CsvLogger(const std::string& base_path);
    ~CsvLogger();

    // Initialize CSV files with headers
    bool initialize();

    // Write image metadata for a camera (includes QVIO data)
    void writeImageMetadata(const std::string& camera_name,
                           const ImageMetadata& metadata);

    // Get current batch number
    int getCurrentBatchNumber() const { return current_batch_number_.load(); }

    // Increment batch number for new command
    void incrementBatchNumber() { current_batch_number_++; }


    // Set the base folder path where CSVs will be stored
    void setBasePath(const std::string& path);

private:
    std::string base_path_;
    std::string hires_csv_path_;
    std::string hires2_csv_path_;

    std::ofstream hires_file_;
    std::ofstream hires2_file_;

    std::mutex hires_mutex_;
    std::mutex hires2_mutex_;

    std::atomic<int> current_batch_number_;

    // Helper to create directories if they don't exist
    bool createDirectories(const std::string& path);
};


} // namespace wds_snapshot

#endif // CSV_LOGGER_HPP