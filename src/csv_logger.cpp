#include "csv_logger.hpp"
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace wds_snapshot {

CsvLogger::CsvLogger(const std::string& base_path)
    : base_path_(base_path), current_batch_number_(1) {
}

CsvLogger::~CsvLogger() {
    if (hires1_file_.is_open()) {
        hires1_file_.close();
    }
    if (hires2_file_.is_open()) {
        hires2_file_.close();
    }
    // if (qvio_file_.is_open()) {
    //     qvio_file_.close();
    // }
}

bool CsvLogger::createDirectories(const std::string& path) {
    // Create base directory
    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "Failed to create directory: " << path << std::endl;
        return false;
    }

    // Create subdirectories for cameras
    std::string hires1_dir = path + "/hires1";
    std::string hires2_dir = path + "/hires2";

    if (mkdir(hires1_dir.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "Failed to create hires1 directory" << std::endl;
        return false;
    }

    if (mkdir(hires2_dir.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "Failed to create hires2 directory" << std::endl;
        return false;
    }

    return true;
}

bool CsvLogger::initialize() {
    // Ensure base path ends with /
    if (!base_path_.empty() && base_path_.back() != '/') {
        base_path_ += '/';
    }

    // Create directories
    if (!createDirectories(base_path_)) {
        return false;
    }

    // Set CSV file paths
    hires1_csv_path_ = base_path_ + "hires1.csv";
    hires2_csv_path_ = base_path_ + "hires2.csv";
    // qvio_csv_path_ = base_path_ + "qvio_sensor.csv";

    // Open CSV files and write headers
    hires1_file_.open(hires1_csv_path_, std::ios::out | std::ios::app);
    if (!hires1_file_.is_open()) {
        std::cerr << "Failed to open hires1.csv" << std::endl;
        return false;
    }

    // Check if file is empty to write header
    hires1_file_.seekp(0, std::ios::end);
    if (hires1_file_.tellp() == 0) {
        hires1_file_ << "Batch_ID,Frame_Number,Height,Width,Encoding,"
                     << "Image_File_Path,Image_Title_JPG,"
                     << "Position_X,Position_Y,Position_Z,"
                     << "Orientation_W,Orientation_X,Orientation_Y,Orientation_Z\n";
    }

    hires2_file_.open(hires2_csv_path_, std::ios::out | std::ios::app);
    if (!hires2_file_.is_open()) {
        std::cerr << "Failed to open hires2.csv" << std::endl;
        return false;
    }

    hires2_file_.seekp(0, std::ios::end);
    if (hires2_file_.tellp() == 0) {
        hires2_file_ << "Batch_ID,Frame_Number,Height,Width,Encoding,"
                     << "Image_File_Path,Image_Title_JPG,"
                     << "Position_X,Position_Y,Position_Z,"
                     << "Orientation_W,Orientation_X,Orientation_Y,Orientation_Z\n";
    }

    // qvio_file_.open(qvio_csv_path_, std::ios::out | std::ios::app);
    // if (!qvio_file_.is_open()) {
    //     std::cerr << "Failed to open qvio_sensor.csv" << std::endl;
    //     return false;
    // }

    // qvio_file_.seekp(0, std::ios::end);
    // if (qvio_file_.tellp() == 0) {
    //     qvio_file_ << "Batch_ID,Timestamp,Position_X,Position_Y,"
    //                << "Position_Z,Orientation_W,Orientation_X,Orientation_Y,"
    //                << "Orientation_Z\n";
    // }

    std::cout << "[CsvLogger] Initialized CSV files in: " << base_path_ << std::endl;
    return true;
}

// std::string CsvLogger::timestampToHHMMSS(double timestamp) {
//     // Convert timestamp to time_t
//     std::time_t time_val = static_cast<std::time_t>(timestamp);

//     // Convert to local time structure
//     std::tm* local_time = std::localtime(&time_val);

//     // Format as HHMMSS
//     std::ostringstream oss;
//     oss << std::setfill('0') << std::setw(2) << local_time->tm_hour
//         << std::setw(2) << local_time->tm_min
//         << std::setw(2) << local_time->tm_sec;

//     return oss.str();
// }

void CsvLogger::writeImageMetadata(const std::string& camera_name,
                                   const ImageMetadata& metadata) {
    std::ofstream* file_ptr = nullptr;
    std::mutex* mutex_ptr = nullptr;

    if (camera_name == "hires1" || camera_name == "hires") {
        file_ptr = &hires1_file_;
        mutex_ptr = &hires1_mutex_;
    } else if (camera_name == "hires2") {
        file_ptr = &hires2_file_;
        mutex_ptr = &hires2_mutex_;
    } else {
        std::cerr << "[CsvLogger] Unknown camera: " << camera_name << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(*mutex_ptr);

    if (!file_ptr->is_open()) {
        std::cerr << "[CsvLogger] CSV file not open for camera: " << camera_name << std::endl;
        return;
    }

    // Write metadata row
    *file_ptr << metadata.batch_id << ","
              << metadata.frame_number << ","
              << metadata.height << ","
              << metadata.width << ","
              << metadata.encoding << ","
              << metadata.image_file_path << ","
              << metadata.image_title << ","
              << std::fixed << std::setprecision(6)
              << metadata.position_x << ","
              << metadata.position_y << ","
              << metadata.position_z << ","
              << metadata.orientation_w << ","
              << metadata.orientation_x << ","
              << metadata.orientation_y << ","
              << metadata.orientation_z << "\n";

    file_ptr->flush();
}

// void CsvLogger::writeQvioMetadata(const QvioMetadata& metadata) {
//     std::lock_guard<std::mutex> lock(qvio_mutex_);

//     if (!qvio_file_.is_open()) {
//         std::cerr << "[CsvLogger] QVIO CSV file not open" << std::endl;
//         return;
//     }

//     qvio_file_ << metadata.batch_id << ","
//                << metadata.timestamp_hhmmss << ","
//                << std::fixed << std::setprecision(6)
//                << metadata.position_x << ","
//                << metadata.position_y << ","
//                << metadata.position_z << ","
//                << metadata.orientation_w << ","
//                << metadata.orientation_x << ","
//                << metadata.orientation_y << ","
//                << metadata.orientation_z << "\n";

//     qvio_file_.flush();
// }

void CsvLogger::setBasePath(const std::string& path) {
    base_path_ = path;
    if (!base_path_.empty() && base_path_.back() != '/') {
        base_path_ += '/';
    }
}


} // namespace wds_snapshot