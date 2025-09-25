#include "csv_logger.hpp"
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cerrno>


namespace wds_snapshot {

CsvLogger::CsvLogger(const std::string& base_path)
    : base_path_(base_path), current_batch_number_(1) {
}

CsvLogger::~CsvLogger() {
    if (hires_file_.is_open()) {
        hires_file_.close();
    }
    if (hires2_file_.is_open()) {
        hires2_file_.close();
    }
}

bool CsvLogger::createDirectories(const std::string& path) {
    // Create base directory
    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "Failed to create directory: " << path << std::endl;
        return false;
    }

    // Create subdirectories for cameras
    std::string hires_dir = path + "/hires";
    std::string hires2_dir = path + "/hires2";

    if (mkdir(hires_dir.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "Failed to create hires directory" << std::endl;
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
    hires_csv_path_ = base_path_ + "hires.csv";
    hires2_csv_path_ = base_path_ + "hires2.csv";
    // qvio_csv_path_ = base_path_ + "qvio_sensor.csv";

    // Open CSV files and write headers
    hires_file_.open(hires_csv_path_, std::ios::out | std::ios::app);
    if (!hires_file_.is_open()) {
        std::cerr << "Failed to open hires.csv" << std::endl;
        return false;
    }

    // Check if file is empty to write header
    hires_file_.seekp(0, std::ios::end);
    if (hires_file_.tellp() == 0) {
        hires_file_ << "Batch_ID,Frame_Number,Height,Width,Encoding,"
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


    std::cout << "[CsvLogger] Initialized CSV files in: " << base_path_ << std::endl;
    return true;
}

void CsvLogger::writeImageMetadata(const std::string& camera_name,
                                   const ImageMetadata& metadata) {
    std::ofstream* file_ptr = nullptr;
    std::mutex* mutex_ptr = nullptr;

    // Simplified mapping - no more hires
    if (camera_name.find("hires2") != std::string::npos) {
        file_ptr = &hires2_file_;
        mutex_ptr = &hires2_mutex_;
    } else {  // hires, hires_grey, hires_color all go to hires
        file_ptr = &hires_file_;  // Keep using hires_file_ internally
        mutex_ptr = &hires_mutex_;
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

void CsvLogger::setBasePath(const std::string& path) {
    base_path_ = path;
    if (!base_path_.empty() && base_path_.back() != '/') {
        base_path_ += '/';
    }
}


} // namespace wds_snapshot