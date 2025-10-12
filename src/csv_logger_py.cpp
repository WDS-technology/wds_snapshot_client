#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // for vector, string, etc.
#include "csv_logger.hpp" // include your C++ header

namespace py = pybind11;

PYBIND11_MODULE(py_csv_logger, m) {
    m.doc() = "Pybind11 bindings for the CsvLogger class.";

    // Bind ImageMetadata struct (now includes QVIO data)
    py::class_<wds_snapshot::ImageMetadata>(m, "ImageMetadata")
        .def(py::init<>())
        .def_readwrite("batch_id", &wds_snapshot::ImageMetadata::batch_id)
        .def_readwrite("frame_number", &wds_snapshot::ImageMetadata::frame_number)
        .def_readwrite("height", &wds_snapshot::ImageMetadata::height)
        .def_readwrite("width", &wds_snapshot::ImageMetadata::width)
        .def_readwrite("encoding", &wds_snapshot::ImageMetadata::encoding)
        .def_readwrite("image_file_path", &wds_snapshot::ImageMetadata::image_file_path)
        .def_readwrite("image_title", &wds_snapshot::ImageMetadata::image_title)
        .def_readwrite("position_x", &wds_snapshot::ImageMetadata::position_x)
        .def_readwrite("position_y", &wds_snapshot::ImageMetadata::position_y)
        .def_readwrite("position_z", &wds_snapshot::ImageMetadata::position_z)
        .def_readwrite("orientation_w", &wds_snapshot::ImageMetadata::orientation_w)
        .def_readwrite("orientation_x", &wds_snapshot::ImageMetadata::orientation_x)
        .def_readwrite("orientation_y", &wds_snapshot::ImageMetadata::orientation_y)
        .def_readwrite("orientation_z", &wds_snapshot::ImageMetadata::orientation_z);

    // Bind CsvLogger class
    py::class_<wds_snapshot::CsvLogger>(m, "CsvLogger")
        .def(py::init<const std::string&>())
        .def("initialize", &wds_snapshot::CsvLogger::initialize)
        .def("writeImageMetadata", &wds_snapshot::CsvLogger::writeImageMetadata)
        .def("getCurrentBatchNumber", &wds_snapshot::CsvLogger::getCurrentBatchNumber)
        .def("incrementBatchNumber", &wds_snapshot::CsvLogger::incrementBatchNumber)
        .def("setBasePath", &wds_snapshot::CsvLogger::setBasePath);
}