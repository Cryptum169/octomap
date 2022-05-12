
#include "/home/alex/.local/lib/python3.8/site-packages/pybind11/include/pybind11/pybind11.h"
#include "/home/alex/.local/lib/python3.8/site-packages/pybind11/include/pybind11/numpy.h"
#include <memory>
#include <vector>

#include "octomap/math/Pose6D.h"
#include "octomap/octomap_types.h"
#include "octomap/Pointcloud.h"
#include "octomap/octomap.h"

namespace py = pybind11;

using namespace pybind11::literals;
using namespace std;

PYBIND11_MODULE(pyoctomap, m)
{
    m.doc() = "Octomap pybind11 Interface";

    py::class_<octomath::Vector3>(m, "Vector3", py::buffer_protocol())
        .def(py::init([](py::buffer b) {
            auto info = b.request();

            auto size = 1;
            for (const float v : info.shape) size *= v;

            if (size != float(3)) {
                throw std::runtime_error("Incoming Shape incorrect");
            } 

            if (info.itemsize != sizeof(float)) {
                throw std::runtime_error("Incoming type not float");
            }

            float *data_ptr = (float *) info.ptr;
            float x = data_ptr[0];
            float y = data_ptr[1];
            float z = data_ptr[2];

            return octomath::Vector3(x, y, z);
        }))
        .def_buffer([](octomath::Vector3 &m) -> py::buffer_info
                    { return py::buffer_info(
                          m.get_data(),
                          sizeof(float),
                          py::format_descriptor<float>::format(),
                          1,
                          {3},
                          {sizeof(float) * 3}); })
        .def(
            "rotate_IP", [](octomath::Vector3 *self, double roll, double yaw, double pitch)
            { return self->rotate_IP(roll, yaw, pitch); },
            py::arg("roll"), py::arg("yaw"), py::arg("pitch"))
        .def("numpy", [](octomath::Vector3 *self)
             { return py::array_t<float>(3, self->get_data()); });
    // .def("numpy", [](octomath::Vector3* self) {return self->get_data();});

    py::class_<octomath::Quaternion>(m, "Quaternion", py::buffer_protocol())
        .def(py::init<float, float, float, float>(),
             py::arg("uu"), py::arg("xx"), py::arg("yy"), py::arg("zz"))
        .def("numpy", [](octomath::Quaternion *self)
             { return py::array_t<float>(4, self->get_data()); });

    py::class_<octomath::Pose6D>(m, "Pose6D")
        .def(py::init<>())
        .def(py::init<octomath::Vector3, octomath::Quaternion>(),
             py::arg("trans"), py::arg("rot"))
        .def("trans", [](octomath::Pose6D *self)
             { return self->trans(); })
        .def("rot", [](octomath::Pose6D *self)
             { return self->rot(); });

    py::class_<octomap::Pointcloud>(m, "Pointcloud", py::buffer_protocol())
        .def(py::init([](py::array_t<float> b) {
            auto info = b.request();

            if (info.ndim < 2)
                { throw std::runtime_error("ndim < 2"); }

            if (info.shape[1] != 3)
                { throw std::runtime_error("dim 1 size != 3"); }

            octomap::Pointcloud pcd;

            auto r = b.mutable_unchecked<2>();
            for (py::ssize_t i = 0; i < r.shape(0); i++) {
                pcd.push_back(r(i, 0), r(i, 1), r(i, 2));
            }

            return pcd;
        }))
        .def("numpy", [](octomap::Pointcloud *self) {
            std::vector<size_t> shape = {self->size(), 3};
            py::array_t<float> arr(shape);

            py::buffer_info buf = arr.request();
            float *ptr = static_cast<float *>(buf.ptr);

            // // size_t idx = 0;
            for (size_t pt_idx = 0; pt_idx < self->size(); pt_idx++) {
                for (size_t dim_idx = 0; dim_idx < 3; dim_idx++) {
                    ptr[pt_idx * 3 + dim_idx] = self->getPoint(pt_idx)(dim_idx);
                }
            }

            return arr;
        });

    py::class_<octomap::OcTree>(m, "OcTree")
        .def(py::init<double>(), py::arg("resolution"))
        .def(py::init<std::string>(), py::arg("_filename"))
        .def("insertPointCloud", [](octomap::OcTree *self, octomap::Pointcloud pcd, octomath::Pose6D pose)
             { return self->insertPointCloud(pcd, octomath::Vector3(), pose); })
        .def("writeBinary", [](octomap::OcTree *self, std::string filename)
             { return self->writeBinary(filename); })
        .def("getPoints", [](octomap::OcTree *self) {
            unsigned int maxDepth = self->getTreeDepth();
            vector<octomap::OcTreeNode*> collapsed_occ_nodes;
            do {
                collapsed_occ_nodes.clear();
                for (octomap::OcTree::iterator it = self->begin(); it != self->end(); ++it)
                {
                    if(self->isNodeOccupied(*it) && it.getDepth() < maxDepth)
                    { collapsed_occ_nodes.push_back(&(*it)); }
                }

                for (vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
                { self->expandNode(*it); }
                // cout << "expanded " << collapsed_occ_nodes.size() << " nodes" << endl;
            } while(collapsed_occ_nodes.size() > 0);

            octomap::Pointcloud pcl;
            for (octomap::OcTree::iterator it = self->begin(); it != self->end(); ++it)
            {
                if(self->isNodeOccupied(*it))
                { pcl.push_back(it.getCoordinate()); }
            }

            return pcl;
        });
}
