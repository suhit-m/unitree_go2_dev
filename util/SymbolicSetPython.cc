#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "cuddObj.hh"
#include "SymbolicSet.hh"

namespace py = pybind11;

PYBIND11_MODULE(SymbolicSetPython, m) {

    py::class_<Cudd>(m, "Cudd")
        .def(py::init<>());

    py::class_<scots::SymbolicSet>(m, "SymbolicSet")
    .def(py::init<Cudd&, const char*, int>(),
         py::arg("ddmgr"), py::arg("filename"), py::arg("newID")=0)
    .def("isElement", &scots::SymbolicSet::isElement)
    .def("setValuedMap", &scots::SymbolicSet::setValuedMap)
    .def("printInfo", &scots::SymbolicSet::printInfo);
}