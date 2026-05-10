#include "lib/TankStatusClass.h"
#include <pybind11/pybind11.h>

extern "C" {
#include "lib/TankStatus.h" 
}

namespace py = pybind11;

PYBIND11_MODULE(tankstatus_wrapper, m) {
  m.doc() = "C wrapper for TankStatus.h";

  // Note: I removed the binding for the raw C struct (TankStatusStruct) 
  // because Pybind11 works best interacting directly with your wrapper Class.

  py::class_<TankStatusClass>(m, "TankStatusClass")
      .def(py::init<>())

      // Because we removed the pointers, we can use simple def_readwrite!
      .def_readwrite("drive_left", &TankStatusClass::driveLeft)
      .def_readwrite("drive_right", &TankStatusClass::driveRight)
      .def_readwrite("euler_x", &TankStatusClass::eulerX)
      .def_readwrite("euler_y", &TankStatusClass::eulerY)
      .def_readwrite("euler_z", &TankStatusClass::eulerZ)
      .def_readwrite("change_flag", &TankStatusClass::changeFlag)

      // Expose public float fields
      .def_readwrite("eulerXFloat", &TankStatusClass::eulerXFloat)
      .def_readwrite("eulerYFloat", &TankStatusClass::eulerYFloat)
      .def_readwrite("eulerZFloat", &TankStatusClass::eulerZFloat)

      // Expose the packet length constant
      .def_readonly("packet_length", &TankStatusClass::packetLength)

      // Methods
      .def("make_into_bytes", [](TankStatusClass &self) {
          // Returns the buffer as a Python 'bytes' object
          return py::bytes((char *)self.MakeIntoBytes(), self.packetLength);
      })

      .def("build_from_bytes", [](TankStatusClass &self, py::bytes b) {
          std::string str(b); // Safely convert Python bytes
          if (str.length() >= self.packetLength) {
              self.BuildFromBytes((unsigned char *)str.data());
          } else {
              throw std::runtime_error("Invalid packet length");
          }
      });
}
