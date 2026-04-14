#include "TankStatusClass.h"
#include <pybind11/pybind11.h>

extern "C" {
#include "lib/TankStatus.h" // Your C header
}

namespace py = pybind11;

PYBIND11_MODULE(tankstatus_wrapper, m) {
  m.doc() = "C wrapper for TankStatus.h";
  // Expose your C function: .def("python_name", &c_function_name)
  m.def("make_tankstatus", &makeByteTankStatus,
        "A function that adds two numbers");
  m.def("read_tankstatus", &readByteTankStatus,
        "A function that adds two numbers");

  py::class_<TankStatus>(m, "TankStatusStruct")
      .def(py::init<>())                                      // Constructor
      .def_readwrite("drive_left", &TankStatus::driveLeft)    // Direct access
      .def_readwrite("drive_right", &TankStatus::driveRight)  // Direct access
      .def_readwrite("euler_x", &TankStatus::eulerX)          // Direct access
      .def_readwrite("euler_y", &TankStatus::eulerY)          // Direct access
      .def_readwrite("euler_z", &TankStatus::eulerZ)          // Direct access
      .def_readwrite("change_flag", &TankStatus::changeFlag); // Direct access
                                                              //
  py::class_<TankStatusClass>(m, "TankStatusClass")
      .def(py::init<>())

      // Expose public float fields
      .def_readwrite("eulerXFloat", &TankStatusClass::eulerXFloat)
      .def_readwrite("eulerYFloat", &TankStatusClass::eulerYFloat)
      .def_readwrite("eulerZFloat", &TankStatusClass::eulerZFloat)

      .def_property(
          "drive_left",
          [](TankStatusClass &self) {
            return self.driveLeft ? *self.driveLeft : 0;
          }, // Getter
          [](TankStatusClass &self, unsigned char val) {
            if (self.driveLeft)
              *self.driveLeft = val;
          } // Setter
          )
      .def_property(
          "drive_right",
          [](TankStatusClass &self) {
            return self.driveRight ? *self.driveRight : 0;
          },
          [](TankStatusClass &self, unsigned char val) {
            if (self.driveRight)
              *self.driveRight = val;
          })
      .def_property(
          "euler_x",
          [](TankStatusClass &self) { return self.eulerX ? *self.eulerX : 0; },
          [](TankStatusClass &self, short val) {
            if (self.eulerX)
              *self.eulerX = val;
          })
      .def_property(
          "euler_y",
          [](TankStatusClass &self) { return self.eulerY ? *self.eulerY : 0; },
          [](TankStatusClass &self, short val) {
            if (self.eulerY)
              *self.eulerY = val;
          })
      .def_property(
          "euler_z",
          [](TankStatusClass &self) { return self.eulerZ ? *self.eulerZ : 0; },
          [](TankStatusClass &self, short val) {
            if (self.eulerZ)
              *self.eulerZ = val;
          })
      .def_property(
          "change_flag",
          [](TankStatusClass &self) {
            return self.changeFlag ? *self.changeFlag : false;
          },
          [](TankStatusClass &self, bool val) {
            if (self.changeFlag)
              *self.changeFlag = val;
          })

      // Expose the packet length constant
      .def_readonly("packet_length", &TankStatusClass::packetLength)

      // Methods
      .def("make_into_bytes",
           [](TankStatusClass &self) {
             // Returns the buffer as a Python 'bytes' object
             return py::bytes((char *)self.MakeIntoBytes(),
                              TANKSTATUS_PACKET_LENGTH);
           })

      .def("build_from_bytes",
           [](TankStatusClass &self, py::buffer b) {
             py::buffer_info info = b.request();

             // Check size
             if (info.size != TANKSTATUS_PACKET_LENGTH) {
               throw std::runtime_error(
                   "Invalid packet length: expected " +
                   std::to_string(TANKSTATUS_PACKET_LENGTH));
             }

             // Pass the pointer to the raw data
             self.BuildFromBytes((unsigned char *)info.ptr);
           })

      /*
         Handling 'volatile' pointers for Python:
         Python cannot directly "bind" to a raw C++ pointer to see live updates
         unless you use a property getter that dereferences them.
      */
      .def_property_readonly("change_flag",
                             [](TankStatusClass &self) {
                               return self.changeFlag ? *self.changeFlag
                                                      : false;
                             })
      .def_property_readonly("eulerX", [](TankStatusClass &self) {
        return self.eulerX ? *self.eulerX : 0;
      });
  // Repeat for driveLeft, driveRight, etc.
}
