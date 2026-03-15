#pragma once
#include "IOutImpl.h"

class ABytePublisher {
public:
  IOutImpl byteSender;
  virtual void listen();
  virtual void write();
  virtual void connect();
};
