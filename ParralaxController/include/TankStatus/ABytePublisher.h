#pragma once
#include "IOutImpl.h"
#include "ATankStatusPublisher.h"

class ABytePublisher:public ATankStatusPublisher {
public:
  IOutImpl byteSender;
  virtual void listen();
  virtual void write();
  virtual void connect();
};
