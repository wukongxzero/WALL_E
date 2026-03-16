#pragma once
#include "IOutImpl.h"
#include "TankStatus/ATankStatusPublisher.h"

class ABytePublisher : public ATankStatusPublisher {
public:
  IOutImpl byteSender;

#ifdef VIRTUALDEFINED
  virtual void listen();
  virtual void write();
  virtual void connect();
#else
  void listen();
  void write();
  void connect();
#endif
};
