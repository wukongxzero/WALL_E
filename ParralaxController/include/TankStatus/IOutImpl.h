#pragma once

class IOutImpl {
public:
  // outputs a byte string

  virtual unsigned char *listen();
  // return error status on failed send(assiming handshake but not sure yet)
  virtual int send(unsigned char *bytesToWrite);

private:
};
