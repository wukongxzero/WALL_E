#pragma once

class IOutImpl {
public:
  // outputs a byte string

// TODO:if using simpleIDE define this in MAIN
#ifdef VIRTUALDEFINED

  // return error status on failed send(assiming handshake but not sure yet)
  virtual int send(unsigned char *bytesToWrite);
  virtual unsigned char *listen();
#else
  int send(unsigned char *bytesToWrite);
  unsigned char *listen();
#endif

private:
};
