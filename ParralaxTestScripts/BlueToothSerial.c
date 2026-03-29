#include "include/TankStatus.h"
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

int bitBackCounter = 0;

void printTankStatus(struct TankStatus *ts) {
  printf("driveLeft%uc|driveRight%uc|eulerY%f\n", ts->driveLeft, ts->driveRight,
         ts->eulerY);
}

struct TankStatus dummyData;
struct TankStatus dummyData2;

unsigned char tankStatusBufferOut[16];
unsigned char tankStatusBufferIn[16];

int main(int argc, char **argv) {
  struct sockaddr_rc addr = {0};
  int s, status;
  char dest[18];
  char buf[1024];

  constructTankStatus(&dummyData);
  constructTankStatus(&dummyData2);
  dummyData.driveLeft = 130;
  dummyData.driveRight = 155;
  dummyData.eulerY = 45; // for easy round animation

  // // Test bytre status again

  makeByteTankStatus(tankStatusBufferOut, 16, &dummyData);
  // for (int i = 0; i < 16; i++) {
  //   printf("buffer point %i", i);
  //   printf("%02X \n", tankStatusBuffer[i]);
  // }

  // printf("dummyData2 print status");
  // // readByteTankStatus(tankStatusBuffer, 16, &dummyData2);
  // printf("dummyData2:%i,%i,%f\n", dummyData2.driveLeft,
  // dummyData2.driveRight,
  //        dummyData2.eulerY);

  printTankStatus(&dummyData);
  printTankStatus(&dummyData2);

  if (argc < 2) {
    fprintf(stderr, "Usage: %s <HC-06 MAC ADDRESS>\n", argv[0]);
    exit(1);
  }
  strncpy(dest, argv[1], 18);

  // 1. Create a Bluetooth Socket
  s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

  // 2. Set the connection parameters (Channel 1 is default for HC-06)
  addr.rc_family = AF_BLUETOOTH;
  addr.rc_channel = (uint8_t)1;
  str2ba(dest, &addr.rc_bdaddr);

  printf("Connecting to %s...\n", dest);
  // 3. Connect to the HC-06
  status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

  if (status == 0) {
    printf("Connected! Type anything to send. Ctrl+C to exit.\n\n");
  } else {
    perror("Connect failed");
    close(s);
    return 1;
  }

  // 4. Multiplexing Loop (Keyboard <-> Bluetooth)
  struct pollfd fds[2];
  fds[0].fd = STDIN_FILENO; // Keyboard
  fds[0].events = POLLIN;
  fds[1].fd = s; // Bluetooth Socket
  fds[1].events = POLLIN;

  while (poll(fds, 2, -1) > 0) {
    // Data from Keyboard -> Send to HC-06
    if (fds[0].revents & POLLIN) {
      int bytes_read = read(STDIN_FILENO, buf, sizeof(buf));
      if (bytes_read > 0) {
        printf("tankstatusMode%s\n", tankStatusBufferOut);
        // write(s, buf, bytes_read);
        dummyData.eulerY += 1;
        makeByteTankStatus(tankStatusBufferOut, 16, &dummyData);

        write(s, tankStatusBufferOut, 16);
      }
    }

    // Data from HC-06 -> Print to Screen
    if (fds[1].revents & POLLIN) {
      int bytesReceived = recv(s, buf, sizeof(buf) - 1, 0);

      printf("bytes recieved%i", bytesReceived);

      if (bytesReceived > 0) {

        for (int i = 0; i < bytesReceived; i++) {
          printf("bytes recieved%i,content%X02\n", bytesReceived, buf[i]);
          tankStatusBufferIn[bitBackCounter] = buf[i];
          bitBackCounter++;
        }
        if (bitBackCounter == 16) {
          readByteTankStatus(&tankStatusBufferIn, 16, &dummyData2);
          printTankStatus(&dummyData2);
        }

        bitBackCounter %= 16;

      } else if (bytesReceived == 0) {
        printf("\nConnection closed by remote host.\n");
        break;
      }

      // clear buffer
      // memset(tankStatusBufferIn, 0, sizeof(tankStatusBufferIn));
      fflush(stdout);
    }
  }

  close(s);
  return 0;
}
