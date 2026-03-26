#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

int main(int argc, char **argv) {
  struct sockaddr_rc addr = {0};
  int s, status;
  char dest[18];
  char buf[1024];

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
        write(s, buf, bytes_read);
      }
    }

    // Data from HC-06 -> Print to Screen
    if (fds[1].revents & POLLIN) {
      int bytes_received = recv(s, buf, sizeof(buf) - 1, 0);
      if (bytes_received > 0) {
        buf[bytes_received] = '\0';
        printf("%s", buf);
        fflush(stdout);
      } else if (bytes_received == 0) {
        printf("\nConnection closed by remote host.\n");
        break;
      }
    }
  }

  close(s);
  return 0;
}
