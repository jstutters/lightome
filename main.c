#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include "lo/lo.h"
 
int initializeSerialPort() { 
  int fd;
  struct termios config;

  fd = open("/dev/tty.usbserial-m256-275", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    perror("Failed to open serial port");
    return -1;
  }

  if (tcgetattr(fd, &config) < 0) {
    perror("Unable to get serial port attributes");
    return -1;
  }
  
  // Set the serial port baud rate
  cfsetispeed(&config, B115200);
  cfsetospeed(&config, B115200);

  // 8N1
  config.c_cflag &= ~PARENB;
  config.c_cflag &= ~CSTOPB;
  config.c_cflag &= ~CSIZE;
  config.c_cflag |= CS8;
  // no flow control
  config.c_cflag &= ~CRTSCTS;

  config.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  config.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  config.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  config.c_cc[VMIN]  = 0;
  config.c_cc[VTIME] = 20;
  
  if( tcsetattr(fd, TCSANOW, &config) < 0) {
      perror("init_serialport: Couldn't set term attributes");
      return -1;
  }

  tcflush(fd, TCIOFLUSH);
  return fd;
}

int serialWriteByte(int fd, uint8_t b) {
  int n = write(fd, &b, 1);
  if (n != 1) {
    return -1;
  }
  return 0;
}

void sendOscMessage(char* path, int* args, int numArgs) {
  int i;
  lo_message msg = lo_message_new();
  for (i = 0; i < numArgs; i++) {
    lo_message_add_int32(msg, args[i]);
  }
  lo_address addr = lo_address_new("127.0.0.1", "57120");
  lo_send_message(addr, path, msg);
}

int ledOnHandler(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *userData) {
  printf("Got led_on message\n");
  fflush(stdout);
  int fd = *(int*)userData;
  uint8_t x = argv[0]->i;
  uint8_t y = argv[1]->i;
  uint8_t address = (y << 4) | x;
  uint8_t status = 0x2f;
  serialWriteByte(fd, status);
  serialWriteByte(fd, address);
  return 1;
}

int ledOffHandler(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *userData) {
  printf("Got led_off message\n");
  fflush(stdout);
  int fd = *(int*)userData;
  uint8_t x = argv[0]->i;
  uint8_t y = argv[1]->i;
  uint8_t address = (y << 4) | x;
  uint8_t status = 0x3f;
  serialWriteByte(fd, status);
  serialWriteByte(fd, address);
  return 1;
}

int clearHandler(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *userData) {
  printf("Clearing LEDs\n");
  fflush(stdout);
  int fd = *(int*)userData;
  uint8_t b = 0x90 | argv[0]->i;
  serialWriteByte(fd, b);
  return 1;
}

int genericHandler(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *userData) {
  printf("Got an unknown osc message\n");
  fflush(stdout);
  return 1;
}

void oscServerError(int num, const char *msg, const char *path) {
  printf("liblo server error\n");
  fflush(stdout);
}

int main(int argc, char** argv) {
  int fd;
  uint8_t cin;
  uint8_t status;
  uint8_t address;
  uint8_t x;
  uint8_t y;
  int lineCount = 0;
  int oscArgs[2];
  lo_server_thread oscServer;
  
  fd = initializeSerialPort();
  if (fd < 0) {
    printf("Unable to initialize serial port\n");
    return EXIT_FAILURE;
  }
  printf("Serial port initialized\n");
  
  oscServer = lo_server_thread_new("12000", oscServerError);
  printf("Server thread created\n");
  lo_server_thread_add_method(oscServer, "/lightome/led_on", "ii", ledOnHandler, &fd);
  lo_server_thread_add_method(oscServer, "/lightome/led_off", "ii", ledOffHandler, &fd);
  lo_server_thread_add_method(oscServer, "/lightome/clear", "i", clearHandler, &fd);
  lo_server_thread_add_method(oscServer, NULL, NULL, genericHandler, NULL);
  lo_server_thread_start(oscServer);
  printf("OSC server started\n");

  while (1) {
    if (read(fd, &cin, 1) > 0) {
      status = cin >> 4;
      read(fd, &cin, 1);
      address = cin;
      y = address >> 4;
      x = address & 0x0f;
      oscArgs[0] = x;
      oscArgs[1] = y;
      if (status == 0) {
        printf("Press  : %u, %u\n", x, y);
        sendOscMessage("/lightome/press", oscArgs, 2);
      } else if (status == 1) {
        printf("Release: %u, %u\n", x, y);
        sendOscMessage("/lightome/release", oscArgs, 2);
      }
      lineCount++;
    }
    usleep(500);
  }
  close(fd);
  return 0;
}
