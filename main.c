/* -------------------------------------------------------------------------------
 * Lightome - serial to OSC bridge for the Monome
 * Copyright (C) 2011
 *
 * Jonny Stutters <jstutters@jeremah.co.uk>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------------ */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <dirent.h>
#include "lo/lo.h"

#define FALSE 0
#define TRUE 1

char* pathToMonome() {
  DIR *dfd;
  struct dirent *dp;
  dfd = opendir("/dev");
  char* monomePath = NULL;
  char* monomeFile = NULL;
  char* m256 = NULL;
  char* m128 = NULL;
  char* m64 = NULL;
  if (dfd != NULL) {
    while ((dp = readdir(dfd)) != NULL) {
      m256 = strstr(dp->d_name, "tty.usbserial-m256");
      m128 = strstr(dp->d_name, "tty.usbserial-m128");
      m64 = strstr(dp->d_name, "tty.usbserial-m64");

      if (m256 || m128 || m64) {
        monomeFile = dp->d_name;
        printf("%s\n", monomeFile);
        monomePath = malloc(6 + strlen(monomeFile));
        printf("Memory allocated\n");
        strcpy(monomePath, "/dev/");
        strcpy(monomePath + 5, monomeFile);
      }
    }
    closedir(dfd);
  } else {
    perror("Couldn't get directory listing for /dev\n");
  }
  return monomePath;
}

int initializeSerialPort(char* monomePath) { 
  int fd;
  struct termios config;

  printf("Initializing serial port at %s\n", monomePath);
  fd = open(monomePath, O_RDWR | O_NOCTTY);
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
  config.c_cc[VMIN]  = 2;
  config.c_cc[VTIME] = 0;
  
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

void printHelp() {
  printf("usage: lightome [-o tcp_send_port] [-i tcp_listen_port] [-p path_to_monome_tty]\n");
}

int main(int argc, char** argv) {
  char* monomePath = NULL;
  int fd;
  uint8_t cin[2];
  uint8_t status;
  uint8_t address;
  uint8_t x;
  uint8_t y;
  int lineCount = 0;
  int oscArgs[2];
  char* inputOscPort = NULL;
  char* outputOscPort = NULL;
  int c;
  uint8_t inputPortSet = FALSE;
  uint8_t outputPortSet = FALSE;
  uint8_t monomePathSet = FALSE;
  lo_server_thread oscServer;

  while ((c = getopt(argc, argv, "hi:o:p:")) != -1) {
    switch (c) {
      case 'i':
        inputOscPort = optarg;
        inputPortSet = TRUE;
        break;
      case 'o':
        outputOscPort = optarg;
        outputPortSet = TRUE;
        break;
      case 'p':
        monomePath = optarg;
        monomePathSet = TRUE;
        break;
      case 'h':
        printHelp();
        return EXIT_SUCCESS;
      default:
        abort();
    }
  }
  if (inputPortSet == FALSE) {
    inputOscPort = "9000";
  }
  if (outputPortSet == FALSE) {
    outputOscPort = "57120";
  }

  printf("Input port: %s, output port: %s\n", inputOscPort, outputOscPort);
 
  if (monomePathSet == FALSE) {
    printf("Looking for Monome\n");
    monomePath = pathToMonome();
  }

  fd = initializeSerialPort(monomePath);
  if (fd < 0) {
    printf("Unable to initialize serial port\n");
    return EXIT_FAILURE;
  }
  printf("Serial port initialized\n");
  if (monomePathSet == FALSE) {
    free(monomePath);
    monomePath = NULL;
  }
  
  oscServer = lo_server_thread_new("12000", oscServerError);
  printf("Server thread created\n");
  lo_server_thread_add_method(oscServer, "/lightome/led_on", "ii", ledOnHandler, &fd);
  lo_server_thread_add_method(oscServer, "/lightome/led_off", "ii", ledOffHandler, &fd);
  lo_server_thread_add_method(oscServer, "/lightome/clear", "i", clearHandler, &fd);
  lo_server_thread_add_method(oscServer, NULL, NULL, genericHandler, NULL);
  lo_server_thread_start(oscServer);
  printf("OSC server started\n");
  printf("Press ctrl-c to exit\n");

  while (1) {
    if (read(fd, &cin, 2) > 0) {
      status = cin[0] >> 4;
      address = cin[1];
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
  }
  close(fd);
  return 0;
}
