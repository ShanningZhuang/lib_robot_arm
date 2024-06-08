#pragma once

#include <string>
#include <fcntl.h> // O_RDWR
#include <iostream>
#include <vector>
#include <memory>

#include "libpcan.h"

#define STATUS DWORD
#define FAIL 1
#define SUCCESS 0

class CanCommunicator
{
  void *canHandler = nullptr;
  std::string devicePath;
  int flag;
  bool isOpen = false;
  uint16_t btr0btr1;
  uint16_t canMsgType;

public:
  CanCommunicator(std::string devicePath, uint16_t canMsgType, int flag = O_RDWR, uint16_t btr0btr1 = CAN_BAUD_1M)
      : devicePath(devicePath), flag(flag), btr0btr1(btr0btr1), canMsgType(canMsgType)
  {
  }
  ~CanCommunicator()
  {
    close();
  }

  STATUS open();
  STATUS close();
  STATUS init();

  STATUS get_version(std::string *info);
  STATUS get_status(STATUS status);

  STATUS send(DWORD id, std::vector<BYTE> &msg);
  STATUS receive(TPCANMsg &msg);
  STATUS receive_timeout(TPCANMsg &msg, int ms);
};