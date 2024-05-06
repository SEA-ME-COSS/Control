#pragma once

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

#include "dbcppp/Network.h"

class CANSender {
public:
    CANSender(const std::string& dbc_file, const std::string& can_interface);
    ~CANSender();
    void sendSpeedMessage(double speed);

private:
    int sock;
    uint64_t target_message_id;
    double speed_value;
    std::unique_ptr<dbcppp::INetwork> net;
    std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;
};
