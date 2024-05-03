#pragma once

#include "utils/car_struct.h"
#include "utils/msg_structs.h"

#include "dbcppp/include/dbcppp/Network.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <string>
#include <unordered_map>

class CANSender {
public:
    CANSender(const std::string& dbc_file, const std::string& can_interface);
    ~CANSender();

private:
    int sock;
    uint64_t target_message_id;
    double speed_value;
    std::shared_ptr<const dbcppp::INetwork> net;
    std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;
};

