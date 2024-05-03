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

class CANReceiver {
public:
    CANReceiver(const std::string& dbc_file, const std::string& can_interface);
    ~CANReceiver();
    void receiveMessages();

private:
    void printMessage(const dbcppp::IMessage& msg, const can_frame& frame);

    std::unique_ptr<dbcppp::INetwork> net;
    std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;
    int sock;
    uint64_t last_received;
    std::chrono::time_point<std::chrono::steady_clock> last_received_time;
    std::chrono::seconds message_timeout;
};
