// CANReceiver.cpp
#include "CANReceiver.hpp"

CANReceiver::CANReceiver(const std::string& dbc_file, const std::string& can_interface)
    : last_received(0), message_timeout(std::chrono::seconds(2)) {
    std::ifstream idbc(dbc_file);
    net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    if (!net) {
        throw std::runtime_error("Failed to parse DBC file.");
    }

    for (const auto& msg : net->Messages()) {
        messages[msg.Id()] = &msg;
    }

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        throw std::runtime_error("Error while opening socket.");
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface.c_str());
    ioctl(sock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
}

CANReceiver::~CANReceiver() {
    close(sock);
}

void CANReceiver::receiveMessages() {
    struct can_frame frame;
    int nbytes = read(sock, &frame, sizeof(struct can_frame));
    if (nbytes > 0) {
        auto current_time = std::chrono::steady_clock::now();
        auto iter = messages.find(frame.can_id);
        if (iter != messages.end()) {
            const auto& msg = *iter->second;
            if (msg.Id() == 254 || (msg.Id() == 255 && (last_received != 254 || std::chrono::duration_cast<std::chrono::seconds>(current_time - last_received_time) >= message_timeout))) {
                printMessage(msg, frame);
                last_received = msg.Id();
                last_received_time = current_time;
            }
        }
    }

    // Check timeout for switching back to SpeedMessage1
    if (last_received == 254) {
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_received_time) >= message_timeout) {
            last_received = 0; // Allow SpeedMessage1 to be printed again
        }
    }
}

void CANReceiver::printMessage(const dbcppp::IMessage& msg, const can_frame& frame) {
    std::cout << "Received Message: " << msg.Name() << "\n";
    for (const auto& sig : msg.Signals()) {
        std::cout << "\t" << sig.Name() << " = " << sig.RawToPhys(sig.Decode(frame.data)) << " " << sig.Unit() << "\n";
    }
}
