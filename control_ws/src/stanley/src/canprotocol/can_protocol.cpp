// CANReceiver.cpp
#include "CANReceiver.hpp"

CANSender::CANSender(const std::string& dbc_file, const std::string& can_interface)
    : target_message_id(255), speed_value(10.0) {
    std::ifstream idbc(dbc_file);
    this->net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    if (!this->net) {
        throw std::runtime_error("Failed to parse DBC file.");
    }

    for (const auto& msg : this->net->Messages()) {
        this->messages[msg.Id()] = &msg;
    }

    this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->sock < 0) {
        throw std::runtime_error("Error while opening socket.");
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface.c_str());
    ioctl(sock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(this->sock, (struct sockaddr *)&addr, sizeof(addr));
}

CANSender::~CANSender() {
    close(sock);
}

void CANSender::sendSpeedMessage(double speed) {
    auto iter = messages.find(target_message_id);
    if (iter == messages.end()) {
        throw std::runtime_error("Message with ID 255 not found in DBC.");
    }

    const auto& msg = *iter->second;
    std::vector<uint8_t> frame_data(8, 0);
    for (const auto& sig : msg.Signals()) {
        if (sig.Name() == "Speed") {
            auto raw_value = sig.PhysToRaw(speed);
            sig.Encode(raw_value, frame_data.data());
        }
    }

    struct can_frame frame;
    frame.can_id = target_message_id;
    frame.can_dlc = frame_data.size();
    std::memcpy(frame.data, frame_data.data(), frame_data.size());

    int nbytes = write(sock, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "Error while sending frame" << std::endl;
    } else {
        std::cout << "Message sent successfully: Speed = " << speed << std::endl;
    }
}

