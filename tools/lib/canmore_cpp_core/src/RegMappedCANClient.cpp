#include "canmore_cpp/RegMappedClient.hpp"

#include <chrono>

using namespace Canmore;

RegMappedCANClient::RegMappedCANClient(int ifIndex, uint8_t clientId, uint8_t channel):
    CANSocket(ifIndex), clientId(clientId), channel(channel) {
    // Configure filter for specific client/channel
    struct can_filter rfilter[] = { { .can_id = CANMORE_CALC_UTIL_ID_C2A(clientId, channel),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK) } };
    setRxFilters(std::span { rfilter });

    // Configure reg_mapped_client struct for linuxmappings
    clientCfg.tx_func = &clientTxCB;
    clientCfg.clear_rx_func = &clearRxCB;
    clientCfg.rx_func = &clientRxCB;
    clientCfg.transfer_mode = TRANSFER_MODE_BULK;
    clientCfg.arg = this;
    clientCfg.timeout_ms = REG_MAPPED_TIMEOUT_MS;
    clientCfg.max_in_flight = REG_MAPPED_MAX_IN_FLIGHT_PACKETS_CAN;
}

void RegMappedCANClient::sendRaw(const std::span<const uint8_t> &data) {
    transmitFrame(CANMORE_CALC_UTIL_ID_A2C(clientId, channel), data);
}

bool RegMappedCANClient::clientRx(const std::span<uint8_t> &buf, unsigned int timeoutMs) {
    if (buf.size_bytes() > CAN_MAX_DLEN) {
        return false;
    }

    // Run a simple PollGroup with just this FD
    // The frame handler will put the frame into the mailbox
    PollGroup group;
    group.addFd(*this);

    // We need to keep track of the time elapsed since signals can make PollGroup exit before timeoutMs, without
    // the fd processing an event
    auto start_time = std::chrono::high_resolution_clock::now();
    int remainingMs = timeoutMs;
    do {
        // Process events until we fill the mailbox or we run out of time
        group.processEvent(remainingMs);

        remainingMs = timeoutMs - std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::high_resolution_clock::now() - start_time)
                                      .count();
    } while (remainingMs > 0 && !frameMailbox);

    if (!frameMailbox) {
        // Nothing in the mailbox, we didn't receive data in time
        return false;
    }

    if (frameMailbox->first != CANMORE_CALC_UTIL_ID_C2A(clientId, channel)) {
        // Invalid client ID
        frameMailbox.reset();
        return false;
    }

    if (frameMailbox->second.size() != buf.size_bytes()) {
        // Unexpected length
        frameMailbox.reset();
        return false;
    }

    // Copy the data and free the mailbox
    std::copy(frameMailbox->second.begin(), frameMailbox->second.end(), buf.data());
    frameMailbox.reset();

    return true;
}
