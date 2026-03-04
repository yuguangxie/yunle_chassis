#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <netinet/in.h>

namespace chassis_driver
{

/**
 * @brief Thin UDP socket wrapper for fixed remote endpoint communication.
 */
class UdpChannel
{
public:
  UdpChannel() = default;

  /** @brief Close socket and release resources. */
  ~UdpChannel();

  /**
   * @brief Open UDP socket with local bind and remote target.
   * @return True when socket, bind and endpoint setup succeed.
   */
  bool open(const std::string & local_ip, uint16_t local_port, const std::string & remote_ip,
    uint16_t remote_port, int socket_timeout_ms, int recv_buffer_size);

  /**
   * @brief Receive one UDP datagram into output byte buffer.
   * @param out_payload Received payload bytes.
   * @return True when datagram is received.
   */
  bool receive(std::vector<uint8_t> & out_payload);

  /**
   * @brief Send one UDP datagram to configured remote endpoint.
   * @param payload Payload bytes.
   * @return True when full payload is sent.
   */
  bool send(const std::vector<uint8_t> & payload);

  /** @brief Idempotently close socket and clear state. */
  void close();

private:
  int fd_{-1};
  bool opened_{false};
  struct sockaddr_in remote_addr_{};
  bool remote_addr_valid_{false};
  std::vector<uint8_t> rx_buffer_;
};

}  // namespace chassis_driver
