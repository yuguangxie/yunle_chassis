#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <netinet/in.h>

namespace chassis_driver
{

class UdpChannel
{
public:
  UdpChannel() = default;
  ~UdpChannel();

  bool open(const std::string & local_ip, uint16_t local_port, const std::string & remote_ip,
    uint16_t remote_port, int socket_timeout_ms, int recv_buffer_size);
  bool receive(std::vector<uint8_t> & out_payload);
  bool send(const std::vector<uint8_t> & payload);
  void close();

private:
  int fd_{-1};
  bool opened_{false};
  struct sockaddr_in * remote_addr_ptr_{nullptr};
  std::vector<uint8_t> rx_buffer_;
};

}  // namespace chassis_driver
