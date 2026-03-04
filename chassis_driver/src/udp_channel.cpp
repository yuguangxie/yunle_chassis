#include "chassis_driver/udp_channel.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>

namespace chassis_driver
{

UdpChannel::~UdpChannel()
{
  close();
}

bool UdpChannel::open(const std::string & local_ip, uint16_t local_port, const std::string & remote_ip,
  uint16_t remote_port, int socket_timeout_ms, int recv_buffer_size)
{
  close();
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    return false;
  }

  if (recv_buffer_size > 0) {
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &recv_buffer_size, sizeof(recv_buffer_size));
  }

  struct timeval tv;
  tv.tv_sec = socket_timeout_ms / 1000;
  tv.tv_usec = (socket_timeout_ms % 1000) * 1000;
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(local_port);
  if (::inet_pton(AF_INET, local_ip.c_str(), &local_addr.sin_addr) != 1) {
    close();
    return false;
  }

  if (::bind(fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
    close();
    return false;
  }

  remote_addr_ptr_ = new sockaddr_in();
  std::memset(remote_addr_ptr_, 0, sizeof(sockaddr_in));
  remote_addr_ptr_->sin_family = AF_INET;
  remote_addr_ptr_->sin_port = htons(remote_port);
  if (::inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_ptr_->sin_addr) != 1) {
    close();
    return false;
  }

  rx_buffer_.assign(static_cast<size_t>(recv_buffer_size > 0 ? recv_buffer_size : 2048), 0U);
  opened_ = true;
  return true;
}

bool UdpChannel::receive(std::vector<uint8_t> & out_payload)
{
  if (!opened_) {
    return false;
  }
  sockaddr_in src{};
  socklen_t len = sizeof(src);
  const ssize_t n = ::recvfrom(fd_, rx_buffer_.data(), rx_buffer_.size(), 0,
    reinterpret_cast<sockaddr *>(&src), &len);
  if (n <= 0) {
    return false;
  }
  out_payload.assign(rx_buffer_.begin(), rx_buffer_.begin() + n);
  return true;
}

bool UdpChannel::send(const std::vector<uint8_t> & payload)
{
  if (!opened_ || remote_addr_ptr_ == nullptr) {
    return false;
  }
  const ssize_t n = ::sendto(fd_, payload.data(), payload.size(), 0,
    reinterpret_cast<sockaddr *>(remote_addr_ptr_), sizeof(sockaddr_in));
  return n == static_cast<ssize_t>(payload.size());
}

void UdpChannel::close()
{
  if (fd_ >= 0) {
    ::shutdown(fd_, SHUT_RDWR);
    ::close(fd_);
    fd_ = -1;
  }
  if (remote_addr_ptr_ != nullptr) {
    delete remote_addr_ptr_;
    remote_addr_ptr_ = nullptr;
  }
  opened_ = false;
}

}  // namespace chassis_driver
