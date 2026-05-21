#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <netinet/in.h>

namespace chassis_driver
{

/**
 * @brief Thin UDP socket wrapper for fixed remote endpoint communication.
 * @brief 面向固定远端端点通信的轻量 UDP socket 封装。
 */
class UdpChannel
{
public:
  UdpChannel() = default;

  /** @brief Close socket and release resources. */
  /** @brief 关闭 socket 并释放资源。 */
  ~UdpChannel();

  /**
   * @brief Open UDP socket with local bind and remote target.
   * @return True when socket, bind and endpoint setup succeed.
   * @brief 打开 UDP socket，绑定本地端点并配置远端目标。
   * @return socket 创建、绑定和端点配置均成功时返回 true。
   */
  bool open(const std::string & local_ip, uint16_t local_port, const std::string & remote_ip,
    uint16_t remote_port, int socket_timeout_ms, int recv_buffer_size);

  /**
   * @brief Receive one UDP datagram into output byte buffer.
   * @param out_payload Received payload bytes.
   * @return True when datagram is received.
   * @brief 接收一个 UDP 数据报到输出字节缓冲区。
   * @param out_payload 接收到的载荷字节。
   * @return 成功接收到数据报时返回 true。
   */
  bool receive(std::vector<uint8_t> & out_payload);

  /**
   * @brief Send one UDP datagram to configured remote endpoint.
   * @param payload Payload bytes.
   * @return True when full payload is sent.
   * @brief 向配置好的远端端点发送一个 UDP 数据报。
   * @param payload 待发送的载荷字节。
   * @return 完整载荷发送成功时返回 true。
   */
  bool send(const std::vector<uint8_t> & payload);

  /** @brief Idempotently close socket and clear state. */
  /** @brief 幂等关闭 socket 并清理状态。 */
  void close();

private:
  int fd_{-1};
  bool opened_{false};
  struct sockaddr_in remote_addr_{};
  bool remote_addr_valid_{false};
  std::vector<uint8_t> rx_buffer_;
};

}  // namespace chassis_driver
