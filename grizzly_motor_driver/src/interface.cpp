#include <string>

#include "grizzly_motor_driver/interface.h"

namespace grizzly_motor_driver
{

Interface::Interface(const std::string &can_device) :
  can_device_(can_device),
  connected_(false)
{
}

Interface::~Interface()
{
  if (connected_)
  {
    disconnect();
  }
}

bool Interface::connect()
{
  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
      std::cout << "SocketCAN Interface: Error while opening socket." << std::endl;
      return false;
    }

    struct ifreq ifr;

    snprintf (ifr.ifr_name, sizeof(can_device_.c_str()), "%s", can_device_.c_str());

    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
    {
      close(socket_);
      std::cout << "SocketCAN Interface: Error while trying to control device." << std::endl;
      connected_ = false;
      return connected_;
    }

    struct sockaddr_can addr;
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    std::cout << "SocketCAN Interface: " << can_device_.c_str() << " at index " <<  ifr.ifr_ifindex << "." << std::endl;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      std::cout << "SocketCAN Interface: Error in socket bind." << std::endl;
      connected_ = false;
      return connected_;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1;

    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::cout << "SocketCAN Interface: Opened socket on " << can_device_.c_str() << "." << std::endl;
    connected_ = true;

    return connected_;
}

bool Interface::disconnect()
{
  int ret = close(socket_);
  if (ret == 0)
  {
    std::cout << "SocketCAN Interface: Closed Socket on CAN device: " << can_device_.c_str() << "." << std::endl;
    connected_ = false;
  }
  else
  {
    std::cout << "SocketCAN Interface: Unable to close Socket CAN on " << can_device_.c_str()
      << " due to errno " << ret << "." << std::endl;
  }
  return connected_;
}

bool Interface::isConnected() const
{
  return connected_;
}

bool Interface::connectIfNotConnected()
{
  return connected_;
}

bool Interface::receive(Frame *frame)
{
  can_frame read_frame;

  int8_t ret = read(socket_, &read_frame, sizeof(struct can_frame));

  if (ret == sizeof(struct can_frame))
  {
    // printCanFrame(read_frame);
    *frame = fromCanFrame(read_frame);
    return true;
  }
  else
  {
    if (ret < 0)
    {
      if (errno == EAGAIN)
      {
        std::cout << "SocketCAN Interface: No more frames." << std::endl;
      }
      else
      {
        std::cout << "SocketCAN Interface: Error reading from socket due to errno " << errno << "." << std::endl;
      }
    }
    else
    {
    std::cout << "SocketCAN Interface: Error reading returned unexpected size." << std::endl;
    }
    return false;
  }
}

void Interface::queue(const Frame &frame)
{
  queue_outbound_.push_back(toCanFrame(frame));
}

void Interface::queue(const can_frame &send_frame)
{
  queue_outbound_.push_back(send_frame);
}

bool Interface::send(const can_frame *send_frame)
{
  int8_t ret = write(socket_, send_frame, sizeof(struct can_frame));
  if (ret == -1)
  {
    std::cout << "SocketCAN Interface: Error in sending." << std::endl;
    return false;
  }
  else if (ret < sizeof(struct can_frame))
  {
    std::cout << "SocketCAN Interface: Error in sending, not all bytes sent." << std::endl;
    return false;
  }
  std::cout << "SocketCAN Interface: Send was successful." << std::endl;
  return true;
}

void Interface::sendQueued()
{
  for (auto &it : queue_outbound_)
  {
    send(&it);
  }
  queue_outbound_.erase(queue_outbound_.begin(), queue_outbound_.end());
}
void Interface::printCanFrame(const can_frame &frame)
{
  std::cout << "CAN ID:0x" << std::hex << (frame.can_id);
  std::cout << " Len:" << static_cast<int>(frame.can_dlc);
  std::cout << " Data:[";
  for (uint8_t i = 0; i < frame.can_dlc ; i++)
  {
    std::cout << " " << std::hex << static_cast<int>(frame.data[i]);
  }
  std::cout << " ]" << std::endl;
}

can_frame Interface::toCanFrame(const Frame &frame)
{
  can_frame ret_frame;
  ret_frame.can_id = frame.id;
  ret_frame.can_id = ret_frame.can_id | CAN_EFF_FLAG;
  ret_frame.can_dlc = frame.len;
  std::memcpy(ret_frame.data, frame.data.raw, frame.len);
  return ret_frame;
}

Frame Interface::fromCanFrame(const can_frame &frame)
{
  Frame ret_frame;
  ret_frame.id = frame.can_id & CAN_EFF_MASK;
  ret_frame.len = frame.can_dlc;
  std::memcpy(ret_frame.data.raw, frame.data, frame.can_dlc);
  return ret_frame;
}

}  // namespace grizzly_motor_driver
