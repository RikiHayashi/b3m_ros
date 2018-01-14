#include <array>
#include <numeric>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace B3M {
enum MODE {
  Normal = 0x00,
  Free = 0x10,
  Hold = 0x11,
};
class B3M {
 public:
  B3M(std::string port, int baudrate = 115200) : serial_port_(io_srv_) {
    serial_port_.open(port);
    set_baudrate(baudrate);
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
  }

  void set_baudrate(int baudrate) {
    serial_port_.set_option(boost::asio::serial_port::baud_rate(baudrate));
  }

  void set_mode(unsigned char id, MODE mode) {
    std::array<unsigned char, 8> buffer;
    buffer[0] = static_cast<unsigned char>(0x08);
    buffer[1] = static_cast<unsigned char>(0x04);
    buffer[2] = static_cast<unsigned char>(0x00);
    buffer[3] = static_cast<unsigned char>(id);
    buffer[4] = static_cast<unsigned char>(mode);
    buffer[5] = static_cast<unsigned char>(0x28);
    buffer[6] = static_cast<unsigned char>(0x01);
    buffer[7] = static_cast<unsigned char>(
        std::accumulate(buffer.begin(), buffer.end() - 1, 0) & 0xFF);
    serial_port_.write_some(boost::asio::buffer(buffer, buffer.size()));
    boost::this_thread::sleep(boost::posix_time::microseconds(250));
    std::vector<unsigned char> readbuf(256);
    serial_port_.read_some(boost::asio::buffer(readbuf, readbuf.size()));
  }

  void set_angle(unsigned char id, double angle) {
    int pos = angle * 100;
    std::array<unsigned char, 9> buffer;
    buffer[0] = static_cast<unsigned char>(0x09);
    buffer[1] = static_cast<unsigned char>(0x04);
    buffer[2] = static_cast<unsigned char>(0x00);
    buffer[3] = static_cast<unsigned char>(id);
    buffer[4] = static_cast<unsigned char>(pos & 0xFF);
    buffer[5] = static_cast<unsigned char>((pos & 0xFF00) >> 8);
    buffer[6] = static_cast<unsigned char>(0x2A);
    buffer[7] = static_cast<unsigned char>(0x01);
    buffer[8] = static_cast<unsigned char>(
        std::accumulate(buffer.begin(), buffer.end() - 1, 0) & 0xFF);
    serial_port_.write_some(boost::asio::buffer(buffer, buffer.size()));
    boost::this_thread::sleep(boost::posix_time::microseconds(250));
    std::vector<unsigned char> readbuf(256);
    serial_port_.read_some(boost::asio::buffer(readbuf, readbuf.size()));
  }

  short get_angle(unsigned char id) {
    std::array<unsigned char, 7> buffer;
    buffer[0] = static_cast<unsigned char>(0x07);
    buffer[1] = static_cast<unsigned char>(0x03);
    buffer[2] = static_cast<unsigned char>(0x00);
    buffer[3] = static_cast<unsigned char>(id);
    buffer[4] = static_cast<unsigned char>(0x2C);
    buffer[5] = static_cast<unsigned char>(2);
    buffer[6] = static_cast<unsigned char>(
        std::accumulate(buffer.begin(), buffer.end() - 1, 0) & 0xFF);
    serial_port_.write_some(boost::asio::buffer(buffer, buffer.size()));
    boost::this_thread::sleep(boost::posix_time::microseconds(250));
    std::vector<unsigned char> readbuf(7);
    serial_port_.read_some(boost::asio::buffer(readbuf, readbuf.size()));
    return (readbuf[5] << 8) + readbuf[4];
  }

  void servo_sleep(int t) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(t));
  }

  void servo_usleep(int t) {
    boost::this_thread::sleep(boost::posix_time::microseconds(t));
  }

  void set_angle_wait_until(unsigned char id, double angle, double diff = 0.1) {
    set_angle(id, angle);
    servo_usleep(250);
    while (abs(angle - (get_angle(id) / 100.0)) > diff) {
      servo_usleep(250);
    }
  }

 private:
  boost::asio::io_service io_srv_;
  boost::asio::serial_port serial_port_;
};
}
