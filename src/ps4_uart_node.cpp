#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <bitset>
#include <string>

class PS4UARTNode : public rclcpp::Node
{
public:
  PS4UARTNode()
      : Node("ps4_uart_node"),
        uart_fd_(-1)
  {
    uart_fd_ = open(
        "/dev/ttyTHS1",
        O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (uart_fd_ == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "UART open failed");
      return;
    }

    struct termios tty;
    tcgetattr(uart_fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(uart_fd_, TCSANOW, &tty);

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        10,
        std::bind(
            &PS4UARTNode::joy_callback,
            this,
            std::placeholders::_1));

    timer_ = create_wall_timer( // PS4データ送信用タイマー
        std::chrono::milliseconds(50),
        std::bind(
            &PS4UARTNode::send_uart_data,
            this));
    // ESP32データ受信用タイマー
    receive_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PS4UARTNode::receive_uart_data, this));
  }

  ~PS4UARTNode()
  {
    if (uart_fd_ != -1)
    {
      close(uart_fd_);
    }
  }

private:
  int uart_fd_;
  sensor_msgs::msg::Joy::SharedPtr latest_joy_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::TimerBase::SharedPtr receive_timer_; // 受信データを定期的にチェックするタイマー
  std::string receive_buffer_;                 // 受信データを格納するバッファ

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    latest_joy_ = msg;
  }

  void send_uart_data()
  {
    if (
        uart_fd_ == -1 ||
        !latest_joy_ ||
        latest_joy_->buttons.size() < 4 ||
        latest_joy_->axes.size() < 8)
    {
      return;
    }

    uint8_t button_data = 0;

    // 十字キー
    if (latest_joy_->axes[7] > 0.5)
    {
      button_data |= 1 << 7; // 上
    }

    if (latest_joy_->axes[7] < -0.5)
    {
      button_data |= 1 << 6; // 下
    }

    if (latest_joy_->axes[6] > 0.5)
    {
      button_data |= 1 << 5; // 左
    }

    if (latest_joy_->axes[6] < -0.5)
    {
      button_data |= 1 << 4; // 右
    }

    // 三角、丸、バツ、四角
    const int TRIANGLE = 3;
    const int CIRCLE = 1;
    const int CROSS = 0;
    const int SQUARE = 2;

    button_data |=
        latest_joy_->buttons[TRIANGLE] ? (1 << 3) : 0;

    button_data |=
        latest_joy_->buttons[CIRCLE] ? (1 << 2) : 0;

    button_data |=
        latest_joy_->buttons[CROSS] ? (1 << 1) : 0;

    button_data |=
        latest_joy_->buttons[SQUARE] ? (1 << 0) : 0;

    // L2、R2を0～255に変換
    const float raw_l2 = latest_joy_->axes[3];
    const float raw_r2 = latest_joy_->axes[4];

    const uint8_t l2_val = static_cast<uint8_t>(
        (-raw_l2 + 1.0f) * 127.5f);

    const uint8_t r2_val = static_cast<uint8_t>(
        (-raw_r2 + 1.0f) * 127.5f);

    // JetsonからESP32へ4バイト送信
    const uint8_t buffer[4] = {
        0xAA,
        button_data,
        l2_val,
        r2_val};

    const ssize_t written_size = write(
        uart_fd_,
        buffer,
        sizeof(buffer));

    if (written_size != static_cast<ssize_t>(sizeof(buffer)))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "UART write incomplete: %ld bytes",
          static_cast<long>(written_size));
      return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Sent: buttons=%s, L2=%u, R2=%u",
        std::bitset<8>(button_data).to_string().c_str(),
        static_cast<unsigned int>(l2_val),
        static_cast<unsigned int>(r2_val));
  }
  void receive_uart_data()
  {
    if (uart_fd_ == -1)
    {
      return;
    }

    char buffer[256];

    const ssize_t received_size = read(
        uart_fd_,
        buffer,
        sizeof(buffer));

    if (received_size <= 0)
    {
      return;
    }

    receive_buffer_.append(
        buffer,
        static_cast<std::size_t>(received_size));

    std::size_t newline_position;

    while (
        (newline_position = receive_buffer_.find('\n')) !=
        std::string::npos)
    {
      std::string line = receive_buffer_.substr(
          0,
          newline_position);

      receive_buffer_.erase(
          0,
          newline_position + 1);

      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }

      if (line.rfind("PWR,", 0) == 0)
      {
        RCLCPP_INFO(
            this->get_logger(),
            "Received: %s",
            line.c_str());
      }
    }

    if (receive_buffer_.size() > 1024)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "UART receive buffer overflow. Buffer cleared.");

      receive_buffer_.clear();
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PS4UARTNode>());
  rclcpp::shutdown();

  return 0;
}