#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <bitset>

class PS4UARTNode : public rclcpp::Node
{
public:
  PS4UARTNode() : Node("ps4_uart_node")
  {
    uart_fd_ = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY);
    if (uart_fd_ == -1) {
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
      "/joy", 10,
      std::bind(&PS4UARTNode::joy_callback, this, std::placeholders::_1)
    );

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PS4UARTNode::send_uart_data, this)
    );
  }

  ~PS4UARTNode() {
    if (uart_fd_ != -1) close(uart_fd_);
  }

private:
  int uart_fd_;
  sensor_msgs::msg::Joy::SharedPtr latest_joy_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    latest_joy_ = msg;
  }

  void send_uart_data()
  {
  if (!latest_joy_ || latest_joy_->buttons.size() < 4 || latest_joy_->axes.size() < 5)
    return;

  uint8_t button_data = 0;

  // 十字キー
  if (latest_joy_->axes[7] > 0.5)  button_data |= 1 << 7; // ↑
  if (latest_joy_->axes[7] < -0.5) button_data |= 1 << 6; // ↓
  if (latest_joy_->axes[6] > 0.5)  button_data |= 1 << 5; // ←
  if (latest_joy_->axes[6] < -0.5) button_data |= 1 << 4; // →

  // ○×△□（buttons）
  const int TRIANGLE = 3, CIRCLE = 1, CROSS = 0, SQUARE = 2;
  button_data |= (latest_joy_->buttons[TRIANGLE] ? 1 << 3 : 0);
  button_data |= (latest_joy_->buttons[CIRCLE]   ? 1 << 2 : 0);
  button_data |= (latest_joy_->buttons[CROSS]    ? 1 << 1 : 0);
  button_data |= (latest_joy_->buttons[SQUARE]   ? 1 << 0 : 0);

  // L2, R2（axes[3], axes[4]）を 0〜255 に変換
  float raw_l2 = latest_joy_->axes[3]; // -1.0 ～ +1.0
  float raw_r2 = latest_joy_->axes[4];
  uint8_t l2_val = static_cast<uint8_t>((-raw_l2 + 1.0f) * 127.5f);
  uint8_t r2_val = static_cast<uint8_t>((-raw_r2 + 1.0f) * 127.5f);

  // データ送信（3バイト）
  uint8_t buffer[4] = {0xAA, button_data, l2_val, r2_val};
  write(uart_fd_, buffer, 4);

  // ターミナル表示（確認用）
  RCLCPP_INFO(this->get_logger(), "Sent: buttons=%s, L2=%u, R2=%u",
              std::bitset<8>(button_data).to_string().c_str(),
              l2_val, r2_val);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PS4UARTNode>());
  rclcpp::shutdown();
  return 0;
}
