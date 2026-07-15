#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

class CSVLoggerNode : public rclcpp::Node
{
public:
    CSVLoggerNode()
        : Node("csv_logger_node")
    {
        const std::string output_directory = "power_logs";

        std::filesystem::create_directories(output_directory);

        const std::string filename =
            output_directory + "/" + make_timestamp() + ".csv";

        output_file_.open(filename);

        if (!output_file_.is_open())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open CSV file: %s",
                filename.c_str());
            return;
        }

        output_file_
            << "ros_time_sec,"
            << "sequence,"
            << "esp32_time_ms,"
            << "ina260_voltage_V,"
            << "ina260_current_A,"
            << "ina3221_ch0_voltage_V,"
            << "ina3221_ch0_current_A,"
            << "ina3221_ch1_voltage_V,"
            << "ina3221_ch1_current_A,"
            << "ina3221_ch2_voltage_V,"
            << "ina3221_ch2_current_A"
            << '\n';

        subscription_ =
            create_subscription<std_msgs::msg::String>(
                "/power_sensors",
                10,
                std::bind(
                    &CSVLoggerNode::power_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Logging power sensor data to: %s",
            filename.c_str());
    }

    ~CSVLoggerNode()
    {
        if (output_file_.is_open())
        {
            output_file_.close();
        }
    }

private:
    std::ofstream output_file_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    static std::string make_timestamp()
    {
        const auto now = std::chrono::system_clock::now();
        const std::time_t current_time =
            std::chrono::system_clock::to_time_t(now);

        std::tm local_time{};
        localtime_r(&current_time, &local_time);

        std::ostringstream stream;
        stream << std::put_time(
            &local_time,
            "%Y%m%d_%H%M%S");

        return stream.str();
    }

    void power_callback(
        const std_msgs::msg::String::SharedPtr message)
    {
        if (!output_file_.is_open())
        {
            return;
        }

        const std::string prefix = "PWR,";

        if (message->data.rfind(prefix, 0) != 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid power sensor message: %s",
                message->data.c_str());
            return;
        }

        const std::string sensor_data =
            message->data.substr(prefix.size());

        const double ros_time_sec =
            this->get_clock()->now().seconds();

        output_file_
            << std::fixed
            << std::setprecision(6)
            << ros_time_sec
            << ','
            << sensor_data
            << '\n';

        output_file_.flush();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CSVLoggerNode>());
    rclcpp::shutdown();

    return 0;
}