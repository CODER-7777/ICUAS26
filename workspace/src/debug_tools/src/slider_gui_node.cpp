#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <csignal>
#include <QCoreApplication>
#include <QApplication>
#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>

#include <thread>
#include <mutex>
#include <array>

class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode()
  : Node("battery_slider_node")
  {
    for (int i = 1; i <= 5; i++) {
      pubs_[i] = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "/cf_" + std::to_string(i)+"/battery_status", 10
      );
      percentage_[i] = 100.0f;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&BatteryNode::publish_all, this)
    );
  }

  void set_percentage(int idx, float value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    percentage_[idx] = value;
  }

private:
  void publish_all()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (int i = 1; i <= 5; i++) {
      sensor_msgs::msg::BatteryState msg;
      msg.header.stamp = this->now();
      msg.present = true;

      // Only percentage is meaningful
      msg.percentage = percentage_[i];

      // Everything else explicitly "unknown"
      msg.voltage = NAN;
      msg.current = NAN;
      msg.temperature = NAN;
      msg.charge = NAN;
      msg.capacity = NAN;
      msg.design_capacity = NAN;

      pubs_[i]->publish(msg);
    }
  }

  std::array<rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr, 5+1> pubs_;
  std::array<float, 5+1> percentage_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
};


class BatteryGUI : public QWidget
{
public:
  BatteryGUI(std::shared_ptr<BatteryNode> node)
  : node_(node)
  {
    setWindowTitle("Battery Percentage Sliders");

    auto *layout = new QHBoxLayout(this);

    for (int i = 1; i <= 5; i++) {
      auto *col = new QVBoxLayout();

      auto *label = new QLabel(QString("Battery %1").arg(i));
      label->setAlignment(Qt::AlignCenter);

      auto *slider = new QSlider(Qt::Vertical);
      slider->setRange(0, 100);
      slider->setValue(100);


      auto *value_label = new QLabel("100 %");
      value_label->setAlignment(Qt::AlignCenter);


      connect(slider, &QSlider::valueChanged,
              this, [this, i,value_label](int value) {
                node_->set_percentage(i, value);
                value_label->setText(QString("%1 %").arg(value));
              });

      col->addWidget(label);
      col->addWidget(slider);
      col->addWidget(value_label);
      layout->addLayout(col);
    }
  }

private:
  std::shared_ptr<BatteryNode> node_;
};

void sigint_handler(int)
{
  QCoreApplication::quit();
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigint_handler);

  auto node = std::make_shared<BatteryNode>();

  std::thread ros_thread([&]() {
    rclcpp::spin(node);
  });

  QApplication app(argc, argv);
  BatteryGUI gui(node);
  gui.show();

  int ret = app.exec();

  rclcpp::shutdown();
  ros_thread.join();
  return ret;
}

