#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <qchar.h>
#include <qspinbox.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <QApplication>
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>
#include <csignal>

#include <array>
#include <mutex>
#include <thread>

class BatteryNode : public rclcpp::Node {
  std::array<rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr,
             5 + 1>
      pubs_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr,
             5 + 1>
      pubs_pose;
  std::array<float, 5 + 1> percentage_;
  std::array<geometry_msgs::msg::PoseStamped, 5 + 1> pose_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;

public:
  BatteryNode() : Node("battery_slider_node") {
    for (int i = 1; i <= 5; i++) {
      pubs_[i] = this->create_publisher<sensor_msgs::msg::BatteryState>(
          "/cf_" + std::to_string(i) + "/battery_status", 10);
      pubs_pose[i] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/cf_" + std::to_string(i) + "/pose", 10);
      percentage_[i] = 100.0f;
      geometry_msgs::msg::PoseStamped pose;
      // pose.set__pose(const geometry_msgs::msg::Pose_<allocator<void>> &_arg)
      pose_[i] = pose;
    }

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), // 10 Hz
                                std::bind(&BatteryNode::publish_all, this));
  }

  void set_percentage(int idx, float value) {
    std::lock_guard<std::mutex> lock(mutex_);
    percentage_[idx] = value;
  }

  void set_x(int idx, double x) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_[idx].pose.position.x = x;
  }
  void set_y(int idx, double y) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_[idx].pose.position.y = y;
  }
  void set_z(int idx, double z) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_[idx].pose.position.z = z;
  }
  void set_pos(int idx, double x, double y, double z) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_x(idx,x);
    set_y(idx,y);
    set_z(idx,z);
  }

private:
  void publish_all() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (int i = 1; i <= 5; i++) {
      // Publishing BatteryState
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

      // Publishing Pose
      pubs_pose[i]->publish(pose_[i]);
    }
  }
};

class BatteryGUI : public QWidget {
public:
  BatteryGUI(std::shared_ptr<BatteryNode> node) : node_(node) {
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

      connect(slider, &QSlider::valueChanged, this,
              [this, i, value_label](int value) {
                node_->set_percentage(i, value);
                value_label->setText(QString("%1 %").arg(value));
              });

      auto *pose_label = new QLabel(QString("Pose %1").arg(i));
      pose_label->setAlignment(Qt::AlignCenter);
      QFormLayout *form = new QFormLayout();

      QDoubleSpinBox *xInput = new QDoubleSpinBox();
      QDoubleSpinBox *yInput = new QDoubleSpinBox();
      QDoubleSpinBox *zInput = new QDoubleSpinBox();

      xInput->setRange(-1000, 1000);
      yInput->setRange(-1000, 1000);
      zInput->setRange(-1000, 1000);

      form->addRow("Coordinate X:", xInput);
      form->addRow("Coordinate Y:", yInput);
      form->addRow("Coordinate Z:", zInput);

      connect(xInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,
              [this,i](double val) { node_->set_x(i, val); });

      connect(yInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,
              [this,i](double val) { node_->set_y(i, val); });


      connect(zInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,
              [this,i](double val) { node_->set_z(i, val); });

      col->addWidget(label);
      col->addWidget(slider);
      col->addWidget(value_label);
      col->addWidget(pose_label);
      layout->addLayout(col);
      layout->addLayout(form);
    }
  }

private:
  std::shared_ptr<BatteryNode> node_;
};

void sigint_handler(int) { QCoreApplication::quit(); }

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigint_handler);

  auto node = std::make_shared<BatteryNode>();

  std::thread ros_thread([&]() { rclcpp::spin(node); });

  QApplication app(argc, argv);
  BatteryGUI gui(node);
  gui.show();

  int ret = app.exec();

  rclcpp::shutdown();
  ros_thread.join();
  return ret;
}
