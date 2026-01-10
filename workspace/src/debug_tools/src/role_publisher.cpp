#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QTimer>

#include <map>
#include <memory>
#include <string>

/* ================= ENUM ================= */

enum class Role : int
{
  UNASSIGNED = 0,
  CHARGE     = 1,
  SCOUT      = 2,
  LAND       = 3,
  FOLLOW     = 4,
  CENTER     = 5
};

static const std::map<QString, Role> ROLE_MAP = {
  {"UNASSIGNED", Role::UNASSIGNED},
  {"CHARGE",     Role::CHARGE},
  {"SCOUT",      Role::SCOUT},
  {"LAND",       Role::LAND},
  {"FOLLOW",     Role::FOLLOW},
  {"CENTER",     Role::CENTER}
};

/* ================= ROS NODE ================= */

class RolePublisherNode : public rclcpp::Node
{
public:
  RolePublisherNode() : Node("role_panel_node")
  {
    for (int i = 1; i <= 5; ++i)
    {
      std::string topic = "/cf_" + std::to_string(i) + "/role";
      pubs_[i] = create_publisher<std_msgs::msg::Int32>(topic, 10);
      roles_[i] = Role::UNASSIGNED;   // default
    }
  }

  void set_role(int drone_id, Role role)
  {
    roles_[drone_id] = role;
  }

  void publish_all()
  {
    for (const auto &[id, pub] : pubs_)
    {
      std_msgs::msg::Int32 msg;
      msg.data = static_cast<int>(roles_[id]);
      pub->publish(msg);
    }
  }

private:
  std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> pubs_;
  std::map<int, Role> roles_;
};

/* ================= QT PANEL ================= */

class RolePanel : public QWidget
{
public:
  explicit RolePanel(std::shared_ptr<RolePublisherNode> node)
  : node_(node)
  {
    auto *main = new QVBoxLayout(this);

    for (int i = 1; i <= 5; ++i)
    {
      auto *row   = new QHBoxLayout();
      auto *label = new QLabel(QString("cf_%1").arg(i));
      auto *combo = new QComboBox();

      for (const auto &kv : ROLE_MAP)
        combo->addItem(kv.first);

      combo->setCurrentText("UNASSIGNED");

      connect(combo, &QComboBox::currentTextChanged,
              this, [this, i](const QString &text)
              {
                node_->set_role(i, ROLE_MAP.at(text));
              });

      row->addWidget(label);
      row->addWidget(combo);
      main->addLayout(row);
    }

    setLayout(main);
    setWindowTitle("Crazyflie Role Panel");
    resize(320, 200);

    /* ---- fixed publish timer (10 Hz) ---- */
    publish_timer_ = new QTimer(this);
    connect(publish_timer_, &QTimer::timeout,
            this, [this]() { node_->publish_all(); });
    publish_timer_->start(100);   // ms → 10 Hz
  }

private:
  std::shared_ptr<RolePublisherNode> node_;
  QTimer *publish_timer_;
};

/* ================= MAIN ================= */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<RolePublisherNode>();

  RolePanel panel(node);
  panel.show();

  QTimer ros_spin_timer;
  QObject::connect(&ros_spin_timer, &QTimer::timeout,
                   [&]() { rclcpp::spin_some(node); });
  ros_spin_timer.start(10);

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}

