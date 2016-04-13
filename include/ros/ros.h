#ifndef ROS_H
#define ROS_H

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#define ROS_INFO(str, ...) printf(str "\n", __VA_ARGS__)

namespace rclcpp
{
  namespace node
  {
    class Node;
  }
}

namespace ros
{

void init(int &argc, char **argv, const std::string &node_name, uint32_t options=0);
bool ok();
void spinOnce();
void spin();

class Shim
{
public:
  static Shim *get_shim();
  std::string node_name;
  std::shared_ptr<rclcpp::node::Node> node;
};

////////////////////////////////////////////////////////////////////////
class ROS2PublisherBase
{
public:
};

template <class M>
class ROS2Publisher : public ROS2PublisherBase
{
public:
  typename rclcpp::publisher::Publisher<M>::SharedPtr pub;
  ROS2Publisher(typename rclcpp::publisher::Publisher<M>::SharedPtr _pub) : pub(_pub) { }
};

class Publisher // coming from ros1
{
public:
  ROS2PublisherBase *pub;
  //std::shared_ptr<ROS2PublisherBase> pub;
  Publisher() : pub(NULL) { }

  template <typename M>
  void publish(const M& message) const
  {
    typename rclcpp::publisher::Publisher<M>::SharedPtr ros2pub;
    ROS2Publisher<M> *step1 = static_cast<ROS2Publisher<M> *>(pub);
    step1->pub->publish(message);

    //std::shared_ptr<ROS2Publisher<M>> step1 = static_cast<std::shared_ptr<ROS2Publisher<M>>>(pub);
    //ros2pub = static_cast<std::shared_ptr<ROS2Publisher<M>>>(pub)->pub;
  }

#if 0
  template <typename M>
  void publish(const boost::shared_ptr<M>& message) const;
#endif
};
////////////////////////////////////////////////////////////////////////

class ROS2SubscriberBase
{
public:
};

template <class M>
class ROS2Subscriber : public ROS2SubscriberBase
{
public:
  typename rclcpp::subscription::Subscription<M>::SharedPtr sub;
  ROS2Subscriber(typename rclcpp::subscription::Subscription<M>::SharedPtr _sub) : sub(_sub) { }
};

class Subscriber
{
public:
  ROS2SubscriberBase *sub;
  Subscriber() : sub(NULL) { }
};

class NodeHandle
{
public:
  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch __attribute__((unused)) = false )
  {
    typename rclcpp::publisher::Publisher<M>::SharedPtr pub;
    pub = Shim::get_shim()->node->create_publisher<M>(topic, (size_t)queue_size); // TODO: latching
    ROS2Publisher<M> *pub_templated = new ROS2Publisher<M>(pub);
    Publisher ros1_pub;
    ros1_pub.pub = static_cast<ROS2PublisherBase *>(pub_templated);
    return ros1_pub;
  }

  template <class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const std::shared_ptr<M const>&))
  {
    typename rclcpp::subscription::Subscription<M>::SharedPtr ros2_sub;
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = queue_size;
    // We need this lambda to work around the fact that (one common variant
    // of) ROS1 callbacks take 'const std::shared_ptr<M const>&' but ROS2
    // callbacks take 'const std::shared_ptr<M const>&' (no & at the end).
    auto shfp = [fp] (const std::shared_ptr<M const> msg) { fp(msg); };
    ros2_sub = Shim::get_shim()->node->create_subscription<M>(topic, shfp, rmw_qos_profile_default);
    ROS2Subscriber<M> *sub_templated = new ROS2Subscriber<M>(ros2_sub);
    Subscriber ros1_sub;
    ros1_sub.sub = static_cast<ROS2SubscriberBase *>(sub_templated);
    return ros1_sub;
  }
};

class Rate
{
public:
  rclcpp::WallRate ros2_rate;

  Rate(double hz);
  bool sleep();
};


}

#endif
