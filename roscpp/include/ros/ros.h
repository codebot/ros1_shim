#ifndef ROS_H
#define ROS_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "ros/time.h"
#include "wall_timer.h"
#include <boost/bind.hpp>

/*
#define ROS_INFO(str, ...) printf(str "\n", ## __VA_ARGS__)
#define ROS_DEBUG(str, ...) printf(str "\n", ## __VA_ARGS__)
#define ROS_WARN(str, ...) printf(str "\n", ## __VA_ARGS__)
#define ROS_ERROR(str, ...) printf(str "\n", ## __VA_ARGS__)
#define ROS_FATAL(str, ...) printf(str "\n", ## __VA_ARGS__)
*/

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
  std::string topic_;

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

  std::string getTopic() const { return topic_; }

  uint32_t getNumSubscribers() const { return 42; } // TODO: not always true

  void shutdown() { } // todo: something

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
  void shutdown() { } // todo: something
};

class ServiceServer
{
public:
};

class ServiceClient
{
public:
  template <class Service>
  bool call(Service &service)
  {
    return true;
  }
};

class NodeHandle
{
public:
  std::string prefix;

  NodeHandle(const std::string &_prefix) : prefix(_prefix) { }
  NodeHandle() : prefix() { }

  bool ok() { return ros::ok(); }

  template <class T>
  void param(const std::string &name __attribute__((unused)), T &param, T default_value)
  {
    param = default_value;
  }

  template <class T>
  void getParamCached(const std::string &name __attribute((unused)), T &param __attribute__((unused)))
  {
  }

  template <class T>
  void getParam(const std::string &name __attribute((unused)), T &param __attribute__((unused)))
  {
  }
 
  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch __attribute__((unused)) = false )
  {
    typename rclcpp::publisher::Publisher<M>::SharedPtr pub;
    pub = Shim::get_shim()->node->create_publisher<M>(topic, (size_t)queue_size); // TODO: latching
    ROS2Publisher<M> *pub_templated = new ROS2Publisher<M>(pub);
    Publisher ros1_pub;
    ros1_pub.pub = static_cast<ROS2PublisherBase *>(pub_templated);
    ros1_pub.topic_ = topic;
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
    // callbacks take 'const std::shared_ptr<M const>' (no & at the end).
    auto shfp = [fp] (const std::shared_ptr<M const> msg) { fp(msg); };
    ros2_sub = Shim::get_shim()->node->create_subscription<M>(topic, shfp, rmw_qos_profile_default);
    ROS2Subscriber<M> *sub_templated = new ROS2Subscriber<M>(ros2_sub);
    Subscriber ros1_sub;
    ros1_sub.sub = static_cast<ROS2SubscriberBase *>(sub_templated);
    return ros1_sub;
  }

  template<class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void (const std::shared_ptr<M const>&)>& callback,
                             const VoidConstPtr& tracked_object = VoidConstPtr())
  {
    typename rclcpp::subscription::Subscription<M>::SharedPtr ros2_sub;
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = queue_size;
    // We need this lambda to work around the fact that (one common variant
    // of) ROS1 callbacks take 'const std::shared_ptr<M const>&' but ROS2
    // callbacks take 'const std::shared_ptr<M const>' (no & at the end).
    auto shfp = [callback] (const std::shared_ptr<M const> msg) { callback(msg); };
    ros2_sub = Shim::get_shim()->node->create_subscription<M>(topic, shfp, rmw_qos_profile_default);
    ROS2Subscriber<M> *sub_templated = new ROS2Subscriber<M>(ros2_sub);
    Subscriber ros1_sub;
    ros1_sub.sub = static_cast<ROS2SubscriberBase *>(sub_templated);
    return ros1_sub;

    //return Subscriber();
    /*
    SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, callback);
    ops.tracked_object = tracked_object;
    ops.transport_hints = transport_hints;
    return subscribe(ops);
    */
  }


  template <class MReq, class MRes>
  ServiceServer advertiseService(const std::string &service_name, bool (*fp)(MReq &, MRes &))
  {
    auto fp_wrapper =
        [fp] 
        (const std::shared_ptr<rmw_request_id_t> request_header, 
         const std::shared_ptr<MReq> request, std::shared_ptr<MRes> response) 
        {
          //fp(std::make_shared(request), std::make_shared(response)); 
        };
    Shim::get_shim()->node->create_service<typename MReq::_service>
        (service_name, fp_wrapper);
    return ServiceServer();
  }

  template <class Service>
  ServiceClient serviceClient(const std::string &service_name)
  {
    return ServiceClient();
  }

  template<class T>
  WallTimer createWallTimer(
      WallDuration period, void(T::*callback)(const WallTimerEvent&), T* obj,
      bool oneshot = false, bool autostart = true) const
  {
    return createWallTimer(period, boost::bind(callback, obj, _1), oneshot, autostart);
  }

  template<class T>
  WallTimer createWallTimer(
      WallDuration period, void(T::*callback)(const WallTimerEvent&),
      const boost::shared_ptr<T>& obj,
      bool oneshot = false, bool autostart = true) const
  {
    WallTimerOptions ops(period, boost::bind(callback, obj.get(), _1), 0);
    ops.tracked_object = obj;
    ops.oneshot = oneshot;
    ops.autostart = autostart;
    return createWallTimer(ops);
  }

  WallTimer createWallTimer(
      WallDuration period, const WallTimerCallback& callback, 
      bool oneshot = false, bool autostart = true) const;

  WallTimer createWallTimer(WallTimerOptions& ops) const;


  void setCallbackQueue(CallbackQueueInterface* queue);
};

class Rate
{
public:
  rclcpp::WallRate ros2_rate;

  Rate(double hz);
  bool sleep();
};

/*
class Time
{
public:
  static Time now();
  double toSec();
};
*/

}

#endif
