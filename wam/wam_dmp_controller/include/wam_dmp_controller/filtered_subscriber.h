#ifndef ROSRT_FILTERED_SUBSCRIBER_H
#define ROSRT_FILTERED_SUBSCRIBER_H

#include <boost/function.hpp>

#include <lockfree/object_pool.h>
#include "pool_gc.h"

#include <ros/atomic.h>
#include <ros/ros.h>
#include <rosrt/subscriber.h>

namespace rosrt
{

/**
 * \brief A lock-free, filtered subscriber.  Allows you to receive ROS messages inside a realtime thread.
 *
 * This subscriber will pass the message through a user-defined "filter", which will convert the message
 * into another user-defined type. The filter function will be called outside of realtime.
 *
 * This subscriber works in a polling manner rather than the usual callback-based mechanism, e.g.:
\verbatim
bool filter(const MsgConstPtr& msg, const FilteredPtr filtered)
{
   // filtered->data = msg->data;
}
FilteredSubscriber<Msg,Filtered> sub(2, nh, "my_topic", filter);
while (true)
{
  FilteredConstPtr filtered_msg = sub.poll();
  if (filtered_msg)
  {
    // do something with filtered_msg
    ...
  }
}
\endverbatim
 */
template<typename M, typename Filtered>
class FilteredSubscriber
{
public:
  /**
   * \brief Default constructor.  You must call initialize() before doing anything else if you use this constructor.
   */
  FilteredSubscriber()
  : filtered_pool_(0)
  {
  }

  /**
   * \brief Constructor with initialization.  Call subscribe() to subscribe to a topic.
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   */
  FilteredSubscriber(uint32_t message_pool_size)
  : filtered_pool_(0)
  {
    initialize(message_pool_size);
  }

  /**
   * \brief Constructor with initialization and subscription
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   * \param nh The ros::NodeHandle to use to subscribe
   * \param topic The topic to subscribe on
   * \param filter The filter function to convert the message into the type Filtered
   * \param [optional] transport_hints the transport hints to use
   */
  FilteredSubscriber(uint32_t message_pool_size, ros::NodeHandle& nh, const std::string& topic,
                     boost::function<bool (const boost::shared_ptr<M const>& msg, const boost::shared_ptr<Filtered> filtered)> filter,
                     const ros::TransportHints& transport_hints = ros::TransportHints())
  : filtered_pool_(0)
  {
    initialize(message_pool_size);
    subscribe(nh, topic, filter, transport_hints);
  }

  ~FilteredSubscriber()
  {
    Filtered const* latest = latest_.exchange(0);
    if (latest)
    {
      filtered_pool_->free(latest);
    }

    detail::addPoolToGC((void*)filtered_pool_, detail::deletePool<Filtered>, detail::poolIsDeletable<Filtered>);
  }

  /**
   * \brief Initialize this subscribe.  Only use with the default constructor.
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   */
  void initialize(uint32_t message_pool_size)
  {
    ROS_ASSERT(message_pool_size > 1);
    ROS_ASSERT(!filtered_pool_);
    filtered_pool_ = new lockfree::ObjectPool<Filtered>();
    filtered_pool_->initialize(message_pool_size, Filtered());
    latest_.store(0);
  }

  /**
   * \brief Initialize this subscribe.  Only use with the default constructor.
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   * \param nh The ros::NodeHandle to use to subscribe
   * \param topic The topic to subscribe on
   * \param filter The filter function to convert the message into the type Filtered
   * \param [optional] transport_hints the transport hints to use
   * \return Whether or not we successfully subscribed
   */
  bool initialize(uint32_t message_pool_size, ros::NodeHandle& nh, const std::string& topic,
                  boost::function<bool (const boost::shared_ptr<M const>& msg, const boost::shared_ptr<Filtered> filtered)> filter,
                  const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    initialize(message_pool_size);
    return subscribe(nh, topic, filter, transport_hints);
  }

  /**
   * \brief Subscribe to a topic
   * \param nh The ros::NodeHandle to use to subscribe
   * \param topic The topic to subscribe on
   * \param filter The filter function to convert the message into the type Filtered
   * \param [optional] transport_hints the transport hints to use
   * \return Whether or not we successfully subscribed
   */
  bool subscribe(ros::NodeHandle& nh, const std::string& topic,
                 boost::function<bool (const boost::shared_ptr<M const>& msg, const boost::shared_ptr<Filtered> filtered)> filter,
                 const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, 1, boost::bind(&FilteredSubscriber::callback, this, _1));
    ops.callback_queue = detail::getSubscriberCallbackQueue();
    sub_ = nh.subscribe(ops);
    filter_ = filter;
    return (bool)sub_;
  }

  /**
   * \brief Retrieve the newest message received.
   *
   * The same message will only be returned once, i.e:
\verbatim
<msg received>
msg = poll(); // Returns a valid message
msg = poll(); // Returns NULL
\endverbatim
   */
  boost::shared_ptr<Filtered> poll()
  {
    Filtered* latest = latest_.exchange(0);
    if (!latest)
    {
      return boost::shared_ptr<Filtered>();
    }

    boost::shared_ptr<Filtered> ptr = filtered_pool_->makeShared(latest);
    if (!ptr)
    {
      filtered_pool_->free(latest);
      return boost::shared_ptr<Filtered>();
    }

    return ptr;
  }

private:
  void callback(const boost::shared_ptr<M const>& msg)
  {
    boost::shared_ptr<Filtered> filtered = filtered_pool_->allocateShared();
    if (!filtered)
    {
      ROS_ERROR("FilteredSubscriber: could not allocate filtered object.");
      return;
    }
    if (!filter_(msg, filtered))
    {
      ROS_ERROR("FilteredSubscriber: filter function failed.");
      return;
    }

    Filtered* latest = filtered_pool_->removeShared(filtered);
    Filtered* old = latest_.exchange(latest);
    if (old)
    {
      filtered_pool_->free(old);
    }
  }

  ros::atomic<Filtered*> latest_;

  lockfree::ObjectPool<Filtered>* filtered_pool_;
  ros::Subscriber sub_;
  boost::function<bool (const boost::shared_ptr<M const>& msg, const boost::shared_ptr<Filtered> filtered)> filter_;
};

} // namespace rosrt

#endif // ROSRT_FILTERED_SUBSCRIBER_H