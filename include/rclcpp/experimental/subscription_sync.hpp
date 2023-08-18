#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_SYNC_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_SYNC_HPP_

#include <rmw/types.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "rcl/types.h"
#include "rcl/time.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{

template<
  typename MessageT,
  typename SubscribedType,
  typename SubscribedTypeAlloc = std::allocator<SubscribedType>,
  typename SubscribedTypeDeleter = std::default_delete<SubscribedType>,
  typename ROSMessageType = SubscribedType,
  typename Alloc = std::allocator<void>
>
class SubscriptionSync
  : public SubscriptionIntraProcessBuffer<
    SubscribedType,
    SubscribedTypeAlloc,
    SubscribedTypeDeleter,
    ROSMessageType
  >
{
  using SubscriptionIntraProcessBufferT = SubscriptionIntraProcessBuffer<
    SubscribedType,
    SubscribedTypeAlloc,
    SubscribedTypeDeleter,
    ROSMessageType
  >;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionSync)

  using MessageAllocTraits =
    typename SubscriptionIntraProcessBufferT::SubscribedTypeAllocatorTraits;
  using MessageAlloc = typename SubscriptionIntraProcessBufferT::SubscribedTypeAllocator;
  using ConstMessageSharedPtr = typename SubscriptionIntraProcessBufferT::ConstDataSharedPtr;
  using MessageUniquePtr = typename SubscriptionIntraProcessBufferT::SubscribedTypeUniquePtr;
  using BufferUniquePtr = typename SubscriptionIntraProcessBufferT::BufferUniquePtr;

  rclcpp::Clock ros_clock;

  SubscriptionSync(
    AnySubscriptionCallback<MessageT, Alloc> callback,
    std::shared_ptr<Alloc> allocator,
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile,
    rclcpp::IntraProcessBufferType buffer_type)
  : SubscriptionIntraProcessBuffer<SubscribedType, SubscribedTypeAlloc,
      SubscribedTypeDeleter, ROSMessageType>(
      std::make_shared<SubscribedTypeAlloc>(*allocator),
      context,
      topic_name,
      qos_profile,
      buffer_type),
    any_callback_(callback)
  {
 
    TRACEPOINT(
      rclcpp_subscription_callback_added,
      static_cast<const void *>(this),
      static_cast<const void *>(&any_callback_));
    // The callback object gets copied, so if registration is done too early/before this point
    // (e.g. in `AnySubscriptionCallback::set()`), its address won't match any address used later
    // in subsequent tracepoints.
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  virtual ~SubscriptionSync() = default;

  
  std::shared_ptr<void>
  take_data() override
  {
    ConstMessageSharedPtr shared_msg;
    MessageUniquePtr unique_msg;
    std::unique_ptr<rclcpp::MessageInfo> message_info;

    if (any_callback_.use_take_shared_method()) {
      std::tie(shared_msg, message_info) = this->buffer_->consume_shared_with_message_info();
      if (!shared_msg || !message_info) {
        return nullptr;
      }
    } else {
      std::tie(unique_msg, message_info) = this->buffer_->consume_unique_with_message_info();
      if (!unique_msg || !message_info) {
        return nullptr;
      }
    }
    return std::static_pointer_cast<void>(
      std::make_shared<std::tuple<ConstMessageSharedPtr, MessageUniquePtr, std::unique_ptr<rclcpp::MessageInfo>>>(
        std::tuple<ConstMessageSharedPtr, MessageUniquePtr,std::unique_ptr<rclcpp::MessageInfo>>(
          shared_msg, std::move(unique_msg),
        std::move(message_info)))
    );
  }
  
  void execute(std::shared_ptr<void> & data) override
  {
    execute_impl<SubscribedType>(data);
  }

protected:
  template<typename T>
  typename std::enable_if<std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(std::shared_ptr<void> & data)
  {
    (void)data;
    throw std::runtime_error("Subscription intra-process can't handle serialized messages");
  }

  template<class T>
  typename std::enable_if<!std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(std::shared_ptr<void> & data)
  {
    if (!data) {
      return;
    }


    auto shared_ptr = std::static_pointer_cast<std::tuple<ConstMessageSharedPtr, MessageUniquePtr,std::unique_ptr<rclcpp::MessageInfo>>>(
      data);

    auto msg_info = *(std::get<2>(*shared_ptr));
    //msg_info.get_rmw_message_info().received_timestamp = static_cast<int64_t>(ros_clock.now().nanoseconds());
    if (any_callback_.use_take_shared_method()) {
      ConstMessageSharedPtr shared_msg = std::get<0>(*shared_ptr);
      any_callback_.dispatch_intra_process(shared_msg, msg_info);
    } else {
      MessageUniquePtr unique_msg = std::move(std::get<1>(*shared_ptr));
      any_callback_.dispatch_intra_process(std::move(unique_msg), msg_info);
    }
    shared_ptr.reset();
  }
  
  AnySubscriptionCallback<MessageT, Alloc> any_callback_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif 
