#ifndef RCLCPP__EXPERIMENTAL__SYNCHRONIZER_HPP_
#define RCLCPP__EXPERIMENTAL__SYNCHRONIZER_HPP_
#ifdef INTERNEURON
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rcl/wait.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rcl/types.h"
#include "rcl/time.h"

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/waitable.hpp"
#include <rmw/types.h>
#include "rclcpp/clock.hpp"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp
{
namespace experimental
{

template<typename CallbackT>
class Synchronizer : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(Synchronizer)
  RCLCPP_PUBLIC
  Synchronizer(
    CallbackT && callback,
    rclcpp::Context::SharedPtr context,
    size_t allowed_time_deviation_ms,
    std::vector<bool>trigger_channels,
    std::vector<uint64_t>sub_intra_ids,
    )
  : gc_(context), allowed_time_deviation_ms_(allowed_time_deviation_ms), trigger_channels_(trigger_channels), sub_intra_ids_(sub_intra_ids)
  {
    weak_ipm_ = context->get_sub_context<rclcpp::experimental::IntraProcessManager>();
  }

  RCLCPP_PUBLIC
  virtual ~Synchronizer();

  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override {return 1;}

  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  bool
  is_ready(rcl_wait_set_t * wait_set) override = 0;

  std::shared_ptr<void>
  take_data() override = 0;

  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override
  {
    (void)id;
    return take_data();
  }

  void
  execute(std::shared_ptr<void> & data) override = 0;

  virtual
  bool
  use_take_shared_method() const = 0;

  RCLCPP_PUBLIC
  const char *
  get_topic_names() const;

  RCLCPP_PUBLIC
  QoS
  get_actual_qos() const;

protected:
  std::recursive_mutex callback_mutex_;
  std::function<void(size_t)> on_new_message_callback_ {nullptr};
  size_t unread_count_{0};
  rclcpp::GuardCondition gc_;

  virtual void
  trigger_guard_condition() = 0;

private:
using IntraProcessManagerWeakPtr =
    std::weak_ptr<rclcpp::experimental::IntraProcessManager>;
  IntraProcessManagerWeakPtr weak_ipm_;
  std::vector<uint64_t>sub_intra_ids_;
  size_t allowed_time_deviation_ms_;
  std::vector<bool>trigger_channels_;
};

}  // namespace experimental
}  // namespace rclcpp
#endif
#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
