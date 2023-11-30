#include "drake/visualization/lcm_image_stream_worker.h"

#include <map>
#include <mutex>
#include <stdexcept>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "absl/container/btree_map.h"
#pragma GCC diagnostic pop

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_image_array.hpp"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/sensors/image_io.h"
#include "drake/systems/sensors/lcm_image_array_to_images.h"
#include "drake/visualization/colorize_depth_image.h"
#include "drake/visualization/colorize_label_image.h"
#include "drake/visualization/concatenate_images.h"

namespace drake {
namespace visualization {
namespace internal {
namespace {

using drake::lcm::DrakeLcm;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::sensors::ImageFileFormat;
using drake::systems::sensors::ImageIo;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::LcmImageArrayToImages;
using drake::visualization::ColorizeDepthImage;
using drake::visualization::ColorizeLabelImage;
using drake::visualization::ConcatenateImages;

using LatestPng = std::pair<std::shared_ptr<const std::vector<uint8_t>>, int>;

/* Subscribes to LCM image array messages on a given channel, converts the
images to a single colorized `*.png` file. */
class ChannelWorker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ChannelWorker);

  explicit ChannelWorker(const std::string& channel_name, DrakeLcm* lcm) {
    DRAKE_DEMAND(lcm != nullptr);
    DiagramBuilder<double> builder;
    subscriber_ = builder.AddSystem(
        LcmSubscriberSystem::Make<lcmt_image_array>(channel_name, lcm));
    auto* to_images = builder.AddSystem<LcmImageArrayToImages>();
    auto* depth_to_color = builder.AddSystem<ColorizeDepthImage<double>>();
    auto* label_to_color = builder.AddSystem<ColorizeLabelImage<double>>();
    auto* concatenate = builder.AddSystem<ConcatenateImages<double>>(1, 3);
    builder.Connect(*subscriber_, *to_images);
    builder.Connect(to_images->color_image_output_port(),
                    concatenate->get_input_port(0, 0));
    builder.Connect(to_images->depth_image_output_port(),
                    depth_to_color->GetInputPort("depth_image_32f"));
    builder.Connect(depth_to_color->get_output_port(),
                    concatenate->get_input_port(0, 1));
    builder.Connect(to_images->label_image_output_port(),
                    label_to_color->get_input_port());
    builder.Connect(label_to_color->get_output_port(),
                    concatenate->get_input_port(0, 2));
    builder.ExportOutput(concatenate->get_output_port());
    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
    thread_ = std::thread(&ChannelWorker::ThreadMain, this);
  }

  ~ChannelWorker() {
    Shutdown();
    thread_.join();
  }

  void Shutdown() { thread_shutdown_ = true; }

  LatestPng GetLatestPng() const {
    std::lock_guard<std::mutex> lock(latest_png_mutex_);
    return latest_png_;
  }

 private:
  void ThreadMain() {
    try {
      while (!thread_shutdown_) {
        ThreadWork();
      }
    } catch (const std::exception& e) {
      drake::log()->critical(
          "ImageStreamWorker's channel thread crashed via an exception: {}",
          e.what());
    }
  }

  void ThreadWork() {
    // Check if the DrakeLcmSubscriber has any pending messages.
    const auto& subscriber_context =
        subscriber_->GetMyContextFromRoot(*context_);
    const int old_count = subscriber_->GetMessageCount(subscriber_context);
    const int wait_count = subscriber_->WaitForMessage(
        old_count, /* message = */ nullptr, /* timeout = */ 0.01);
    if (wait_count == old_count) {
      // No messages.
      return;
    }

    // Copy the new message into the Context.
    diagram_->ExecuteInitializationEvents(context_.get());

    // Calculate the full colorized and tiled image.
    const auto& image =
        diagram_->get_output_port().template Eval<ImageRgba8U>(*context_);
    std::vector<uint8_t> png_data =
        ImageIo{}.Save(image, ImageFileFormat::kPng);

    // Box up the new LatestPng.
    const int new_count = subscriber_->GetMessageCount(subscriber_context);
    auto new_png = std::make_shared<std::vector<uint8_t>>(std::move(png_data));
    LatestPng new_pair{std::move(new_png), new_count};

    // Move the data into the mailbox.
    std::lock_guard<std::mutex> lock(latest_png_mutex_);
    latest_png_ = std::move(new_pair);
  }

  // The `latest_png_mutex_` guards `latest_png_`.
  mutable std::mutex latest_png_mutex_;
  LatestPng latest_png_;

  // The `thread_` has exclusive access the `diagram_` and `context_` objects;
  // no other threads are allowed to touch them (other than in our constructor
  // during set-up). The `subscriber_` pointer is an alias into `diagram_`.
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  LcmSubscriberSystem* subscriber_{};

  // Our destructor sets `thread_shutdown_` to ask `thread_` to exit.
  std::atomic_bool thread_shutdown_;
  std::thread thread_;
};

}  // namespace

class LcmImageStreamWorker::Impl final {
 public:
  Impl(std::string lcm_url, std::string_view channel_regex)
      : lcm_(std::move(lcm_url)) {
    lcm_.SubscribeMultichannel(
        channel_regex, [this](std::string_view channel_name,
                              const void* /* buffer */, int /* size */) {
          this->HandleChannelName(channel_name);
        });
    lcm_thread_ = std::thread(&Impl::LcmThreadMain, this);
  }

  ~Impl() {
    // Manually shutting down the workers is not strictly required, but might
    // improve the responsiveness of this destructor.
    for (const auto& [_, worker] : workers_) {
      worker->Shutdown();
    }
    lcm_thread_shutdown_ = true;
    lcm_thread_.join();
  }

  std::vector<std::string> GetChannelNames() const {
    std::lock_guard<std::mutex> lock(workers_mutex_);
    std::vector<std::string> result;
    result.reserve(workers_.size());
    for (const auto& [channel_name, _] : workers_) {
      result.push_back(channel_name);
    }
    return result;
  }

  LatestPng GetLatestPng(std::string_view channel_name) const {
    const ChannelWorker* worker = [&]() {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      auto iter = workers_.find(channel_name);
      return (iter != workers_.end()) ? iter->second.get() : nullptr;
    }();
    return (worker != nullptr) ? worker->GetLatestPng() : LatestPng{};
  }

 private:
  void LcmThreadMain() {
    try {
      while (!lcm_thread_shutdown_) {
        // This copies any lcmt_image_array message(s) from DrakeLcm's internal
        // buffer into the various DrakeLcmSubscribers' member fields (in our
        // ChannelWorker objects). Inside of DrakeLcmSubscriber, there is a
        // already a mutex -- i.e., LCM messages are allowed to come from
        // threads other than the thread that owns the Context.
        lcm_.HandleSubscriptions(/* timeout_millis = */ 10);
      }
    } catch (const std::exception& e) {
      drake::log()->critical(
          "ImageStreamWorker's LCM thread crashed via an exception: {}",
          e.what());
    }
  }

  // Called only on the `lcm_thread_` thread.
  void HandleChannelName(std::string_view channel_name) {
    std::lock_guard<std::mutex> lock(workers_mutex_);
    auto iter = workers_.find(channel_name);
    if (iter == workers_.end()) {
      std::string name{channel_name};
      workers_.emplace(name, std::make_unique<ChannelWorker>(name, &lcm_));
    }
  }

  // The `lcm_thread_` has exclusive access the `lcm_` object; no other threads
  // are allowed to touch it (other than in our constructor during set-up). Our
  // destructor sets `lcm_thread_shutdown_` to ask `lcm_thread_` to exit.
  DrakeLcm lcm_;
  std::atomic_bool lcm_thread_shutdown_;
  std::thread lcm_thread_;

  // The `workers_mutex_` guards the `workers_` map but _not_ the calling of any
  // member functions on the ChannelWorker object itself (i.e., it is shallow).
  mutable std::mutex workers_mutex_;
  absl::btree_map<std::string, std::unique_ptr<ChannelWorker>> workers_;
};

LcmImageStreamWorker::LcmImageStreamWorker(std::string lcm_url,
                                           std::string_view channel_regex)
    : impl_(std::make_unique<Impl>(std::move(lcm_url), channel_regex)) {}

LcmImageStreamWorker::~LcmImageStreamWorker() = default;

std::vector<std::string> LcmImageStreamWorker::GetChannelNames() const {
  return impl_->GetChannelNames();
}

std::pair<std::shared_ptr<const std::vector<uint8_t>>, int>
LcmImageStreamWorker::GetLatestPng(std::string_view channel_name) const {
  return impl_->GetLatestPng(channel_name);
}

}  // namespace internal
}  // namespace visualization
}  // namespace drake
