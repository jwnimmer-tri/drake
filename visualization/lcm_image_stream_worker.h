#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace visualization {
namespace internal {

/* Subscribes to LCM image array messages and converts each message to a tiled
and colorized `*.png` file. Uses worker threads for performance. */
class LcmImageStreamWorker final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmImageStreamWorker);

  /* Subscribes to lcmt_image_array messages on channels matching the regex. */
  LcmImageStreamWorker(std::string lcm_url, std::string_view channel_regex);

  ~LcmImageStreamWorker();

  /* Returns the list of channel names seen so far. */
  std::vector<std::string> GetChannelNames() const;

  /* Returns the latest PNG file contents for the given channel name, along with
  a sequence number. The sequence number will increase any time the PNG file
  changes (and not necessarily just by +1; there are allowed to be gaps). */
  std::pair<std::shared_ptr<const std::vector<uint8_t>>, int> GetLatestPng(
      std::string_view channel_name) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace internal
}  // namespace visualization
}  // namespace drake
