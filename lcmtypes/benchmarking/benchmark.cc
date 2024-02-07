#include <benchmark/benchmark.h>

#include "drake/lcmt_image_array.hpp"
#include "drake/tools/performance/fixture_common.h"

/* A collection of LCM encoding / decoding scenarios. */

namespace drake {
namespace {

class LcmFixture : public benchmark::Fixture {
 public:
  LcmFixture() {
    tools::performance::AddMinMaxStatistics(this);
    this->Unit(benchmark::kMicrosecond);
  }

  static lcmt_image MakeImage(std::string frame_name, int bytes_per_pixel) {
    lcmt_image message{};
    message.header.frame_name = frame_name, message.width = 848;
    message.height = 480;
    message.data.resize(message.width * message.height * bytes_per_pixel);
    message.size = message.data.size();
    return message;
  }

  static lcmt_image_array MakeImageArray() {
    lcmt_image_array message{};
    message.header.seq = 1;
    message.header.utime = 2;
    message.header.frame_name = "frame";
    message.images.push_back(MakeImage("color", 4));
    message.images.push_back(MakeImage("depth", 2));
    message.num_images = message.images.size();
    return message;
  }
};

template <typename Message>
__attribute__((noinline)) bool Encode(const Message& message,
                                      std::vector<uint8_t>* bytes) {
  const int64_t num_bytes = message.getEncodedSize();
  bytes->resize(num_bytes);
  message.encode(bytes->data(), 0, num_bytes);
  return true;
}

template <typename Message>
__attribute__((noinline)) bool Decode(const std::vector<uint8_t>& bytes,
                                      Message* message) {
  message->decode(bytes.data(), 0, bytes.size());
  return true;
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(LcmFixture, ImageArrayEncode)(benchmark::State& state) {
  const auto message = MakeImageArray();
  for (auto _ : state) {
    std::vector<uint8_t> bytes;
    benchmark::DoNotOptimize(Encode(message, &bytes));
  }
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(LcmFixture, ImageArrayDecode)(benchmark::State& state) {
  std::vector<uint8_t> bytes;
  Encode(MakeImageArray(), &bytes);
  for (auto _ : state) {
    lcmt_image_array message{};
    benchmark::DoNotOptimize(Decode(bytes, &message));
  }
}

}  // namespace
}  // namespace drake
