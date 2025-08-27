#pragma once

#include <mutex>
#include <ostream>
#include <utility>

namespace drake {
namespace vendor_iostream {
namespace internal {

// A class that acts somewhat like a std::ostream.
class ReplacementStream {
 public:
  ReplacementStream() = default;
  ~ReplacementStream() = default;

  // This function mimics the same function on std::ostream.
  ReplacementStream& flush();

  // Helpers for used by the operator<< function template, below.
  // The mutex must already be held when calling GetOutput() or Finished().
  std::mutex& GetMutex();
  std::ostream& GetOutput();
  void Finished();
};

// This overload is used to format a streamable `item` into the stream.
template <typename T>
ReplacementStream& operator<<(ReplacementStream& stream, T&& item) {
  std::lock_guard guard{stream.GetMutex()};
  stream.GetOutput() << std::forward<T>(item);
  stream.Finished();
  return stream;
}

// This overload is used for stream configuration with <iomanip>.
ReplacementStream& operator<<(ReplacementStream& stream,
                              std::ostream& (*func)(std::ostream&));

// A stream that logs each line to drake::log()->info().
extern ReplacementStream cout;

// A stream that logs each line to drake::log()->warning().
extern ReplacementStream cerr;

}  // namespace internal
}  // namespace vendor_iostream
}  // namespace drake
