#pragma once

// A replacement for <iostream>.
// See also https://en.cppreference.com/w/cpp/header/iostream.

#include <mutex>

// These are required for us to be a work-alike for <iostream>.
#include <ios>
#include <istream>
#include <ostream>
#include <streambuf>

namespace drake {
namespace vendor_iostream {
namespace internal {

class ReplacementStream {
 public:
  ReplacementStream() = default;
  ~ReplacementStream() = default;

  std::mutex& GetMutex();
  std::ostream& GetOutput();
  void Finished();

  // To match std.
  ReplacementStream& flush();
  //    *this << "\n";
  //    return *this;
};

template <typename T>
ReplacementStream& operator<<(ReplacementStream& stream, T&& item) {
  std::lock_guard guard{stream.GetMutex()};
  stream.GetOutput() << std::forward<T>(item);
  stream.Finished();
  return stream;
}

ReplacementStream& operator<<(ReplacementStream& stream,
                              std::ostream& (*func)(std::ostream&));

extern ReplacementStream cout;
extern ReplacementStream cerr;

}  // namespace internal
}  // namespace vendor_iostream
}  // namespace drake

// This is undefined behavior. Deal with it.
namespace std {
using drake::vendor_iostream::internal::cerr;
using drake::vendor_iostream::internal::cout;
}  // namespace std
