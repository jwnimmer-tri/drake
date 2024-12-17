#include "drake_cout_cerr.h"

#include <sstream>

namespace drake {
namespace vendor_iostream {
namespace internal {
namespace {

class Impl {
 public:
  std::mutex mutex_;
  std::ostringstream buffer_;

  static Impl& Singleton() {
    static Impl* result{new Impl};  // Leaked on purpose.
    return *result;
  }
};

}  // namespace

std::mutex& ReplacementStream::GetMutex() {
  return Impl::Singleton().mutex_;
}

std::ostream& ReplacementStream::GetOutput() {
  return Impl::Singleton().buffer_;
}

void ReplacementStream::Finished() {
  // XXX actually drake-log
}

ReplacementStream& operator<<(ReplacementStream& stream,
                              std::ostream& (*func)(std::ostream&)) {
  std::lock_guard guard{stream.GetMutex()};
  func(stream.GetOutput());
  stream.Finished();
  return stream;
}

ReplacementStream& ReplacementStream::flush() {
  *this << "\n";
  return *this;
}

ReplacementStream cout;
ReplacementStream cerr;

}  // namespace internal
}  // namespace vendor_iostream
}  // namespace drake
