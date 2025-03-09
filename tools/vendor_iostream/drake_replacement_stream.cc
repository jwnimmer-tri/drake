#include "drake_replacement_stream.h"

#include <sstream>
#include <string>

#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace vendor_iostream {
namespace internal {
namespace {

class Impl {
 public:
  std::mutex mutex_;
  std::ostringstream buffer_;

  static Impl& Singleton() {
    static never_destroyed<Impl> result;
    return result.access();
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
  auto& impl = Impl::Singleton();
  const std::string_view buffer_view = impl.buffer_.view();
  const size_t offset = buffer_view.find('\n');
  if (offset == std::string_view::npos) {
    return;
  }
  const std::string_view one_line = buffer_view.substr(0, offset);
  if (this == &cerr) {
    log()->warn("[cerr] {}", one_line);
  } else {
    log()->info("[cout] {}", one_line);
  }
  std::string content = std::move(impl.buffer_).str();
  content.erase(0, offset + 1);
  impl.buffer_.str(std::move(content));
}

ReplacementStream& operator<<(ReplacementStream& stream,
                              std::ostream& (*func)(std::ostream&)) {
  std::lock_guard guard{stream.GetMutex()};
  func(stream.GetOutput());
  stream.Finished();
  return stream;
}

ReplacementStream& ReplacementStream::flush() {
  // Note that the operator<< here will take the mutex.
  *this << "\n";
  return *this;
}

ReplacementStream cout;
ReplacementStream cerr;

}  // namespace internal
}  // namespace vendor_iostream
}  // namespace drake
