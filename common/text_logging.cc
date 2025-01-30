#include "drake/common/text_logging.h"

namespace drake {

logging::logger* log() {
  // We can't use never_destroyed due to our private constructor.
  static logging::logger* g_logger = new logging::logger;
  return g_logger;
}

}  // namespace drake
