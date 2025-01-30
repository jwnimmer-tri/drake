#pragma once

#ifdef HAVE_SPDLOG
#include <spdlog/spdlog.h>
#endif

#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace logging {

#ifdef HAVE_SPDLOG
using spdlog::sinks::sink;
#else
// A stubbed-out version of `spdlog::sinks::sink`.
class sink {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(sink);
  sink() = default;
};
#endif

/// (Advanced) Retrieves the default sink for all Drake logs.  When spdlog is
/// enabled, the return value can be cast to spdlog::sinks::dist_sink_mt and
/// thus allows consumers of Drake to redirect Drake's text logs to locations
/// other than the default of stderr.  When spdlog is disabled, the return
/// value is an empty class.
sink* get_dist_sink();

}  // namespace logging
}  // namespace drake
