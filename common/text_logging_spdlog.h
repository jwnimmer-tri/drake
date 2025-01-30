#pragma once

#include <spdlog/spdlog.h>

namespace drake {
namespace logging {

// XXX don't require a cast
/** (Advanced) Retrieves the default sink for all Drake logs.  When spdlog is
enabled, the return value can be cast to spdlog::sinks::dist_sink_mt and thus
allows consumers of Drake to redirect Drake's text logs to locations other than
the default of stderr.  When spdlog is disabled, the return value is an empty
class. */
spdlog::sinks::sink* get_dist_sink();

}  // namespace logging
}  // namespace drake
