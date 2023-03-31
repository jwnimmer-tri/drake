#pragma once

#include <spdlog/sinks/dist_sink.h>

namespace drake {
namespace logging {

/** (Advanced) Retrieves the default sink for all Drake logs. This allows
consumers of Drake to redirect Drake's text logs to locations other than the
default of stderr.

@warning Per the C++ "One Definition Rule" this function is only valid to call
if you ensure that the version of spdlog on your #include path and preprocessor
definitions exactly match the version and definitions used when compiling Drake.
*/
spdlog::sinks::dist_sink_mt* get_dist_sink();

}  // namespace logging
}  // namespace drake
