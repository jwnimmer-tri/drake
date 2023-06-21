#include <algorithm>
#include <regex>

#include <gflags/gflags.h>
#include <spdlog/sinks/dist_sink.h>

#include "drake/common/text_logging.h"
#include "drake/multibody/parsing/parser.h"

DEFINE_bool(scene_graph, true, "include/exclude scene graph");
DEFINE_bool(strict, false, "enable strict parsing");
DEFINE_bool(werror, false, "promote warnings to errors");

namespace drake {
namespace multibody {
namespace {

/* A simple log sink that remembers the most severe log message level. */
class LevelMonitor final
    : public spdlog::sinks::base_sink<spdlog::details::null_mutex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LevelMonitor)

  LevelMonitor() = default;

  /* Returns the max level that was ever logged. */
  spdlog::level::level_enum max_level() const {
    return max_level_;
  }

 private:
  void sink_it_(const spdlog::details::log_msg& msg) final {
    max_level_ = std::max(msg.level, max_level_);
  }

  void flush_() final {}

 private:
  spdlog::level::level_enum max_level_{spdlog::level::trace};
};

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[INPUT-FILE-OR-URL ...]\n"
                          "Run multibody parser; print errors if any");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    drake::log()->error("missing input filename");
    return 1;
  }

  // Hardcode a log pattern that gives us un-decorated error messages.
  // This defeats the `-spdlog_pattern` command line option; oh well.
  drake::logging::set_log_pattern("%v");

  std::shared_ptr<LevelMonitor> level_monitor;
  if (FLAGS_werror) {
    // Add a spdlog sink that detects warnings. Not all warnings come from the
    // parser, so this can detect some problems that strict mode doesn't.
    level_monitor = std::make_shared<LevelMonitor>();
    dynamic_cast<spdlog::sinks::dist_sink_mt&>(*logging::get_dist_sink())
        .add_sink(level_monitor);
  }


  int total_num_models = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string file_or_url = argv[i];
    drake::log()->info("parsing {}", file_or_url);

    const bool is_url = std::regex_search(
        file_or_url, std::regex("^[A-Za-z0-9+.-]+://"));
    MultibodyPlant<double> plant{0.0};
    drake::geometry::SceneGraph<double> scene_graph;
    if (FLAGS_scene_graph) {
      plant.RegisterAsSourceForSceneGraph(&scene_graph);
    }
    Parser parser{&plant};
    parser.package_map().PopulateFromRosPackagePath();
    if (FLAGS_strict) {
      parser.SetStrictParsing();
    }

    // Chances are good that parsing may end in an error (implemented as throw).
    // For purposes of this tool, we should make its message usable. To do that,
    // catch it and display it via logging. Yes, this violates the coding
    // standard; in this case it makes the tool significantly more useful.
    std::vector<ModelInstanceIndex> models;
    try {
      if (is_url) {
        models = parser.AddModelsFromUrl(file_or_url);
      } else {
        models = parser.AddModels(file_or_url);
      }
    } catch (const std::exception& e) {
      drake::log()->error(e.what());
      ::exit(EXIT_FAILURE);
    }
    if (level_monitor && level_monitor->max_level() >= spdlog::level::warn) {
      drake::log()->error("Error: Warnings were detected");
      ::exit(EXIT_FAILURE);
    }
    total_num_models += models.size();
  }
  if (total_num_models == 0) {
    drake::log()->error("Error: No models were parsed");
    ::exit(EXIT_FAILURE);
  }

  drake::log()->info("Parsed {} models.", total_num_models);
  return 0;
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}
