#include "drake/common/yaml/yaml_io_options.h"

#include <fmt/format.h>

namespace drake {
namespace yaml {

std::string LoadYamlOptions::ToString() const {
  return fmt::format(
      "{{.allow_yaml_with_no_cpp = {}, "
      ".allow_cpp_with_no_yaml = {}, "
      ".retain_map_defaults = {}}}",
      allow_yaml_with_no_cpp,
      allow_cpp_with_no_yaml,
      retain_map_defaults);
}

}  // namespace yaml
}  // namespace drake
