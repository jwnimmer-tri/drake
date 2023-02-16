#pragma once

#include "drake/common/drake_deprecated.h"

// TODO(jwnimmer-tri) On 2023-06-01 also remove the <ostream> include.
#include <ostream>
#include <string>

#include "drake/common/fmt_ostream.h"

namespace drake {
namespace yaml {

/** Configuration for LoadYamlFile() and LoadYamlString() to govern when certain
conditions are errors or not. Refer to the member fields for details. */
struct LoadYamlOptions {
  /** Allows yaml Maps to have extra key-value pairs that are not Visited by the
  Serializable being parsed into. In other words, the Serializable types provide
  an incomplete schema for the YAML data. This allows for parsing only a subset
  of the YAML data. */
  bool allow_yaml_with_no_cpp{false};

  /** Allows Serializables to provide more key-value pairs than are present in
  the YAML data. In other words, the structs have default values that are left
  intact unless the YAML data provides a value. */
  bool allow_cpp_with_no_yaml{false};

  /** If set to true, when parsing a std::map the Archive will merge the YAML
  data into the destination, instead of replacing the std::map contents
  entirely. In other words, a visited std::map can have default values that are
  left intact unless the YAML data provides a value *for that specific key*. */
  bool retain_map_defaults{false};

  std::string ToString() const;
};

DRAKE_DEPRECATED("2023-06-01", "Use fmt or spdlog for logging, not operator<<.")
inline std::ostream& operator<<(std::ostream& os, const LoadYamlOptions& self) {
  return os << self.ToString();
}

}  // namespace yaml
}  // namespace drake


// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::yaml::LoadYamlOptions>
    : drake::ostream_formatter {};
}  // namespace fmt
