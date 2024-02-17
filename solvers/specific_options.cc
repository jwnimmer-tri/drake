#include "drake/solvers/specific_options.h"

#include <algorithm>
#include <vector>

#include "drake/common/overloaded.h"

namespace drake {
namespace solvers {
namespace internal {

using OptionValue = SolverOptions::OptionValue;

CommonSolverOptionValues SpecificOptions::ExtractCommon(
    const SolverOptions& solver_options) {
  CommonSolverOptionValues result;
  if (const auto outer = solver_options.options.find("Drake");
      outer != solver_options.options.end()) {
    const string_unordered_map<OptionValue>& common = outer->second;
    // N.B. SetOption sanity checks the value; we don't need to re-check here.
    if (auto iter = common.find(to_string(CommonSolverOption::kPrintFileName));
        iter != common.end()) {
      result.print_file_name = std::get<std::string>(iter->second);
    }
    if (auto iter = common.find(to_string(CommonSolverOption::kPrintToConsole));
        iter != common.end()) {
      result.print_to_console = std::get<int>(iter->second);
    }
  }
  return result;
}

SpecificOptions::SpecificOptions(const SolverId* id,
                                 const SolverOptions* solver_options)
    : all_options_{solver_options}, solver_name_{id->name()} {
  DRAKE_DEMAND(solver_options != nullptr);
  auto iter = solver_options->options.find(solver_name_);
  if (iter != solver_options->options.end()) {
    specific_options_ = &(iter->second);
  }
}

SpecificOptions::~SpecificOptions() = default;

void SpecificOptions::Respell(
    const std::function<string_unordered_map<SolverOptions::OptionValue>(
        const CommonSolverOptionValues&)>& respell) {
  DRAKE_DEMAND(respell != nullptr);
  respelled_ = respell(ExtractCommon(*all_options_));
}

template <typename Result>
std::optional<Result> SpecificOptions::Pop(std::string_view key) {
  (void)(key);
  // XXX
  return {};
}

template std::optional<double> SpecificOptions::Pop(std::string_view);
template std::optional<int> SpecificOptions::Pop(std::string_view);
template std::optional<bool> SpecificOptions::Pop(std::string_view);
template std::optional<std::string> SpecificOptions::Pop(std::string_view);

void SpecificOptions::CopyToCallbacks(
    const std::function<void(const std::string& key, double)>& set_double,
    const std::function<void(const std::string& key, int)>& set_int,
    const std::function<void(const std::string& key, const std::string&)>&
        set_string) const {
  // Check if we have any options at all for this solver.
  if (specific_options_ == nullptr) {
    return;
  }

  // Wrap the solver's set_{type} callbacks with error-reporting sugar.
  auto on_double = [this, &set_double](const std::string& key, double value) {
    if (set_double != nullptr) {
      set_double(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: floating-point options are not supported; the option {}={} is "
        "invalid",
        solver_name_, key, value));
  };
  auto on_int = [this, &set_int, &set_double](const std::string& key,
                                              int value) {
    if (set_int != nullptr) {
      set_int(key, value);
      return;
    }
    if (set_double != nullptr) {
      set_double(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: integer and floating-point options are not supported; the option "
        "{}={} is invalid",
        solver_name_, key, value));
  };
  auto on_string = [this, &set_string](const std::string& key,
                                       const std::string& value) {
    if (set_string != nullptr) {
      set_string(key, value);
      return;
    }
    throw std::logic_error(fmt::format(
        "{}: string options are not supported; the option {}='{}' is invalid",
        solver_name_, key, value));
  };

  // Set all options.
  for (const auto& [key, boxed_value] : *specific_options_) {
    std::visit(overloaded{[&key, &on_double](double value) {
                            on_double(key, value);
                          },
                          [&key, &on_int](int value) {
                            on_int(key, value);
                          },
                          [&key, &on_string](const std::string& value) {
                            on_string(key, value);
                          }},
               boxed_value);
  }
}

void SpecificOptions::InitializePending() {
  if (specific_options_ == nullptr) {
    return;
  }

  pending_keys_.clear();
  pending_keys_.reserve(specific_options_->size());
  for (const auto& [key, _] : *specific_options_) {
    pending_keys_.insert(key);
  }
}

void SpecificOptions::CheckNoPending() const {
  // Identify any unsupported names (i.e., leftovers in `pending_`).
  if (!pending_keys_.empty()) {
    std::vector<std::string_view> unknown_names;
    for (const auto& name : pending_keys_) {
      unknown_names.push_back(name);
    }
    std::sort(unknown_names.begin(), unknown_names.end());
    throw std::logic_error(fmt::format(
        "{}: the following solver option names were not recognized: {}",
        solver_name_, fmt::join(unknown_names, ", ")));
  }
}

const OptionValue* SpecificOptions::PrepareToCopy(const char* name) {
  DRAKE_DEMAND(name != nullptr);
  if (specific_options_ != nullptr) {
    if (auto iter = specific_options_->find(name);
        iter != specific_options_->end()) {
      pending_keys_.erase(iter->first);
      const OptionValue& value = iter->second;
      return &value;
    }
  }
  return nullptr;
}

template <typename T>
void SpecificOptions::CopyFloatingPointOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto* value = PrepareToCopy(name)) {
    if (std::holds_alternative<double>(*value)) {
      *output = std::get<double>(*value);
      return;
    }
    if (std::holds_alternative<int>(*value)) {
      *output = std::get<int>(*value);
      return;
    }
    throw std::logic_error(
        fmt::format("{}: Expected a floating-point value for option {}={}",
                    solver_name_, name, internal::Repr(*value)));
  }
}
template void SpecificOptions::CopyFloatingPointOption(const char*, double*);
template void SpecificOptions::CopyFloatingPointOption(const char*, float*);

template <typename T>
void SpecificOptions::CopyIntegralOption(const char* name, T* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto* value = PrepareToCopy(name)) {
    if (std::holds_alternative<int>(*value)) {
      *output = std::get<int>(*value);
      return;
    }
    throw std::logic_error(
        fmt::format("{}: Expected an integer value for option {}={}",
                    solver_name_, name, internal::Repr(*value)));
  }
}
template void SpecificOptions::CopyIntegralOption(const char*, int*);
template void SpecificOptions::CopyIntegralOption(const char*, bool*);
template void SpecificOptions::CopyIntegralOption(const char*, uint32_t*);

void SpecificOptions::CopyStringOption(const char* name, std::string* output) {
  DRAKE_DEMAND(output != nullptr);
  if (auto* value = PrepareToCopy(name)) {
    if (std::holds_alternative<std::string>(*value)) {
      *output = std::get<std::string>(*value);
      return;
    }
    throw std::logic_error(
        fmt::format("{}: Expected a string value for option {}={}",
                    solver_name_, name, internal::Repr(*value)));
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
