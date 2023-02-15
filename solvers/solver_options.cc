#include "drake/solvers/solver_options.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <sstream>
#include <type_traits>

#include "absl/container/flat_hash_set.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {
namespace {

// Boilerplate for std::visit.
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// Magically peek at the SolverId's internal int.
struct IntEater {
  int eaten{};
};
void hash_append(IntEater& self, int x) {
  self.eaten = x;
}
int GetSolverIdInt(const SolverId& id) {
  IntEater hasher;
  hash_append(hasher, id);
  return hasher.eaten;
}

}  // namespace

// The SetOption implementation is designed to be extremely fast. There's a good
// chance that any given option is never retrieved even once (e.g., if it was
// for a solver that was not selected), so we won't bother hashing anything
// here, just appending it staight away.
void SolverOptions::SetOption(const SolverId& id, std::string_view name,
                              int value) {
  table_.push_back(Row{.solver_id = GetSolverIdInt(id),
                       .name = AddStr(name),
                       .skip_back = UpdateBeginsAndReturnSkipBack(id),
                       .kind = kInt,
                       .value = {.int_value = value}});
}

void SolverOptions::SetOption(const SolverId& id, std::string_view name,
                              double value) {
  table_.push_back(Row{.solver_id = GetSolverIdInt(id),
                       .name = AddStr(name),
                       .skip_back = UpdateBeginsAndReturnSkipBack(id),
                       .kind = kDouble,
                       .value = {.double_value = value}});
}

void SolverOptions::SetOption(const SolverId& id, std::string_view name,
                              std::string_view value) {
  table_.push_back(Row{.solver_id = GetSolverIdInt(id),
                       .name = AddStr(name),
                       .skip_back = UpdateBeginsAndReturnSkipBack(id),
                       .kind = kString,
                       .value = {.string_value = AddStr(value)}});
}

namespace internal {
const SolverId& GetCommonOptionPseudoSolverId() {
  static const never_destroyed<SolverId> singleton{"CommonOption"};
  return singleton.access();
}
}  // namespace internal

void SolverOptions::SetOption(CommonSolverOption name, int value) {
  SetOption(internal::GetCommonOptionPseudoSolverId(), to_string(name), value);
}

void SolverOptions::SetOption(CommonSolverOption name, std::string_view value) {
  SetOption(internal::GetCommonOptionPseudoSolverId(), to_string(name), value);
}

std::string_view SolverOptions::get_print_file_name() const {
  auto variant = GetOption(internal::GetCommonOptionPseudoSolverId(),
                           to_string(CommonSolverOption::kPrintFileName));
  if (std::holds_alternative<std::monostate>(variant)) {
    return {};
  }
  if (std::holds_alternative<std::string_view>(variant)) {
    return std::get<std::string_view>(variant);
  }
  throw std::logic_error("kPrintFileName was set to a non-string value");
}

bool SolverOptions::get_print_to_console() const {
  auto variant = GetOption(internal::GetCommonOptionPseudoSolverId(),
                           to_string(CommonSolverOption::kPrintToConsole));
  if (std::holds_alternative<std::monostate>(variant)) {
    return false;
  }
  if (std::holds_alternative<int>(variant)) {
    return std::get<int>(variant);
  }
  throw std::logic_error("kPrintToConsole was set to a non-int value");
}

SolverOptions::OptionValueView SolverOptions::GetOption(
    const SolverId& id, std::string_view name) const {
  // Find the index of table_ where the LIFO row for this `id` lives.
  const Begin* const metadata = FindBegin(id);
  if (metadata == nullptr) {
    return {};
  }

  int index = metadata->table_index;
  while (true) {
    const Row& row = table_[index];

    // We're iterating in LIFO order, so "last one added wins" for us here is
    // "first one found wins".
    if (to_string_view(row.name) == name) {
      switch (row.kind) {
        case kInt:
          return row.value.int_value;
        case kDouble:
          return row.value.double_value;
        case kString:
          return to_string_view(row.value.string_value);
        case kTombstone:
          DRAKE_UNREACHABLE();
      }
      DRAKE_UNREACHABLE();
    }

    // Check if this was the last option for this id, or if we can keep going.
    if (row.skip_back == 0) {
      return {};
    }
    index -= row.skip_back;
  }
}

template <typename T>
std::optional<T> SolverOptions::GetOption(const SolverId& id,
                                          std::string_view name) const {
  auto variant = GetOption(id, name);
  if (std::holds_alternative<T>(variant)) {
    return std::get<T>(variant);
  }
  return std::nullopt;
}

template <typename T>
SolverOptionsRange<T> SolverOptions::GetRange(const SolverId& id) const {
  const Begin* const metadata = FindBegin(id);
  if (metadata == nullptr) {
    return {};
  }

  std::optional<int> begin;

  // Find the first option of the correct type for this id, while also updating
  // the tombstones.
  absl::flat_hash_set<std::string_view> all_names;
  int index = metadata->table_index;
  while (true) {
    DRAKE_ASSERT(index >= 0);
    DRAKE_ASSERT(index < table_.size());
    const Row& row = table_[index];

    // Check if this was the first option of the requested type.
    if (!begin.has_value() && (row.kind == GetKindFor<T>())) {
      begin = index;
      if (!metadata->dirty) {
        break;
      }
    }

    // Check if we've already seen this name.
    const std::string_view row_name = to_string_view(row.name);
    const bool inserted = all_names.insert(row_name).second;
    if (!inserted) {
      const_cast<Row&>(row).kind = kTombstone;
    }

    // Check if this was the last option for this id, or if we can keep going.
    if (row.skip_back == 0) {
      break;
    }
    index -= row.skip_back;
  }
  metadata->dirty = false;

  // Guard in case there were no options of this type.
  if (!begin.has_value()) {
    return {};
  }

  return SolverOptionsRange<T>(SolverOptionsIterator<T>(this, *begin));
}

SolverOptions::Str SolverOptions::AddStr(std::string_view data) {
  if (cstring_arena_.empty()) {
    // In case std::vector has a small default initial capacity, bump up to a
    // reasonable starting point to avoid too much copying as we warm up.
    cstring_arena_.reserve(512);
  }
  const size_t offset = cstring_arena_.size();
  const size_t length = data.length();
  DRAKE_DEMAND(offset <= std::numeric_limits<int>::max());
  DRAKE_DEMAND(length <= std::numeric_limits<int>::max());
  std::copy(data.begin(), data.end(),
            std::back_insert_iterator(cstring_arena_));
  cstring_arena_.push_back('\0');
  return Str{static_cast<int>(offset), static_cast<int>(length)};
}

void SolverOptions::Update(const SolverOptions& other, const SolverId& id) {
  for (const auto& [name, value] : other.GetRange<int>(id)) {
    SetOption(id, name, value);
  }
  for (const auto& [name, value] : other.GetRange<double>(id)) {
    SetOption(id, name, value);
  }
  for (const auto& [name, value] : other.GetRange<std::string_view>(id)) {
    SetOption(id, name, value);
  }
}

std::string SolverOptions::DumpToString(std::string_view quote) const {
  // TODO(jwnimmer-tri) When a string contains `quote` already, we should escape
  // it. This seems unlikely in practice, so we'll overlook that for now.
  std::stringstream result;
  bool first_outer = true;
  result << "{ ";
  for (const auto& [id, options] : CopyToMap()) {
    if (first_outer) {
      first_outer = false;
    } else {
      result << ", ";
    }
    bool first_inner = true;
    result << fmt::format("{}{}{}: {{ ", quote, id.name(), quote);
    for (const auto& [name, value] : options) {
      if (first_inner) {
        first_inner = false;
      } else {
        result << ", ";
      }
      result << fmt::format("{}{}{}: ", quote, name, quote);
      result << std::visit(  // BR
          overloaded{[](int x) {
                       return fmt::to_string(x);
                     },
                     [](double x) {
                       return fmt::format("{:#}", x);
                     },
                     [quote](const std::string& x) {
                       return fmt::format("{}{}{}", quote, x, quote);
                     }},
          value);
    }
    result << " }";
  }
  result << " }";
  return result.str();
}

std::map<SolverId, std::map<std::string, SolverOptions::OptionValue>>
SolverOptions::CopyToMap() const {
  std::map<SolverId, std::map<std::string, OptionValue>> result;

  // Build a lookup table for IDs.
  std::map<int, SolverId> id_lookup;
  for (const auto& [id, _1, _2] : begins_) {
    id_lookup.emplace(GetSolverIdInt(id), id);
  }

  for (const Row& row : table_) {
    const SolverId& id = id_lookup.at(row.solver_id);
    std::string name{to_string_view(row.name)};
    OptionValue value;
    switch (row.kind) {
      case kInt:
        value = row.value.int_value;
        break;
      case kDouble:
        value = row.value.double_value;
        break;
      case kString:
        value = std::string{to_string_view(row.value.string_value)};
        break;
      case kTombstone:
        continue;
    }
    result[id].insert_or_assign(std::move(name), std::move(value));
  }

  return result;
}

std::string_view SolverOptions::to_string_view(const Str& s) const {
  DRAKE_ASSERT(s.offset >= 0);
  DRAKE_ASSERT(s.length >= 0);
  DRAKE_ASSERT(s.offset + s.length + 1 <=
               static_cast<int>(cstring_arena_.size()));
  return {&cstring_arena_[s.offset], static_cast<size_t>(s.length)};
}

int16_t SolverOptions::UpdateBeginsAndReturnSkipBack(const SolverId& id) {
  DRAKE_DEMAND(table_.size() <= std::numeric_limits<int>::max());
  const int next_index = static_cast<int>(table_.size());

  // If we already have options for this solver_id, update its position.
  for (auto& item : begins_) {
    if (item.id == id) {
      const int skip_back = next_index - item.table_index;
      item.table_index = next_index;
      item.dirty = true;
      DRAKE_ASSERT(skip_back > 0);
      DRAKE_DEMAND(skip_back <= std::numeric_limits<int16_t>::max());
      return static_cast<int16_t>(skip_back);
    }
  }

  // This is the first option for this solver_id.
  if (begins_.empty()) {
    // In case std::vector has a small default initial capacity, bump up to a
    // reasonable starting point to avoid too much copying as we warm up.
    begins_.reserve(16);
  }
  begins_.emplace_back(id, next_index, false);
  const int16_t skip_back = 0;
  return skip_back;
}

const SolverOptions::Begin* SolverOptions::FindBegin(const SolverId& id) const {
  for (const auto& item : begins_) {
    if (item.id == id) {
      return &item;
    }
  }
  return nullptr;
}

template <typename T>
SolverOptionsIterator<T>::SolverOptionsIterator(const SolverOptions* parent,
                                                int index)
    : parent_{parent}, index_{index} {
  DRAKE_ASSERT(parent != nullptr);
  DRAKE_ASSERT(index >= 0);
  DRAKE_ASSERT(static_cast<size_t>(index) < parent_->table_.size());
}

template <typename T>
SolverOptionsIterator<T>::value_type SolverOptionsIterator<T>::operator*()
    const {
  DRAKE_DEMAND(parent_ != nullptr);
  DRAKE_ASSERT(index_ >= 0);
  DRAKE_ASSERT(static_cast<size_t>(index_) < parent_->table_.size());
  const SolverOptions::Row& row = parent_->table_[index_];
  const std::string_view name = parent_->to_string_view(row.name);
  if constexpr (std::is_same_v<T, int>) {
    DRAKE_ASSERT(row.kind == SolverOptions::kInt);
    return {name, row.value.int_value};
  } else if constexpr (std::is_same_v<T, double>) {
    DRAKE_ASSERT(row.kind == SolverOptions::kDouble);
    return {name, row.value.double_value};
  } else {
    DRAKE_ASSERT(row.kind == SolverOptions::kString);
    return {name, parent_->to_string_view(row.value.string_value)};
  }
}

template <typename T>
const SolverOptionsIterator<T>& SolverOptionsIterator<T>::operator++() {
  DRAKE_DEMAND(parent_ != nullptr);

  // Skip until we find the next option with the correct type.
  while (true) {
    DRAKE_ASSERT(index_ >= 0);
    DRAKE_ASSERT(static_cast<size_t>(index_) < parent_->table_.size());
    const SolverOptions::Row& old_row = parent_->table_[index_];
    DRAKE_ASSERT(old_row.skip_back >= 0);
    if (old_row.skip_back == 0) {
      // We've reached the end.
      *this = {};
      break;
    }
    index_ -= old_row.skip_back;
    DRAKE_ASSERT(index_ >= 0);
    const SolverOptions::Row& new_row = parent_->table_[index_];
    if (new_row.kind == SolverOptions::GetKindFor<T>()) {
      break;
    }
  }

  return *this;
}

// Explicitly instantiate our templates on their allowed types.

template std::optional<int> SolverOptions::GetOption(  // BR
    const SolverId&, std::string_view) const;
template std::optional<double> SolverOptions::GetOption(  // BR
    const SolverId&, std::string_view) const;
template std::optional<std::string_view> SolverOptions::GetOption(  // BR
    const SolverId&, std::string_view) const;

template SolverOptionsRange<int> SolverOptions::GetRange(  // BR
    const SolverId&) const;
template SolverOptionsRange<double> SolverOptions::GetRange(  // BR
    const SolverId&) const;
template SolverOptionsRange<std::string_view> SolverOptions::GetRange(  // BR
    const SolverId&) const;

template class SolverOptionsIterator<int>;
template class SolverOptionsIterator<double>;
template class SolverOptionsIterator<std::string_view>;

}  // namespace solvers
}  // namespace drake
