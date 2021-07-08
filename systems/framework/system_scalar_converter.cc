#include "drake/systems/framework/system_scalar_converter.h"

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/hash.h"
#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/system_base.h"

using std::pair;
using std::type_index;
using std::type_info;
using drake::symbolic::Expression;

namespace drake {
namespace systems {

SystemScalarConverter::Key::Key(
    const type_info& t_info, const type_info& u_info)
    : pair<type_index, type_index>(t_info, u_info) {}

size_t SystemScalarConverter::KeyHasher::operator()(const Key& key) const {
  drake::DefaultHasher hasher;
  using drake::hash_append;
  hash_append(hasher, std::hash<std::type_index>{}(key.first));
  hash_append(hasher, std::hash<std::type_index>{}(key.second));
  return static_cast<size_t>(hasher);
}

SystemScalarConverter::SystemScalarConverter() = default;

SystemScalarConverter::~SystemScalarConverter() = default;

bool SystemScalarConverter::empty() const {
  return funcs_.empty();
}

void SystemScalarConverter::Insert(Key key, ErasedConverterFunc converter) {
  const auto& insert_result = funcs_.insert(
      {std::move(key), std::move(converter)});
  DRAKE_DEMAND(insert_result.second);
}

template <typename T, typename U>
void SystemScalarConverter::Remove() {
  funcs_.erase(Key(typeid(T), typeid(U)));
}

void SystemScalarConverter::RemoveUnlessAlsoSupportedBy(
    const SystemScalarConverter& other) {
  // Remove the items from `funcs_` whose key is absent from `other`.
  // (This would use erase_if, if we had it.)
  for (auto iter = funcs_.begin(); iter != funcs_.end(); ) {
    const Key& our_key = iter->first;
    if (other.funcs_.count(our_key) == 0) {
      iter = funcs_.erase(iter);
    } else {
      ++iter;
    }
  }
}

template <typename T, typename U>
bool SystemScalarConverter::IsConvertible() const {
  return funcs_.count(Key(typeid(T), typeid(U))) > 0;
}

std::unique_ptr<SystemBase> SystemScalarConverter::Convert(
    const Key& key, const SystemBase& other) const {
  SystemBase* result = nullptr;
  auto iter = funcs_.find(key);
  if (iter != funcs_.end()) {
    auto& constructor = iter->second;
    result = static_cast<SystemBase*>(constructor(&other));
    DRAKE_DEMAND(result != nullptr);
    // We manually propagate the name from the old System to the new.  The name
    // is the only extrinsic property of the System and LeafSystem base classes
    // that is stored within the System itself.
    result->set_name(other.get_name());
  }
  return std::unique_ptr<SystemBase>(result);
}

void SystemScalarConverter::ThrowConversionMismatch(
    const type_info& s_t_info, const type_info& s_u_info,
    const type_info& other_info) {
  throw std::runtime_error(fmt::format(
      "SystemScalarConverter was configured to convert a {} into a {}"
      " but was called with a {} at runtime",
      NiceTypeName::Get(s_u_info), NiceTypeName::Get(s_t_info),
      NiceTypeName::Get(other_info)));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &SystemScalarConverter::Remove<T, U>,
    &SystemScalarConverter::IsConvertible<T, U>
))

}  // namespace systems
}  // namespace drake
