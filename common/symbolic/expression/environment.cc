/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include): Our header file is included by all.h.
#include "drake/common/symbolic/expression/all.h"
/* clang-format on */

#include <cmath>
#include <initializer_list>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <fmt/format.h>

namespace drake {
namespace symbolic {

using std::endl;
using std::initializer_list;
using std::move;
using std::ostringstream;
using std::runtime_error;
using std::string;

namespace {
void throw_if_dummy(const Variable& var) {
  if (var.is_dummy()) {
    throw runtime_error(
        "Dummy variable (ID = 0) is detected in the initialization of an "
        "environment.");
  }
}

void throw_if_nan(const double v) {
  if (std::isnan(v)) {
    throw runtime_error(
        "NaN is detected in the initialization of an environment.");
  }
}

// Given a list of variables, @p vars, builds an Environment::map which maps a
// Variable to its double value. All values are set to 0.0.
Environment::map BuildMap(const initializer_list<Environment::key_type> vars) {
  Environment::map m;
  for (const Environment::key_type& var : vars) {
    m.emplace(var, 0.0);
  }
  return m;
}

}  // anonymous namespace

Environment::Environment(const std::initializer_list<value_type> init)
    : Environment{map(init)} {}

Environment::Environment(const std::initializer_list<key_type> vars)
    : Environment{BuildMap(vars)} {}

Environment::Environment(map m) : map_{move(m)} {
  for (const auto& p : map_) {
    throw_if_dummy(p.first);
    throw_if_nan(p.second);
  }
}

void Environment::insert(const key_type& key, const mapped_type& elem) {
  throw_if_dummy(key);
  throw_if_nan(elem);
  map_.emplace(key, elem);
}

void Environment::insert(
    const Eigen::Ref<const MatrixX<key_type>>& keys,
    const Eigen::Ref<const MatrixX<mapped_type>>& elements) {
  if (keys.rows() != elements.rows() || keys.cols() != elements.cols()) {
    throw runtime_error(fmt::format(
        "symbolic::Environment::insert: The size of keys ({} x {}) "
        "does not match the size of elements ({} x {}).",
        keys.rows(), keys.cols(), elements.rows(), elements.cols()));
  }

  for (Eigen::Index i = 0; i < keys.cols(); ++i) {
    for (Eigen::Index j = 0; j < keys.rows(); ++j) {
      insert(keys(j, i), elements(j, i));
    }
  }
}

Variables Environment::domain() const {
  Variables dom;
  for (const auto& p : map_) {
    dom += p.first;
  }
  return dom;
}

string Environment::to_string() const {
  ostringstream result;
  for (const auto& [var, value] : *this) {
    result << fmt::format("{} -> {}\n", var, value);
  }
  return result.str();
}

Environment::mapped_type& Environment::operator[](const key_type& key) {
  if (key.is_dummy()) {
    throw runtime_error(
        "Environment::operator[] is called with a dummy variable.");
  }
  return map_[key];
}

const Environment::mapped_type& Environment::operator[](
    const key_type& key) const {
  if (key.is_dummy()) {
    throw runtime_error(
        "Environment::operator[] is called with a dummy variable.");
  }
  if (map_.count(key) == 0) {
    throw runtime_error(fmt::format(
        "Environment::operator[] was called on a const Environment with a "
        "missing key \"{}\".",
        key));
  }
  return map_.at(key);
}

Environment PopulateRandomVariables(Environment env, const Variables& variables,
                                    RandomGenerator* const random_generator) {
  DRAKE_DEMAND(random_generator != nullptr);
  for (const Variable& var : variables) {
    const auto it = env.find(var);
    if (it != env.end()) {
      // The variable is already assigned by env, no need to sample.
      continue;
    }
    switch (var.get_type()) {
      case Variable::Type::CONTINUOUS:
      case Variable::Type::BINARY:
      case Variable::Type::BOOLEAN:
      case Variable::Type::INTEGER:
        // Do nothing for non-random variables.
        break;
      case Variable::Type::RANDOM_UNIFORM:
        env.insert(var, std::uniform_real_distribution<double>{
                            0.0, 1.0}(*random_generator));
        break;
      case Variable::Type::RANDOM_GAUSSIAN:
        env.insert(
            var, std::normal_distribution<double>{0.0, 1.0}(*random_generator));
        break;
      case Variable::Type::RANDOM_EXPONENTIAL:
        env.insert(
            var, std::exponential_distribution<double>{1.0}(*random_generator));
        break;
    }
  }
  return env;
}

}  // namespace symbolic
}  // namespace drake
