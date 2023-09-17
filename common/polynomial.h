#pragma once

#include <complex>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Core>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"

namespace drake {

template <typename T = double>
using Polynomial = symbolic::Polynomial;

typedef Polynomial<double> Polynomiald;

/// A column vector of polynomials; used in several optimization classes.
typedef Eigen::Matrix<Polynomiald, Eigen::Dynamic, 1> VectorXPoly;
}  // namespace drake
