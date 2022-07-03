#include "drake/common/autodiff/scalar.h"

namespace drake {
namespace internal {

namespace {

// Performs the multiply-accumulate operation for all of the terms across the
// two given arrays.
template <size_t M, size_t N = 0>
LazyMacTerm Squash(
    const size_t size,
    const std::array<LazyMacTerm, M>& terms1,
    const std::array<LazyMacTerm, N>& terms2 = {}) {
  LazyMacTerm result(Eigen::VectorXd::Zero(size));
  Eigen::Map<Eigen::VectorXd> vec(result.mutable_data(size), size);
  for (const LazyMacTerm& term : terms1) {
    if (!term.is_zero()) {
      const Eigen::Map<const Eigen::VectorXd> data(term.data(), size);
      vec.noalias() += term.coeff() * data;
    }
  }
  for (const LazyMacTerm& term : terms2) {
    if (!term.is_zero()) {
      const Eigen::Map<const Eigen::VectorXd> data(term.data(), size);
      vec.noalias() += term.coeff() * data;
    }
  }
  return result;
}

}  // namespace

LazyMacTerm::LazyMacTerm(const Eigen::Ref<const Eigen::VectorXd>& derivatives) {
  const Eigen::Index size = derivatives.size();
  if (size == 0) {
    return;
  }

  coeff_ = 1.0;
  const size_t bytes = sizeof(UseCount) + (sizeof(double) * size);
  void* storage = std::malloc(bytes);
  if (storage == nullptr) {
    throw std::bad_alloc();
  }
  new (storage) UseCount(1);
  data_ = reinterpret_cast<double*>(
      static_cast<char*>(storage) + sizeof(UseCount));
  for (Eigen::Index i = 0; i < size; ++i) {
    data_[i] = derivatives[i];
  }
}

double* LazyMacTerm::mutable_data(Eigen::Index size) {
  if (data_ == nullptr) {
    // XXX no need to zero it out here.
    *this = LazyMacTerm(Eigen::VectorXd::Zero(size));
  } else if (use_count().load() > 1) {
    const Eigen::Map<const Eigen::VectorXd> vec(data_, size);
    LazyMacTerm copy{vec};
    *this = std::move(copy);
  }
  DRAKE_ASSERT(data_ != nullptr);
  DRAKE_ASSERT(use_count().load() == 1);
  return data_;
}

void LazyMac::reset(const Eigen::Ref<const Eigen::VectorXd>& derivatives) {
  size_ = derivatives.size();
  terms_ = {{LazyMacTerm{derivatives}}};
}

double* LazyMac::SquashAndGetData() {
  if (empty()) {
    return nullptr;
  }

  // We need exclusive (mutable) access to our storage.
  // If we're shared (readonly) access, then now's the time to copy-on-write.
  double* result = terms_[0].mutable_data(size());
  Eigen::Map<Eigen::VectorXd> result_vec(result, size());
  const double coeff0 = terms_[0].coeff();
  if (coeff0 == 0.0) {
    result_vec.setZero();
  } else if (coeff0 != 1.0) {
    result_vec *= coeff0;
    terms_[0].coeff() = 1.0;
  }
  for (size_t i = 1; i < terms_.size(); ++i) {
    if (terms_[i].is_zero()) {
      continue;
    }
    const Eigen::Map<const Eigen::VectorXd> term_vec(terms_[i].data(), size());
    result_vec.noalias() += terms_[i].coeff() * term_vec;
  }

  return result;
}

void LazyMac::AddImpl(const LazyMac& other) {
  DRAKE_ASSERT(!other.empty());
  // TODO(jwnimmer-tri) We could save some dummy multiplies (by 1.0) here,
  // with a slightly more nuanced implementation.
  AddMulImpl(1.0, other);
}

void LazyMac::AddMulImpl(double scale, const LazyMac& other) {
  DRAKE_ASSERT(scale != 0.0);
  DRAKE_ASSERT(!other.empty());
  if (empty()) {
    *this = other;
    Mul(scale);
    return;
  }

  if (this == &other) {
    Mul(scale + 1.0);
    return;
  }

  // Both `this` and `other` are of non-zero size.
  // In that case, the sizes must match.
  DRAKE_DEMAND(size() == other.size());

  // Copy the terms from other. We'll move them into `this` one by one.
  auto other_terms = other.terms_;
  for (LazyMacTerm& other_term : other_terms) {
    other_term.coeff() *= scale;
  }

  // If we have exclusive storage, then we can mutate it in place.
  if (terms_[0].data() != nullptr &&
      terms_[0].use_count().load() == 1) {
    double* data0 = const_cast<double*>(terms_[0].data());
    Eigen::Map<Eigen::VectorXd> vec0(data0, size());
    if (terms_[0].coeff() != 1.0) {
      vec0 *= terms_[0].coeff();
      terms_[0].coeff() = 1.0;
    }
    for (LazyMacTerm& other_term : other_terms) {
      if (other_term.is_zero()) {
        continue;
      }
      const Eigen::Map<const Eigen::VectorXd> data(other_term.data(), size());
      vec0.noalias() += other_term.coeff() * data;
    }
    return;
  }

  bool overflow = false;
  if (terms_.size() == 1) {
    overflow = true;
  } else {
    // XXX Most likely, this code should be purged (size N == 1).
    // Move the non-zero terms from `other_terms` into `this`.
    size_t this_index = 0;
    for (LazyMacTerm& other_term : other_terms) {
      // Don't move empty terms.
      if (other_term.is_zero()) {
        continue;
      }
      // Move the other_term into the next open slot in `this`.
      for (; !overflow; ++this_index) {
        if (this_index >= terms_.size()) {
          overflow = true;
        } else if (terms_[this_index].is_zero()) {
          terms_[this_index] = std::move(other_term);
          break;
        }
      }
    }
  }

  // We didn't find enough room for everything. We'll need to squash.
  if (overflow) {
    terms_ = {{Squash(size(), terms_, other_terms)}};
  }
}

}  // namespace internal
}  // namespace drake
