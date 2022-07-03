#include "drake/common/autodiff/scalar.h"

namespace drake {
namespace internal {

namespace {

// Performs the multiply-accumulate operation for all of the terms across the
// two given arrays.
template <size_t M, size_t N = 0>
std::shared_ptr<const double[]> Squash(
    const size_t size,
    const std::array<LazyMacTerm, M>& terms1,
    const std::array<LazyMacTerm, N>& terms2 = {}) {
  auto new_storage = std::make_unique<double[]>(size);
  Eigen::Map<Eigen::VectorXd> result(new_storage.get(), size);
  result.setZero();
  for (const LazyMacTerm& term : terms1) {
    if (!term.is_zero()) {
      const Eigen::Map<const Eigen::VectorXd> data(term.data.get(), size);
      result.noalias() += term.coeff * data;
    }
  }
  for (const LazyMacTerm& term : terms2) {
    if (!term.is_zero()) {
      const Eigen::Map<const Eigen::VectorXd> data(term.data.get(), size);
      result.noalias() += term.coeff * data;
    }
  }
  return new_storage;
}

}  // namespace

void LazyMac::reset(const Eigen::Ref<const Eigen::VectorXd>& derivatives) {
  size_ = derivatives.size();
  if (size_ > 0) {
    auto new_storage = std::make_unique<double[]>(size_);
    double* data = new_storage.get();
    for (Eigen::Index i = 0; i < size_; ++i) {
      data[i] = derivatives[i];
    }
    // TODO(jwnimmer-tri) This allocates a new shared_ptr control block; ick.
    terms_ = {{{1.0, std::move(new_storage)}}};
  } else {
    terms_ = {};
  }
}

const double* LazyMac::SquashAndGetData() {
  if (empty()) {
    return nullptr;
  }

  // Check if we're already in canonical form.
  bool is_canonical = terms_[0].coeff == 1.0 && terms_[0].data != nullptr;
  for (size_t i = 1; i < terms_.size(); ++i) {
    if (!terms_[i].is_zero()) {
      is_canonical = false;
    }
  }

  // If not, then squash into canonical form now.
  if (!is_canonical) {
    terms_ = {{{1.0, Squash(size_, terms_)}}};
  }

  return terms_[0].data.get();
}

void LazyMac::AddImpl(const LazyMac& other) {
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
  DRAKE_DEMAND(size_ == other.size_);

  // Copy the terms from other. We'll move them into `this` one by one.
  auto other_terms = other.terms_;
  for (LazyMacTerm& other_term : other_terms) {
    other_term.coeff *= scale;
  }

  // Move the non-zero terms from `other_terms` into `this`.
  size_t this_index = 0;
  bool overflow = false;
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

  // We didn't find enough room for everything. We'll need to squash.
  if (overflow) {
    terms_ = {{{1.0, Squash(size_, terms_, other_terms)}}};
  }
}

}  // namespace internal
}  // namespace drake
