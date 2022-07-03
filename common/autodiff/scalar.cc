#include "drake/common/autodiff/scalar.h"

namespace drake {
namespace internal {

CowVec CowVec::Allocate(Eigen::Index size) {
  DRAKE_DEMAND(size >= 0);

  CowVec result;
  if (size == 0) {
    return result;
  }

  const size_t bytes = sizeof(UseCount) + (sizeof(double) * size);
  void* storage = std::malloc(bytes);
  if (storage == nullptr) {
    throw std::bad_alloc();
  }
  new (storage) UseCount(1);
  result.data_ = reinterpret_cast<double*>(
      static_cast<char*>(storage) + sizeof(UseCount));

  return result;
}

CowVec CowVec::Copy(
    const Eigen::Ref<const Eigen::VectorXd>& derivatives) {
  const Eigen::Index size = derivatives.size();
  CowVec result = Allocate(size);
  for (Eigen::Index i = 0; i < size; ++i) {
    result.data_[i] = derivatives[i];
  }
  return result;
}

double* CowVec::mutable_data(Eigen::Index size) {
  if (data_ == nullptr) {
    *this = CowVec::Allocate(size);
    for (Eigen::Index i = 0; i < size; ++i) {
      data_[i] = 0.0;
    }
  } else if (use_count().load() > 1) {
    const Eigen::Map<const Eigen::VectorXd> vec(data_, size);
    *this = CowVec::Copy(vec);
  }
  DRAKE_ASSERT(data_ != nullptr);
  DRAKE_ASSERT(use_count().load() == 1);
  return data_;
}

Partials::Partials(const Eigen::Ref<const Eigen::VectorXd>& derivatives)
    : size_{derivatives.size()},
      coeff_{size_ > 0 ? 1.0 : 0.0},
      storage_{CowVec::Copy(derivatives)} {
  DRAKE_ASSERT_VOID(CheckInvariants());
}

void Partials::CheckInvariants() const {
  DRAKE_DEMAND(size_ >= 0);
  if (size_ == 0) {
    DRAKE_DEMAND(coeff_ == 0.0);
    DRAKE_DEMAND(storage_.data() == nullptr);
  } else {
    DRAKE_DEMAND(coeff_ != 0.0);
    DRAKE_DEMAND(storage_.data() != nullptr);
    DRAKE_DEMAND(storage_.use_count().load() >= 1);
  }
}

double* Partials::SquashAndGetData() {
  DRAKE_ASSERT_VOID(CheckInvariants());

  if (empty()) {
    return nullptr;
  }

  DRAKE_DEMAND(coeff_ != 0.0);
  // We need exclusive (mutable) access to our storage.
  // If we're shared (readonly) access, then now's the time to copy-on-write.
  double* result = storage_.mutable_data(size_);
  Eigen::Map<Eigen::VectorXd> result_vec(result, size_);
  result_vec *= coeff_;
  coeff_ = 1.0;

  DRAKE_ASSERT_VOID(CheckInvariants());
  return result;
}

void Partials::AddImpl(const Partials& other) {
  DRAKE_ASSERT(!other.empty());
  // TODO(jwnimmer-tri) We could save some dummy multiplies (by 1.0) here,
  // with a slightly more nuanced implementation.
  AddMulImpl(1.0, other);
}

void Partials::AddMulImpl(double scale, const Partials& other) {
  DRAKE_ASSERT_VOID(CheckInvariants());
  DRAKE_ASSERT_VOID(other.CheckInvariants());
  DRAKE_ASSERT(scale != 0.0);
  DRAKE_ASSERT(!other.empty());
  if (empty()) {
    *this = other;
    Mul(scale);
    return;
  }

  // Both `this` and `other` are of non-zero size.
  // In that case, the sizes must match.
  DRAKE_DEMAND(size_ == other.size_);

  if (storage_.data() == other.storage_.data()) {
    coeff_ += scale * other.coeff_;
    return;
  }

  // Copy (and lock) the term from other.
  auto other_storage = other.storage_;

  // If we have exclusive storage, then we can mutate it in place.
  if (storage_.data() != nullptr &&
      storage_.use_count().load() == 1) {
    double* this_data = const_cast<double*>(storage_.data());
    const double* other_data = other_storage.data();
    for (Eigen::Index i = 0; i < size_; ++i) {
      this_data[i] = coeff_ * this_data[i]
          + scale * other.coeff_ * other_data[i];
    }
    coeff_ = 1.0;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // We squash.
  auto result = CowVec::Allocate(size_);
  Eigen::Map<Eigen::VectorXd> result_vec(
      const_cast<double*>(result.data()), size_);
  result_vec.noalias() = 
      coeff_ *
          Eigen::Map<const Eigen::VectorXd>(storage_.data(), size_)
      + (scale * other.coeff_) *
            Eigen::Map<const Eigen::VectorXd>(other_storage.data(), size_);
  coeff_ = 1.0;
  storage_ = std::move(result);
  DRAKE_ASSERT_VOID(CheckInvariants());
}

}  // namespace internal
}  // namespace drake
