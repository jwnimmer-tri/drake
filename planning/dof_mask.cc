#include "drake/planning/dof_mask.h"

#include <cstring>

#include <fmt/format.h>
#include <fmt/ranges.h>

namespace drake {
namespace planning {

using multibody::Joint;
using multibody::JointIndex;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;

namespace {

constexpr int kMaxInlineSize = 64;

int popcount(uint64_t x) {
  return __builtin_popcount(x);
}

}  // namespace

DofMask::DofMask(int size, bool value) {
  DRAKE_THROW_UNLESS(size >= 0);
  size_ = size;
  count_ = value ? size : 0;
  if (size <= kMaxInlineSize) [[likely]] {
    if (value) {
      inline_ = ~inline_;
    }
  } else {
    buffer_.reset(new bool[size]);
    for (int i = 0; i < size; ++i) {
      buffer_[i] = value;
    }
  }
}

DofMask::DofMask(std::initializer_list<bool> values)
    : DofMask(std::vector(std::move(values))) {}

DofMask::DofMask(const std::vector<bool>& values) {
  size_ = ssize(values);
  if (size_ <= kMaxInlineSize) [[likely]] {
    for (int i = 0; i < size_; ++i) {
      const bool bit = values[i];
      inline_ |= (uint64_t{bit} << i);
      if (bit) {
        ++count_;
      }
    }
  } else {
    buffer_.reset(new bool[size_]);
    for (int i = 0; i < size_; ++i) {
      const bool bit = values[i];
      buffer_[i] = bit;
      if (bit) {
        ++count_;
      }
    }
  }
}

// We can't use the defaulted implementation because of our unique_ptr member.
// However, for simplicity we can just delegate to the copy-assignment operator.
DofMask::DofMask(const DofMask& other) {
  *this = other;
}

// We can't use the defaulted implementation because of our unique_ptr member.
DofMask& DofMask::operator=(const DofMask& other) {
  if (this == &other) [[unlikely]] {
    return *this;
  }
  size_ = other.size_;
  count_ = other.count_;
  inline_ = other.inline_;
  if (other.is_inline()) [[likely]] {
    buffer_.reset();
  } else {
    buffer_.reset(new bool[size_]);
    for (int i = 0; i < size_; ++i) {
      buffer_[i] = other.buffer_[i];
    }
  }
  return *this;
}

DofMask::DofMask(DofMask&& other) = default;

DofMask& DofMask::operator=(DofMask&& other) = default;

DofMask DofMask::MakeFromModel(const MultibodyPlant<double>& plant,
                               ModelInstanceIndex model_index) {
  ThrowIfNotCompatible(plant);
  std::vector<bool> bits(plant.num_positions(), false);
  for (const JointIndex& j : plant.GetJointIndices(model_index)) {
    const Joint<double>& joint = plant.get_joint(j);
    if (joint.num_positions() == 0) continue;
    for (int i = joint.position_start();
         i < joint.position_start() + joint.num_positions(); ++i) {
      bits[i] = true;
    }
  }
  return DofMask(std::move(bits));
}

DofMask DofMask::MakeFromModel(const MultibodyPlant<double>& plant,
                               const std::string& model_name) {
  ThrowIfNotCompatible(plant);
  const ModelInstanceIndex model_index =
      plant.GetModelInstanceByName(model_name);
  return DofMask::MakeFromModel(plant, model_index);
}

void DofMask::ThrowIfNotCompatible(const MultibodyPlant<double>& plant) {
  if (!plant.IsVelocityEqualToQDot()) {
    throw std::runtime_error(fmt::format(
        "To use a plant with DofMask, the plant's ith velocity must be the "
        "time derivative of the ith position for all i. This isn't true for "
        "the given plant: '{}'.",
        plant.get_name()));
  }
  // N.B. Since many of our things are purely kinematic, we do not check
  // plant.num_actuated_dofs().
}

std::string DofMask::to_string() const {
  std::vector<int> exploded;
  exploded.resize(size());
  for (int i = 0; i < size(); ++i) {
    exploded[i] = (*this)[i];
  }
  return fmt::to_string(exploded);
}

DofMask DofMask::Complement() const {
  if (is_inline()) [[likely]] {
    return DofMask(
        /* size = */ size_,
        /* count = */ size_ - count_,
        /* inline = */ ~inline_ & all_selected());
  } else {
    DofMask result(
        /* size = */ size_,
        /* count = */ size_ - count_,
        /* inline = */ 0);
    result.buffer_.reset(new bool[size_]);
    for (int i = 0; i < size_; ++i) {
      result.buffer_[i] = !buffer_[i];
    }
    return result;
  }
}

DofMask DofMask::Union(const DofMask& other) const {
  const int mask_size = size();
  DRAKE_THROW_UNLESS(other.size() == mask_size);
  DRAKE_DEMAND(this->is_inline() == other.is_inline());
  DofMask result(mask_size, false);
  if (is_inline()) [[likely]] {
    result.inline_ = (this->inline_ | other.inline_) & all_selected();
    result.count_ = popcount(result.inline_);
  } else {
    for (int i = 0; i < mask_size; ++i) {
      const bool bit = buffer_[i] || other.buffer_[i];
      result.buffer_[i] = bit;
      if (bit) {
        ++result.count_;
      }
    }
  }
  return result;
}

DofMask DofMask::Intersect(const DofMask& other) const {
  const int mask_size = size();
  DRAKE_THROW_UNLESS(other.size() == mask_size);
  DRAKE_DEMAND(this->is_inline() == other.is_inline());
  DofMask result(mask_size, false);
  if (is_inline()) [[likely]] {
    result.inline_ = (this->inline_ & other.inline_) & all_selected();
    result.count_ = popcount(result.inline_);
  } else {
    for (int i = 0; i < mask_size; ++i) {
      const bool bit = buffer_[i] && other.buffer_[i];
      result.buffer_[i] = bit;
      if (bit) {
        ++result.count_;
      }
    }
  }
  return result;
}

DofMask DofMask::Subtract(const DofMask& other) const {
  const int mask_size = size();
  DRAKE_THROW_UNLESS(other.size() == mask_size);
  DRAKE_DEMAND(this->is_inline() == other.is_inline());
  DofMask result(mask_size, false);
  if (is_inline()) [[likely]] {
    result.inline_ = (this->inline_ & ~other.inline_) & all_selected();
    result.count_ = popcount(result.inline_);
  } else {
    for (int i = 0; i < mask_size; ++i) {
      const bool bit = buffer_[i] && !other.buffer_[i];
      result.buffer_[i] = bit;
      if (bit) {
        ++result.count_;
      }
    }
  }
  return result;
}

void DofMask::GetFromArray(const Eigen::Ref<const Eigen::VectorXd>& full_vec,
                           drake::EigenPtr<Eigen::VectorXd> output) const {
  DRAKE_THROW_UNLESS(output != nullptr);
  DRAKE_THROW_UNLESS(output->size() == count());
  DRAKE_THROW_UNLESS(full_vec.size() == size());
  int out_index = -1;
  for (int i = 0; i < size(); ++i) {
    if ((*this)[i]) {
      (*output)[++out_index] = full_vec[i];
      if (out_index >= output->size()) break;
    }
  }
  DRAKE_DEMAND(out_index + 1 == output->size());
}

Eigen::VectorXd DofMask::GetFromArray(
    const Eigen::Ref<const Eigen::VectorXd>& full_vec) const {
  Eigen::VectorXd subset_vec = Eigen::VectorXd::Zero(count());
  GetFromArray(full_vec, &subset_vec);
  return subset_vec;
}

void DofMask::GetColumnsFromMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& full_mat,
    drake::EigenPtr<Eigen::MatrixXd> output) const {
  DRAKE_THROW_UNLESS(output != nullptr);
  DRAKE_THROW_UNLESS(output->cols() == count());
  DRAKE_THROW_UNLESS(full_mat.rows() == output->rows());
  DRAKE_THROW_UNLESS(full_mat.cols() == size());
  int out_index = -1;
  for (int i = 0; i < size(); ++i) {
    if ((*this)[i]) {
      output->col(++out_index) = full_mat.col(i);
      if (out_index >= output->cols()) break;
    }
  }
  DRAKE_DEMAND(out_index + 1 == output->cols());
}

Eigen::MatrixXd DofMask::GetColumnsFromMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& full_mat) const {
  Eigen::MatrixXd columns(full_mat.rows(), count());
  GetColumnsFromMatrix(full_mat, &columns);
  return columns;
}

void DofMask::SetInArray(const Eigen::Ref<const Eigen::VectorXd>& vec,
                         drake::EigenPtr<Eigen::VectorXd> output) const {
  DRAKE_THROW_UNLESS(vec.size() == count());
  DRAKE_THROW_UNLESS(output != nullptr);
  DRAKE_THROW_UNLESS(output->size() == size());
  int input_index = -1;
  for (int i = 0; i < size(); ++i) {
    if ((*this)[i]) {
      (*output)[i] = vec[++input_index];
    }
  }
  DRAKE_DEMAND(input_index + 1 == vec.size());
}

std::vector<JointIndex> DofMask::GetJoints(
    const MultibodyPlant<double>& plant) const {
  DofMask::ThrowIfNotCompatible(plant);
  DRAKE_THROW_UNLESS(size() == plant.num_positions());
  std::vector<JointIndex> result;
  for (const JointIndex& j : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(j);
    for (int i = joint.position_start();
         i < joint.position_start() + joint.num_positions(); ++i) {
      if ((*this)[i]) {
        result.push_back(j);
        break;
      }
    }
  }
  return result;
}

bool DofMask::operator==(const DofMask& o) const {
  if (this->size_ != o.size_) {
    return false;
  }
  DRAKE_DEMAND(this->is_inline() == o.is_inline());
  bool result;
  if (is_inline()) [[likely]] {
    result = (this->inline_ == o.inline_);
  } else {
    result = (std::memcmp(buffer_.get(), o.buffer_.get(), size_) == 0);
  }
  DRAKE_ASSERT((result == false) || (this->count() == o.count()));
  return result;
}

}  // namespace planning
}  // namespace drake
