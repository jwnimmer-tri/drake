#pragma once

#include <memory>
#include <string>

#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/** (Advanced) An abstract base class to consolidate event handling for systems
that publish to Meshcat.

Instances of %MeshcatVisualizerBase created by scalar-conversion will publish
to the same Meshcat instance.
@tparam_nonsymbolic_scalar */
template <typename T>
class MeshcatVisualizerBase : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatVisualizerBase)

  /** Creates an instance of %MeshcatVisualizerBase. */
  MeshcatVisualizerBase(systems::SystemScalarConverter converter,
                        std::shared_ptr<Meshcat> meshcat, std::string prefix,
                        double publish_period, double publish_offset,
                        bool delete_on_initialization_event);

  ~MeshcatVisualizerBase() override;

  /** Calls Meshcat::Delete(), with the `path` set to `prefix`. Since this
  system should only ever add geometry under this prefix, this will remove all
  geometry/transforms added by this system, or by a previous instance of this
  system using the same prefix.  Use `delete_on_initialization_event` to
  determine whether this should be called on initialization. */
  void Delete() const;

 protected:
  /** Meshcat is mutable because we must send messages (a non-const operation)
  from a const System (e.g., during simulation). */
  Meshcat& meshcat() const { return *meshcat_; }
  std::shared_ptr<Meshcat> shared_meshcat() const { return meshcat_; }
  const std::string& prefix() const { return prefix_; }
  double publish_period() const { return publish_period_; }
  double publish_offset() const { return publish_offset_; }
  bool delete_on_initialization_event() const {
    return delete_on_initialization_event_;
  }

  /** The (required) subclass publish event handler. */
  virtual systems::EventStatus DoOnPublish(
      double time, const systems::Context<T>& context) const = 0;

  /** The (optional) subclass initialization event handler. */
  virtual systems::EventStatus DoOnInitialize(
      double time, const systems::Context<T>& context) const;

  /** The (optional) delete handler. */
  virtual systems::EventStatus DoOnDelete() const;

  /** The NVI implementation of GetGraphvizFragment() for subclasses to override
  if desired. The default behavior should be sufficient in most cases. */
  typename systems::LeafSystem<T>::GraphvizFragment DoGetGraphvizFragment(
      const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
      const override;

 private:
  template <typename>
  friend class MeshcatVisualizerBase;

  /* The publish event callback. */
  systems::EventStatus OnPublish(const systems::Context<T>&) const;

  /* The initialization event callback. */
  systems::EventStatus OnInitialize(const systems::Context<T>&) const;

  /* Meshcat is mutable because we must send messages (a non-const operation)
  from a const System (e.g., during simulation). We use shared_ptr instead of
  unique_ptr to facilitate sharing ownership through scalar conversion; creating
  a new Meshcat object during the conversion is not a viable option. */
  mutable std::shared_ptr<Meshcat> meshcat_{};

  const std::string prefix_;
  const double publish_period_;
  const double publish_offset_;
  const bool delete_on_initialization_event_;
};

}  // namespace geometry
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatVisualizerBase)
