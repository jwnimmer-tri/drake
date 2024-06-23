#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/hydroelastic_quadrature_point_data.h"

namespace drake {
namespace multibody {

/**
 A class containing information regarding contact and contact response between
 two geometries attached to a pair of bodies. This class provides the output
 from the Hydroelastic contact model and includes:

    - The shared contact surface between the two geometries, which includes
      the virtual pressures acting at every point on the contact surface.
    - The tractions acting at the quadrature points on the contact surface.
    - The slip speeds at the quadrature points on the contact surface.
    - The spatial force from the integrated tractions that is applied at the
      centroid of the contact surface.

 The two geometries, denoted M and N (and obtainable via
 `contact_surface().id_M()` and `contact_surface().id_N()`) are attached to
 bodies A and B, respectively.

 @tparam_default_scalar */
template <typename T>
class HydroelasticContactInfo {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(HydroelasticContactInfo);

  /** @name                 Construction
   The constructors below adhere to a number of invariants. The
   geometry::ContactSurface defines contact between two geometries M and N (via
   contact_surface().id_M() and contact_surface().id_N(), respectively).
   %HydroelasticContactInfo further associates geometries M and N with the
   bodies to which they are rigidly fixed, A and B, respectively. It is the
   responsibility of the caller of these constructors to ensure that the
   parameters satisfy the documented invariants, see below. Similarly, the
   spatial force `F_Ac_W` must be provided as indicated by the monogram notation
   in use, that is, it is the spatial force on body A, at the contact surface's
   centroid C, and expressed in the world frame. Quadrature points data must be
   provided in accordance to the conventions and monogram notation documented in
   HydroelasticQuadraturePointData. */
  // @{

  /**
   @anchor hydro_contact_info_non_owning_ctor
   Constructs this structure using the given contact surface, traction field,
   and slip field. This constructor does not own the ContactSurface; it points
   to a ContactSurface that another owns, see contact_surface().

   @param[in] contact_surface Contact surface between two geometries M and N,
     see geometry::ContactSurface::id_M() and geometry::ContactSurface::id_N().
   @param[in] F_Ac_W Spatial force applied on body A, at contact surface
     centroid C, and expressed in the world frame W. The position `p_WC` of C in
     the world frame W can be obtained with
     `ContactSurface::centroid()`.
   @param[in] quadrature_point_data Hydroelastic field data at each quadrature
     point. Data must be provided in accordance to the convention that geometry
     M and N are attached to bodies A and B, respectively. Refer to
     HydroelasticQuadraturePointData for further details. */
  HydroelasticContactInfo(
      const geometry::ContactSurface<T>* contact_surface,
      const SpatialForce<T>& F_Ac_W,
      std::vector<HydroelasticQuadraturePointData<T>>&& quadrature_point_data)
      : contact_surface_(std::shared_ptr<void>(), contact_surface),
        F_Ac_W_(F_Ac_W),
        quadrature_point_data_(std::move(quadrature_point_data)) {
    DRAKE_DEMAND(contact_surface_ != nullptr);
  }

  /** This constructor takes ownership of `contact_surface` via a
   std::unique_ptr, instead of aliasing a pre-existing contact surface. In all
   other respects, it is identical to the @ref
   hydro_contact_info_non_owning_ctor "other overload" that takes
   `contact_surface` by raw pointer. */
  HydroelasticContactInfo(
      std::unique_ptr<geometry::ContactSurface<T>> contact_surface,
      const SpatialForce<T>& F_Ac_W,
      std::vector<HydroelasticQuadraturePointData<T>>&& quadrature_point_data)
      : contact_surface_(std::move(contact_surface)),
        F_Ac_W_(F_Ac_W),
        quadrature_point_data_(std::move(quadrature_point_data)) {
    DRAKE_DEMAND(contact_surface_ != nullptr);
  }
  // @}

  /** Returns a reference to the ContactSurface data structure. Note that
   the mesh and gradient vector fields are expressed in the world frame.
   @pre This object has not been moved-from. */
  const geometry::ContactSurface<T>& contact_surface() const {
    return *contact_surface_;
  }

  /** Gets the intermediate data, including tractions, computed by the
   quadrature process. */
  const std::vector<HydroelasticQuadraturePointData<T>>& quadrature_point_data()
      const {
    return quadrature_point_data_;
  }

  /** Gets the spatial force applied on body A, at the centroid point C of the
   surface mesh M, and expressed in the world frame W. The position `p_WC` of
   the centroid point C in the world frame W can be obtained with
   `contact_surface().centroid()`. */
  const SpatialForce<T>& F_Ac_W() const { return F_Ac_W_; }

 private:
  // Note that the mesh of the contact surface is defined in the world frame.
  std::shared_ptr<const geometry::ContactSurface<T>> contact_surface_;

  // The spatial force applied at the centroid (Point C) of the surface mesh.
  SpatialForce<T> F_Ac_W_;

  // The traction and slip velocity evaluated at each quadrature point.
  std::vector<HydroelasticQuadraturePointData<T>> quadrature_point_data_;
};

/** ... */
template <>
class HydroelasticContactInfo<symbolic::Expression> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HydroelasticContactInfo);
  HydroelasticContactInfo() = default;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticContactInfo)
