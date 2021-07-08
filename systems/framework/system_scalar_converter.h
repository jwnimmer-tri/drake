#pragma once

#include <memory>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/system_type_tag.h"

namespace drake {
namespace systems {

class SystemBase;
template <typename T> class System;

/// Helper class to convert a System<U> into a System<T>, intended for internal
/// use by the System framework, not directly by users.
///
/// For user-facing documentation see @ref system_scalar_conversion.
///
/// Because it is not templated on a System subclass, this class can be used by
/// LeafSystem without any direct knowledge of the subtypes being converted.
/// In other words, it enables a runtime flavor of the CRTP.
///
/// Throughout this class, the following template naming convention applies:
///
/// @tparam S is the System subclass that this object will convert from and to.
/// @tparam U the source scalar type (to convert from), which must be one of
///         the @ref default_scalars "default scalars".
/// @tparam T the resulting scalar type (to convert into), which must be one of
///         the @ref default_scalars "default scalars".
///
/// @note Conversions where `T` and `U` types are the same are not supported.
/// Template functions such as IsConvertible<T, U>() are still callable, but
/// will always return false, null, etc.
class SystemScalarConverter {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemScalarConverter);

  /// (Advanced) Creates a converter that supports no conversions.  The single-
  /// argument constructor below is the overload intended for users.
  SystemScalarConverter();

  /// Creates a converter that uses S's scalar-converting copy constructor to
  /// perform system scalar conversion. That constructor takes the form of:
  ///
  /// @code
  /// template <typename T>
  /// class FooSystem final : public LeafSystem<T> {
  ///   template <typename U>
  ///   explicit FooSystem(const FooSystem<U>& other);
  /// };
  /// @endcode
  ///
  /// By default, the converter supports conversions to and from all of the
  /// @ref default_scalars "default scalars", but systems may specialize the
  /// scalar_conversion::Traits to disable support for some or all of these
  /// conversions.  Conversions where `T` and `U` types are the same are not
  /// supported.
  ///
  /// This an implicit conversion constructor (not marked `explicit`), in order
  /// to make calling code substantially more readable, with relatively little
  /// risk of an unwanted accidental conversion happening.
  ///
  /// See @ref system_scalar_conversion for additional overview documentation.
  template <template <typename> class S>
  // NOLINTNEXTLINE(runtime/explicit)
  SystemScalarConverter(SystemTypeTag<S>) {
    AddConstructors<true, S>();
  }

  enum class DRAKE_DEPRECATED("2021-11-01",
      "Use MakeWithoutSubtypeChecking instead of kDisabled.")
  GuaranteedSubtypePreservation {
    /// The argument to Convert must be of the exact type S that was used to
    /// populate the SystemScalarConverter.
    kEnabled,
    /// The argument to Convert need not be the exact type S that was used to
    /// populate the SystemScalarConverter -- it can be either exactly that S,
    /// or a subtype of that S.  This permits subtype information to be lost
    /// across conversion.
    kDisabled,
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  template <template <typename> class S>
  DRAKE_DEPRECATED("2021-11-01",
      "Use MakeWithoutSubtypeChecking instead of kDisabled.")
  SystemScalarConverter(SystemTypeTag<S>, GuaranteedSubtypePreservation sub) {
    if (sub == GuaranteedSubtypePreservation::kEnabled) {
      AddConstructors<true, S>();
    } else {
      AddConstructors<false, S>();
    }
  }
#pragma GCC diagnostic pop

  /// (Advanced) Creates a converter similar to the single-argument constructor,
  /// with the built-in checks for guaranteed subtype preservation of the System
  /// turned off.  In general, subtype preservation is an important invariant of
  /// scalar conversion, so be extremely cautious about disabling it.
  template <template <typename> class S>
  static SystemScalarConverter MakeWithoutSubtypeChecking() {
    SystemScalarConverter result;
    result.AddConstructors<false, S>();
    return result;
  }

  ~SystemScalarConverter();

  /// Returns true iff no conversions are supported.  (In other words, whether
  /// this is a default-constructed object.)
  bool empty() const;

  template <typename T, typename U>
  using ConverterFunction
      DRAKE_DEPRECATED("2021-10-01",
      "Only scalar-converting copy constructors are supported.")
      = std::function<std::unique_ptr<System<T>>(const System<U>&)>;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  template <typename T, typename U>
  DRAKE_DEPRECATED("2021-10-01",
      "Only scalar-converting copy constructors are supported.")
  void Add(const ConverterFunction<T, U>&);
#pragma GCC diagnostic pop

  template <template <typename> class S, typename T, typename U>
  DRAKE_DEPRECATED("2021-10-01",
      "User-defined scalar types cannot be added.")
  void AddIfSupported() {
    AddIfSupported<S, T, U>(GuaranteedSubtypePreservation::kEnabled);
  }

  /// Removes from this converter all pairs where `other.IsConvertible<T, U>`
  /// is false.  The subtype `S` need not be the same between this and `other`.
  void RemoveUnlessAlsoSupportedBy(const SystemScalarConverter& other);

  /// Removes from this converter the ability to convert from System<U> to
  /// System<T>.
  template <typename T, typename U>
  void Remove();

  /// Returns true iff this object can convert a System<U> into a System<T>,
  /// i.e., whether Convert() will return non-null.
  template <typename T, typename U>
  bool IsConvertible() const;

  /// Converts a System<U> into a System<T>.  This is the API that LeafSystem
  /// uses to provide a default implementation of DoToAutoDiffXd, etc.
  template <typename T, typename U>
  std::unique_ptr<System<T>> Convert(const System<U>& other) const {
    // This function definition must appear in the header, because it returns a
    // unique_ptr to a forward-declared class.  Only callers that have already
    // included system.h to define SystemBase and System may call this function.
    return static_pointer_cast<System<T>>(
        Convert({typeid(T), typeid(U)}, other));
  }

 private:
  // Like ConverterFunc, but with the args and return value decayed into void*.
  using ErasedConverterFunc = std::function<void*(const void*)>;

  // A pair of types {T, U}, usable as an unordered_map key.
  struct Key : std::pair<std::type_index, std::type_index> {
    Key(const std::type_info&, const std::type_info&);
  };
  struct KeyHasher {
    size_t operator()(const Key&) const;
  };

  // Calls MaybeAddConstructor for all supported pairs of scalar types.
  template <bool subtype_preservation, template <typename> class S>
  void AddConstructors() {
    // N.B. When changing the pairs of supported types below, be sure to also
    // change the `ConversionPairs` type pack in `DefineFrameworkPySystems`
    // and the factory function immediately below.
    using Expression = symbolic::Expression;
    // From double to all other types.
    MaybeAddConstructor<subtype_preservation, S, AutoDiffXd, double>();
    MaybeAddConstructor<subtype_preservation, S, Expression, double>();
    // From AutoDiffXd to all other types.
    MaybeAddConstructor<subtype_preservation, S, double, AutoDiffXd>();
    MaybeAddConstructor<subtype_preservation, S, Expression, AutoDiffXd>();
    // From Expression to all other types.
    MaybeAddConstructor<subtype_preservation, S, double, Expression>();
    MaybeAddConstructor<subtype_preservation, S, AutoDiffXd, Expression>();
  }

  // Adds a converter for an S<U> into an S<T> using S's scalar-converting copy
  // constructor, unless the traits have disabled the conversion.
  template <bool subtype_preservation,
            template <typename> class S, typename T, typename U>
  void MaybeAddConstructor();

  void Insert(Key, ErasedConverterFunc);

  std::unique_ptr<SystemBase> Convert(const Key&, const SystemBase&) const;

  // Throws an exception that `other` cannot be converted from S<U> to S<T>.
  [[noreturn]] static void ThrowConversionMismatch(
      const std::type_info& s_t_info,
      const std::type_info& s_u_info,
      const std::type_info& other_info);

  // Maps from {T, U} to the function that converts from U into T.
  std::unordered_map<Key, ErasedConverterFunc, KeyHasher> funcs_;
};

#if !defined(DRAKE_DOXYGEN_CXX)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// (This function is deprecated.)
template <typename T, typename U>
void SystemScalarConverter::Add(const ConverterFunction<T, U>& func) {
  // Make sure func contains a target (i.e., is not null-ish).
  DRAKE_ASSERT(static_cast<bool>(func));
  // Copy `func` into a lambda that ends up stored into `funcs_`.  The lambda
  // is typed as `void* => void*` in order to have a non-templated signature
  // and thus fit into a homogeneously-typed std::unordered_map.
  Insert(typeid(T), typeid(U), [func](const void* const bare_u) {
    DRAKE_ASSERT(bare_u);
    const System<U>& other = *static_cast<const System<U>*>(bare_u);
    return func(other).release();
  });
}
#pragma GCC diagnostic pop

// N.B. The pydrake method TemplateSystem._make re-implements this logic in the
// file scalar_conversion.py.  Be sure to keep both in sync.
template <
  bool enable_subtype_preservation,
  template <typename> class S, typename T, typename U>
void SystemScalarConverter::MaybeAddConstructor() {
  using Traits = typename scalar_conversion::Traits<S>;
  if constexpr (Traits::template supported<T, U>::value) {
    Insert({typeid(T), typeid(U)}, [](const void* const bare_u) {
      DRAKE_ASSERT(bare_u != nullptr);
      const System<U>& other = *static_cast<const System<U>*>(bare_u);
      // Confirm the runtime type of `other`.
      const S<U>* typed_other;
      if constexpr (enable_subtype_preservation) {
        // Require an exact match to S.
        const std::type_info& other_info = typeid(other);
        if (other_info != typeid(S<U>)) {
          ThrowConversionMismatch(typeid(S<T>), typeid(S<U>), other_info);
        }
        typed_other = static_cast<const S<U>*>(&other);
      } else {
        // Allow for any subclass of S.
        typed_other = dynamic_cast<const S<U>*>(&other);
        if (typed_other == nullptr) {
          const std::type_info& other_info = typeid(other);
          ThrowConversionMismatch(typeid(S<T>), typeid(S<U>), other_info);
        }
      }
      // Call the scalar-converting copy constructor.
      return static_cast<void*>(new S<T>(*typed_other));
    });
  }
}

#endif  // DRAKE_DOXYGEN_CXX

}  // namespace systems
}  // namespace drake
