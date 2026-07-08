load("@with_cfg.bzl", "with_cfg")
load("//tools/skylark:cc.bzl", "cc_library")

# XXX
# absl_cc_library is a cc_library rule that recompiles the libraries
# listed in its `deps` using only
# static linking.
_builder = with_cfg(cc_library)
_builder.extend("features", ["-supports_dynamic_linker"])
_builder.set(Label("@module_abseil//absl/base:ABSL_OPTION_USE_INLINE_NAMESPACE"), "1")
_builder.set(Label("@module_abseil//absl/base:ABSL_OPTION_INLINE_NAMESPACE_NAME"), "drake_vendor")
absl_cc_library, _ = _builder.build()
