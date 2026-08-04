load("@with_cfg.bzl", "with_cfg")
load("//tools/skylark:cc.bzl", "cc_library")

# cc_custom_library is a cc_library rule that recompiles the LCM cc_library
# listed in its deps using a specific string_flag set for Drake vendoring.
_builder = with_cfg(cc_library)
_builder.set(Label("@module_lcm//lcm:LCM_C_NAMESPACE"), "drake_vendor_lcm")
cc_custom_library, _ = _builder.build()
