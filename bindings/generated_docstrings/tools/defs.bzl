load("//tools/skylark:drake_cc.bzl", "drake_cc_library")

def generate_docstrings(*, subdir):
    """Given a subdir name like "multibody/tree", declares a cc_library named
    ":multibody_tree" that contains "multibody_tree.h" (committed to git), as
    well as a rule ":gen_multibody_tree" that outputs the "gen/multibody_tree.h"
    generated file. The two header files (committed vs generated) are checked
    for equality by the `:diff_test:` in our neighboring `BUILD.bazel` file.
    """
    identifier = subdir.replace("/", "_")
    filename = "{}.h".format(identifier)

    # Declare the committed docstrings.
    drake_cc_library(
        name = identifier,
        hdrs = [filename],
        declare_installed_headers = False,
        tags = ["nolint"],
        visibility = ["//bindings:__subpackages__"],
    )

    # Generate the reference docstrings.
    generate_header(
        name = "gen_{}".format(identifier),
        out = "gen/{}".format(filename),
        hdr_subdir = "drake/{}".format(subdir),
        exclude_hdr_patterns = [
            # Anonymous namespace and deduction guides both confuse pybind.
            "drake/common/overloaded.h",
            # These headers are deprecated for removal on 2026-07-01, and are
            # conditionally omitted when the use_eigen_legacy_autodiff is off.
            # To avoid confusing the docstrings diff_test, we'll omit them from
            # generation entirely, since we don't need any of their docs anyway.
            "drake/common/autodiff_overloads.h",
            "drake/common/autodiffxd.h",
            "drake/common/eigen_autodiff_types.h",
        ],
    )

def _generate_header_impl(ctx):
    inputs = [
        ctx.file._doxygen_xml,
    ]
    outputs = [
        ctx.outputs.out,
    ]
    args = ctx.actions.args()
    args.add("--input=" + inputs[0].path)
    args.add("--output=" + outputs[0].path)
    args.add("--hdr_subdir=" + ctx.attr.hdr_subdir)
    for p in ctx.attr.exclude_hdr_patterns:
        args.add("--exclude_hdr_patterns=" + p)
    ctx.actions.run(
        inputs = inputs,
        outputs = outputs,
        arguments = [args],
        executable = ctx.executable._generate,
    )

generate_header = rule(
    attrs = {
        "_generate": attr.label(
            default = Label("//bindings/generated_docstrings:generate"),
            allow_files = True,
            cfg = "host",
            executable = True,
        ),
        "_doxygen_xml": attr.label(
            default = Label("//doc/doxygen_cxx:doxygen_xml"),
            allow_single_file = True,
        ),
        "hdr_subdir": attr.string(mandatory = True),
        "out": attr.output(mandatory = True),
        "exclude_hdr_patterns": attr.string_list(),
    },
    implementation = _generate_header_impl,
    output_to_genfiles = True,
)
