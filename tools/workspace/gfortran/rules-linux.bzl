def fortran_library(
        name,
        srcs = [],
        srcs_mods = [],
        hdrs = [],
        linkopts = [],
        deps = [],
        **kwargs):
    """Compiles a Fortran library.  This library's symbols will have hidden
    visibility, becaused Drake binary release artifacts should never provide
    them as part of the public API.

    srcs: fortran source files (e.g., `*.f`).
    linkopts: are passed through to the bazel cc_library result.
    deps: are passed through to the bazel cc_library result.
    kwargs: are passed through to the bazel cc_library result.
    """

    # Compile *.f* files to *.pic.o files.
    compiler = "@gfortran//:compiler"
    objs = []
    for i, src in enumerate(srcs):
        obj = src + ".pic.o"
        objs.append(obj)
        outs = [obj]
        if srcs_mods:
            mod = srcs_mods[i]
            outs.append(mod)
        native.genrule(
            name = obj + "_genrule",
            srcs = [src] + hdrs,
            outs = outs,
            tools = [compiler],
            cmd = "$(location {}) -fopenmp -fPIC -c $(location {}) -o $(location {})".format(
                compiler,
                src,
                obj,
            ) + (
                " -J$$(dirname $(location {}))".format(obj)
            ) + " ".join([
                "-I$$(dirname $(location {}))".format(hdr)
                for hdr in hdrs
            ]),
            visibility = ["//visibility:private"],
        )

    # Consolidate object code into an archive, so that we are able to tell the
    # linker to use hidden visibility for it.
    libname = "lib{}_{}.pic.a".format(
        native.repository_name()[1:],  # (Drop the leading @.)
        name,
    )
    native.genrule(
        name = name + "_ar_genrule",
        srcs = objs,
        outs = [libname],
        cmd = "$(AR) qcD $@ $(SRCS)",
        toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
        visibility = ["//visibility:private"],
    )

    # Wrap the *.pic.a file into a cc_library, so that we can attach
    # linkopts and deps.
    native.cc_library(
        name = name,
        srcs = [libname],
        linkopts = linkopts + ["-Wl,--exclude-libs=" + libname],
        deps = deps + ["@gfortran//:runtime"],
        **kwargs
    )
