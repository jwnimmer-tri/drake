load("@cc//:compiler.bzl", "COMPILER_ID", "COMPILER_VERSION_MAJOR")
load("//tools/skylark:cc.bzl", "cc_binary", "cc_library", "cc_test")
load(
    "//tools/skylark:kwargs.bzl",
    "amend",
    "incorporate_allow_network",
    "incorporate_num_threads",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

# The CXX_FLAGS will be enabled for all C++ rules in the project
# building with any compiler.
CXX_FLAGS = [
    "-Werror=all",
    "-Werror=attributes",
    "-Werror=cpp",
    "-Werror=deprecated",
    "-Werror=deprecated-declarations",
    "-Werror=ignored-qualifiers",
    "-Werror=old-style-cast",
    "-Werror=overloaded-virtual",
    "-Werror=shadow",
    "-Werror=unused-result",
    # We eschew Eigen::IO in lieu of drake::fmt_eigen.
    # See drake/common/fmt_eigen.h for details.
    "-DEIGEN_NO_IO=1",
]

# The CLANG_FLAGS will be enabled for all C++ rules in the project when
# building with clang (excluding the Apple LLVM compiler see APPLECLANG_FLAGS
# below).
CLANG_FLAGS = CXX_FLAGS + [
    "-Werror=absolute-value",
    "-Werror=c99-designator",
    "-Werror=inconsistent-missing-override",
    "-Werror=final-dtor-non-final-class",
    "-Werror=literal-conversion",
    "-Werror=non-virtual-dtor",
    "-Werror=range-loop-analysis",
    "-Werror=return-stack-address",
    "-Werror=sign-compare",
]

# The CLANG_VERSION_SPECIFIC_FLAGS will be enabled for all C++ rules in the
# project when building with a Clang compiler of the specified major
# version (excluding the Apple LLVM compiler, see
# APPLECLANG_VERSION_SPECIFIC_FLAGS below).
CLANG_VERSION_SPECIFIC_FLAGS = {
}

# The APPLECLANG_FLAGS will be enabled for all C++ rules in the project when
# building with the Apple LLVM compiler.
APPLECLANG_FLAGS = CLANG_FLAGS

# The APPLECLANG_VERSION_SPECIFIC_FLAGS will be enabled for all C++ rules in
# the project when building with an Apple LLVM compiler of the specified major
# version.
APPLECLANG_VERSION_SPECIFIC_FLAGS = {
}

# The GCC_FLAGS will be enabled for all C++ rules in the project when
# building with gcc.
GCC_FLAGS = CXX_FLAGS + [
    "-Werror=extra",
    "-Werror=logical-op",
    "-Werror=non-virtual-dtor",
    "-Werror=return-local-addr",
    "-Werror=unused-but-set-parameter",
    # This was turned on via -Wextra, but is too strict to have as an error.
    "-Wno-missing-field-initializers",
]

# The GCC_CC_TEST_FLAGS will be enabled for all cc_test rules in the project
# when building with gcc.
GCC_CC_TEST_FLAGS = [
    "-Wno-unused-parameter",
]

GCC_VERSION_SPECIFIC_FLAGS = {
    13: [
        "-Werror=pessimizing-move",
        # TODO(#21337) Investigate and resolve what to do about these warnings
        # long-term. Some seem like true positives (i.e., bugs in Drake).
        "-Wno-array-bounds",
        "-Wno-dangling-reference",
        "-Wno-maybe-uninitialized",
        "-Wno-stringop-overflow",
        "-Wno-stringop-overread",
        "-Wno-uninitialized",
    ],
}

def _amend_copts(kwargs, *, cc_test = False):
    """Given the kwargs from a drake_cc_foo macro, returns a modified copy of
    those kwargs with updated copts:

    - Merges the SCREAM_CAPS baseline options from the above constants into
      copts, based on which compiler is currently being used. Note that which
      SCREAM_CAPS constants are used is affeted by cc_test.

    - Merges the rule's acute gcc_copts (if any) or clang_copts (if any) into
      copts, based on which  compiler is currently being used.

    The cc_test flag should only be set to True only from drake_cc_test rules
    (or equivalent).
    """
    kwargs = dict(kwargs)
    rule_copts = kwargs.pop("copts", [])
    rule_gcc_copts = kwargs.pop("gcc_copts", [])
    rule_clang_copts = kwargs.pop("clang_copts", [])
    if COMPILER_ID == "AppleClang":
        copts = APPLECLANG_FLAGS + rule_copts + rule_clang_copts
        if COMPILER_VERSION_MAJOR in APPLECLANG_VERSION_SPECIFIC_FLAGS:
            copts += APPLECLANG_VERSION_SPECIFIC_FLAGS[COMPILER_VERSION_MAJOR]
    elif COMPILER_ID == "Clang":
        copts = CLANG_FLAGS + rule_copts + rule_clang_copts
        if COMPILER_VERSION_MAJOR in CLANG_VERSION_SPECIFIC_FLAGS:
            copts += CLANG_VERSION_SPECIFIC_FLAGS[COMPILER_VERSION_MAJOR]
    elif COMPILER_ID == "GNU":
        extra_gcc_flags = GCC_CC_TEST_FLAGS if cc_test else []
        copts = GCC_FLAGS + extra_gcc_flags + rule_copts + rule_gcc_copts
        if COMPILER_VERSION_MAJOR in GCC_VERSION_SPECIFIC_FLAGS:
            copts += GCC_VERSION_SPECIFIC_FLAGS[COMPILER_VERSION_MAJOR]
    else:
        copts = rule_copts

    # We can't handle select() yet.
    # TODO(jwnimmer-tri) We should handle select.
    if type(copts) == "list":
        # Disable Werror unless drake_werror is turned on.
        copts = select({
            "//tools:drake_werror": copts,
            "//conditions:default": [
                x.replace("-Werror=", "-W")
                for x in copts
            ],
        })

    kwargs["copts"] = copts
    return kwargs

def _check_library_deps_blacklist(name, deps):
    """Report an error if a library should not use something from deps."""
    if not deps:
        return
    if type(deps) != "list":
        # We can't handle select() yet.
        # TODO(jwnimmer-tri) We should handle select.
        return
    for dep in deps:
        if name == "drake_cc_googletest_main":
            # This library-with-main is a special case.
            continue
        if dep.endswith(":add_text_logging_gflags"):
            fail("The cc_library '" + name + "' must not depend on " +
                 "//common:add_text_logging_gflags; only cc_binary targets " +
                 "are allowed to have gflags")
        if dep.endswith(":main"):
            fail("The cc_library '" + name + "' must not depend on a :main " +
                 "function from a cc_library; only cc_binary program should " +
                 "have a main function")

def _prune_private_hdrs(srcs):
    """Returns (new_srcs, private_hdrs), where .h files have been split out of
    srcs into private_hdrs, leaving new_srcs remaining.
    """
    if type(srcs) == "select":
        # We can't handle select() yet.
        # TODO(jwnimmer-tri) We should handle select.
        return srcs, []
    private_hdrs = [x for x in srcs if x.endswith(".h")]
    if private_hdrs:
        srcs = [x for x in srcs if x not in private_hdrs]
    return srcs, private_hdrs

def installed_headers_for_dep(dep):
    """Convert a cc_library label to a DrakeCc provider label.  Given a label
    `dep` for a cc_library, such as would be found in the `deps = []` of
    some cc_library, returns the corresponding label for the matching DrakeCc
    provider associated with that library.  The returned label is appropriate
    to use in the deps of a `drake_installed_headers()` rule.

    Once our rules are better able to call native rules like native cc_binary,
    instead of having two labels we would prefer to tack a DrakeCc provider
    onto the cc_library target directly.

    Related links from upstream:
    https://github.com/bazelbuild/bazel/issues/2163
    https://docs.bazel.build/versions/master/skylark/cookbook.html#macro-multiple-rules
    """
    suffix = ".installed_headers"
    if ":" in dep:
        # The label is already fully spelled out; just tack on our suffix.
        result = dep + suffix
    else:
        # The label is the form //foo/bar which means //foo/bar:bar.
        last_slash = dep.rindex("/")
        libname = dep[last_slash + 1:]
        result = dep + ":" + libname + suffix
    return result

def installed_headers_for_drake_deps(deps):
    """Filters `deps` to find drake labels (i.e., discard third_party labels),
    and then maps `installed_headers_for_dep()` over that list of drake deps.

    (Absolute paths to Drake's lcmtypes headers are also filtered out, because
    LCM headers follow a different #include convention, and so are installed
    separately.  Refer to drake/lcmtypes/BUILD.bazel for details.  Note that
    within-package paths are left unchanged, so that this macro can still be
    used within Drake's lcmtypes folder.)

    This is useful for computing the deps of a `drake_installed_headers()` rule
    from the deps of a `cc_library()` rule.
    """
    if type(deps) == "select":
        # We can't handle select() yet.
        # TODO(jwnimmer-tri) We should handle select.
        return []
    return [
        installed_headers_for_dep(x)
        for x in deps
        if (
            not x.startswith("@") and
            not x.startswith("//drake/lcmtypes:") and
            not x == "//:drake_shared_library" and
            not x.startswith("//third_party")
        )
    ]

# A provider to collect Drake metadata about C++ rules.  For background, see
# https://docs.bazel.build/versions/master/skylark/rules.html#providers.
DrakeCc = provider()

def _drake_installed_headers_impl(ctx):
    hdrs = list(ctx.files.hdrs)
    for x in ctx.files.hdrs_exclude:
        hdrs.remove(x)
    transitive_hdrs = depset(hdrs, transitive = [
        dep[DrakeCc].transitive_hdrs
        for dep in ctx.attr.deps
    ])
    return [
        DrakeCc(
            transitive_hdrs = transitive_hdrs,
        ),
    ]

"""Declares a rule to provide DrakeCc information about headers that should be
installed.  We use this instead of the built-in `cc` provider so that we can
adjust and filter what is going to be installed, versus everything that is
required to compile.
"""

drake_installed_headers = rule(
    attrs = {
        "hdrs": attr.label_list(
            mandatory = True,
            allow_files = True,
        ),
        "hdrs_exclude": attr.label_list(
            allow_files = True,
        ),
        "deps": attr.label_list(
            mandatory = True,
            providers = [DrakeCc],
        ),
    },
    implementation = _drake_installed_headers_impl,
)

def _path_startswith_match(path, only_startswith, never_startswith):
    # Ignore some leading path elements.  These will happen if Drake is
    # consumed as an external.
    strip = "../drake/"
    if path.startswith(strip):
        path = path[len(strip):]

    # Returns true iff `path` is consistent with the given `only...` and
    # `never...` prefixes.  Omitting either or both of the `...startswith`
    # arguments is treated as a pass (true) by default.
    if only_startswith:
        if not path.startswith(only_startswith):
            return False
    for prefix in never_startswith:
        if path.startswith(prefix):
            return False
    return True

def _gather_transitive_hdrs_impl(ctx):
    # Transitively list all headers.
    all_hdrs = depset([], transitive = [
        dep[DrakeCc].transitive_hdrs
        for dep in ctx.attr.deps
    ])

    # Filter in/out items matching a prefix.
    result = depset([
        x
        for x in all_hdrs.to_list()
        if _path_startswith_match(
            x.short_path,
            ctx.attr.only_startswith,
            ctx.attr.never_startswith,
        )
    ])

    return struct(files = result)

_gather_transitive_hdrs = rule(
    attrs = {
        "deps": attr.label_list(
            allow_files = False,
            providers = [DrakeCc],
        ),
        "only_startswith": attr.string(),
        "never_startswith": attr.string_list(),
    },
    implementation = _gather_transitive_hdrs_impl,
)

def drake_transitive_installed_hdrs_filegroup(
        name,
        deps = [],
        only_startswith = None,
        never_startswith = [],
        **kwargs):
    """Declare a filegroup that contains the transtive installed hdrs of the
    targets named by `deps`.
    """
    _gather_transitive_hdrs(
        name = name + "_gather",
        deps = [installed_headers_for_dep(x) for x in deps],
        visibility = [],
        only_startswith = only_startswith,
        never_startswith = never_startswith,
    )
    native.filegroup(
        name = name,
        srcs = [":" + name + "_gather"],
        **kwargs
    )

def _cc_linkonly_library_impl(ctx):
    deps_cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    return [
        DefaultInfo(
            runfiles = ctx.runfiles(
                collect_data = True,
                collect_default = True,
            ),
        ),
        CcInfo(
            compilation_context = None,
            linking_context = deps_cc_infos.linking_context,
        ),
    ]

cc_linkonly_library = rule(
    implementation = _cc_linkonly_library_impl,
    doc = """
Links the given dependencies but discards the entire compilation context,
i.e., include paths and preprocessor definitions.
""",
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
    },
    fragments = ["cpp"],
)

def _raw_drake_cc_library(
        name,
        srcs = [],  # Cannot list any headers here.
        hdrs = None,
        textual_hdrs = None,
        strip_include_prefix = None,
        include_prefix = None,
        copts = None,
        defines = None,
        data = None,
        interface_deps = [],
        implementation_deps = [],
        linkstatic = None,
        linkopts = None,
        alwayslink = None,
        tags = None,
        testonly = None,
        visibility = None,
        declare_installed_headers = None,
        install_hdrs_exclude = None,
        deprecation = None):
    """Creates a rule to declare a C++ library.  Uses Drake's include_prefix
    and checks the deps blacklist.  If declare_installed_headers is true, also
    adds a drake_installed_headers() target.  (This should be set if and only
    if the caller is drake_cc_library.)
    """
    _check_library_deps_blacklist(name, interface_deps)
    _check_library_deps_blacklist(name, implementation_deps)
    _, private_hdrs = _prune_private_hdrs(srcs)
    if private_hdrs:
        fail("private_hdrs = " + private_hdrs)

    # Require include paths like "drake/foo/bar.h", not "foo/bar.h".
    if strip_include_prefix == None:
        strip_include_prefix = "/"
    if include_prefix == None:
        include_prefix = "drake"

    # Add the installed header tracking, unless we've opted-out.
    if declare_installed_headers:
        drake_installed_headers(
            name = name + ".installed_headers",
            hdrs = hdrs,
            hdrs_exclude = install_hdrs_exclude,
            deps = installed_headers_for_drake_deps(interface_deps),
            tags = ["nolint"],
            visibility = ["//visibility:public"],
        )

    # If we're using implementation_deps, then the result of compiling our srcs
    # needs to use an intermediate label name. The actual `name` label will be
    # used for the "implementation sandwich", below.
    # TODO(jwnimmer-tri) Once https://github.com/bazelbuild/bazel/issues/12350
    # is fixed and Bazel offers interface_deps natively, then we can switch to
    # that implementation instead of making our own sandwich.
    compiled_name = name
    compiled_visibility = visibility
    compiled_deprecation = deprecation
    if implementation_deps:
        if not linkstatic:
            fail("implementation_deps are only supported for static libraries")
        compiled_name = "_{}_compiled_cc_impl".format(name)
        compiled_visibility = ["//visibility:private"]
    cc_library(
        name = compiled_name,
        srcs = srcs,
        hdrs = hdrs,
        textual_hdrs = textual_hdrs,
        strip_include_prefix = strip_include_prefix,
        include_prefix = include_prefix,
        copts = copts,
        defines = defines,
        data = data,
        deps = interface_deps + implementation_deps,
        linkstatic = linkstatic,
        linkopts = linkopts,
        alwayslink = alwayslink,
        tags = tags,
        testonly = testonly,
        visibility = compiled_visibility,
        deprecation = compiled_deprecation,
    )

    # If we're using implementation_deps, then make me an "implementation
    # sandwich".  Create one library with our headers, one library with only
    # our static archive, and then squash them together to the final result.
    if implementation_deps:
        headers_name = "_{}_headers_cc_impl".format(name)
        cc_library(
            name = headers_name,
            hdrs = hdrs,
            strip_include_prefix = strip_include_prefix,
            include_prefix = include_prefix,
            defines = defines,
            deps = interface_deps,  # N.B. No implementation_deps!
            linkstatic = 1,
            tags = tags,
            testonly = testonly,
            visibility = ["//visibility:private"],
        )
        archive_name = "_{}_archive_cc_impl".format(name)
        cc_linkonly_library(
            name = archive_name,
            deps = [":" + compiled_name],
            visibility = ["//visibility:private"],
            tags = tags,
        )
        cc_library(
            name = name,
            deps = [
                ":" + headers_name,
                ":" + archive_name,
            ],
            linkstatic = 1,
            tags = tags,
            testonly = testonly,
            visibility = visibility,
            deprecation = deprecation,
        )

def _amend_private_hdrs(
        kwargs,
        *,
        base_name):
    """XXX
    Given some srcs, prunes any header files into a separate cc_library, and
    appends that new library to deps, returning new_srcs (sans headers) and
    add_deps.  The separate cc_library is private with linkstatic = 1.

    We use this helper in all drake_cc_{library,binary,test) because when we
    want to fiddle with include paths, we *must* have all header files listed
    as hdrs; the include_prefix does not apply to srcs.
    """
    new_srcs, private_hdrs = _prune_private_hdrs(kwargs.get("srcs", []))
    if not private_hdrs:
        return kwargs

    # Create the separate cc_library.
    private_name = "_" + base_name + "_private_headers_cc_impl"
    private_kwargs = dict(kwargs)
    private_kwargs["hdrs"] = private_hdrs
    private_kwargs["linkstatic"] = True
    private_kwargs["visibility"] = ["//visibility:private"]
    private_kwargs = amend(
        private_kwargs,
        "interface_deps",
        append = private_kwargs.pop("deps", []),
    )
    private_kwargs.pop("env", None)
    private_kwargs.pop("linkshared", None)
    private_kwargs.pop("size", None)
    private_kwargs.pop("srcs", None)
    _raw_drake_cc_library(
        name = private_name,
        **private_kwargs
    )
    return amend(kwargs, "implementation_deps", append = [":" + private_name])

def drake_cc_library(
        name,
        *,
        interface_deps = None,
        internal = False,
        declare_installed_headers = 1,
        install_hdrs_exclude = [],
        **kwargs):
    """Creates a rule to declare a C++ library.

    By default, we produce only static libraries, to reduce compilation time
    on all platforms, and to avoid mysterious dyld errors on OS X. This default
    could be revisited if binary size becomes a concern.

    The dependencies of a cc_library can be classified as "interface deps" or
    "implementation deps". Files that are #include'd by headers are "interface
    deps"; files that are #include'd by srcs are "implementation deps".

    Dependencies listed as "implementation deps" will be linked into your
    program, but their header files will only be available to the srcs files
    here. Downstream users of this library will not have access to the
    implementation deps's header files, nor will any preprocessor definitions
    for the implementation deps propagate past this firewall.

    For backwards compatibility, when the `interface_deps=` argument is None,
    we treat `deps=` as the list of "interface deps", with no "implementation
    deps". This means that simply listing out everything as `deps=` will build
    successfully, even if only the cc file needed it.

    When `interface_deps=` is non-None, then it describes the "interface deps"
    and the `deps=` argument is interpreted as the "implementation deps".

    The dependencies of a drake_cc_library must be another drake_cc_library, or
    else be named like "@something//etc..." (i.e., come from the workspace, not
    part of Drake).  In other words, all of Drake's C++ libraries must be
    declared using the drake_cc_library macro.

    Setting `internal = True` is convenient sugar to flag code that is never
    used by Drake's installed headers. This is especially helpful when the
    header_lint tool is complaining about third-party dependency pollution.
    Flagging a library as `internal = True` means that the library is internal
    from the point of view of the build system, which is an even stronger
    promise that merely placing code inside `namespace internal {}`.
    Code that is build-system internal should always be namespace-internal,
    but not all namespace-internal code is build-system internal. For example,
    drake/common/diagnostic_policy.h is namespace-internal, but cannot be
    build-internal because other Drake headers (multibody/parsing/parser.h)
    refer to the diagnostic policy header, for use by their private fields.
    Libraries marked with `internal = True` should generally be listed only as
    deps of _other_ libraries marked as internal, or as "implementation deps"
    (see paragraphs above) of non-internal libraries.
    """
    if "implementation_deps" in kwargs:
        fail("Invalid argument")
    kwargs = _amend_copts(kwargs)
    if interface_deps != None:
        kwargs["interface_deps"] = interface_deps
        kwargs["implementation_deps"] = kwargs.pop("deps", [])
    else:
        kwargs["interface_deps"] = kwargs.pop("deps", [])
        kwargs["implementation_deps"] = []

        hdrs = [],
        srcs = [],
        linkstatic = 1,


    if internal:
        if install_hdrs_exclude != []:
            fail("When using internal = True, hdrs are automatically excluded")
        if kwargs.get("testonly", False):
            fail("Using internal = True is already implied under testonly = False")
        if len(kwargs.get("visibility") or []) == 0:
            fail("When using internal = True, you must set visibility. " +
                 "In most cases, visibility = [\"//visibility:private\"] " +
                 "or visibility = [\"//:__subpackages__\"] are suitable.")
        for item in kwargs["visibility"]:
            if item == "//visibility:public":
                fail("When using internal = True, visibility can't be public")
        install_hdrs_exclude = hdrs
        kwargs = amend(kwargs, "tags", append = [
            "exclude_from_libdrake",
            "exclude_from_package",
        ])

    # We install private_hdrs by default, because Bazel's visibility denotes
    # whether headers can be *directly* included when using cc_library; it does
    # not precisely relate to which headers should appear in the install tree.
    # For example, common/symbolic.h is the only public-visibility header for
    # its cc_library, but we also need to install all of its child headers that
    # it includes, such as common/symbolic_expression.h.
    new_srcs, add_deps = _maybe_add_pruned_private_hdrs_dep(
        base_name = name,
        srcs = srcs,
        deps = interface_deps + implementation_deps,
        declare_installed_headers = declare_installed_headers,
        tags = new_tags,
        **kwargs
    )
    _raw_drake_cc_library(
        name = name,
        hdrs = hdrs,
        srcs = new_srcs,
        interface_deps = interface_deps + add_deps,
        implementation_deps = implementation_deps,
        linkstatic = linkstatic,
        declare_installed_headers = declare_installed_headers,
        install_hdrs_exclude = install_hdrs_exclude,
        tags = new_tags,
        **kwargs
    )

def _check_package_library_name(name):
    # Assert that :name is the default library for native.package_name().
    expected_name = native.package_name().split("/")[-1]
    if name != expected_name:
        fail(("The drake_cc_package_library(name = \"{}\", ...) " +
              "should be named \"{}\"").format(name, expected_name))

def drake_cc_package_library(
        name,
        deps = [],
        testonly = False,
        visibility = None):
    """Creates a rule to declare a C++ "package" library -- a library whose
    name matches the current Bazel package name (i.e., directory name) and
    whose dependencies are (usually) all of the other drake_cc_library targets
    in the current package.  In short, e.g., creates a library named
    //foo/bar:bar that conveniently provides all of the C++ code from the
    //foo/bar package in one place.

    Using this macro documents the intent that the library is a summation of
    everything in the current package and enables Drake's linter rules to
    confirm that all of the drake_cc_library targets have been listed as deps.

    Within Drake, by convention, every package (i.e., directory) that has any
    C++ code should call this macro to create a library for its package.

    The name must be the same as the final element of the current package.
    This rule does not accept srcs, hdrs, etc. -- only deps.
    The testonly argument has the same meaning as the native cc_library.

    The visibility must be explicitly provided, not relying on the BUILD file
    default.  Setting to "//visibility:public" is strongly recommended.
    """
    _check_package_library_name(name)
    if not visibility:
        fail(("//{}:{} must provide a visibility setting; " +
              "add this line to the BUILD.bazel file:\n" +
              "        visibility = \"//visibility:public\",").format(
            native.package_name(),
            name,
        ))
    drake_cc_library(
        name = name,
        testonly = testonly,
        tags = ["drake_cc_package_library"],
        visibility = visibility,
        deps = deps,
    )

def drake_cc_binary(
        name,
        *,
        add_test_rule = False,
        test_rule_args = None,
        test_rule_data = None,
        test_rule_flaky = False,
        test_rule_size = None,
        test_rule_tags = None,
        test_rule_timeout = None,
        **kwargs):
    """Creates a rule to declare a C++ binary.

    By default, we prefer to link static libraries whenever they are available.
    This default could be revisited if binary size becomes a concern.

    Note that drake_cc_binary does not draw any distinction between "interface
    deps" vs "implementation deps". There is no "interface", because this is a
    binary, not a library.

    If you wish to create a smoke-test demonstrating that your binary runs
    without crashing, supply add_test_rule=1. Note that if you wish to do
    this, you should consider suppressing that urge, and instead writing real
    tests. The smoke-test will be named <name>_test. You may override cc_test
    defaults using test_rule_args=["-f", "--bar=42"] or test_rule_size="baz".
    """
    if "@gtest//:main" in kwargs.get("deps", []):
        fail("Use drake_cc_googletest to declare %s as a test" % name)
    orig_copts = kwargs.get("copts", [])
    kwargs = amend(kwargs, "linkshared", default = False)
    kwargs = amend(kwargs, "linkstatic", default = True)
    kwargs = _amend_copts(kwargs)
    kwargs = _amend_private_hdrs(kwargs, base_name = name)
    # We should deduplicate symbols while linking (for a ~6% reduction in disk
    # use), to conserve space in CI; see #18545 for details.
    kwargs = amend(kwargs, "features", append = [
        "-no_deduplicate",
    ])
    cc_binary(
        name = name,
        **kwargs
    )
    if add_test_rule:
        kwargs = amend(kwargs, "data", append = test_rule_data)
        kwargs = amend(kwargs, "tags", append = test_rule_tags)
        kwargs = amend(kwargs, "tags", append = ["nolint"])
        kwargs["copts"] = orig_copts
        kwargs.pop("testonly", None)
        drake_cc_test(
            name = name + "_test",
            args = test_rule_args,
            flaky = test_rule_flaky,
            size = test_rule_size,
            timeout = test_rule_timeout,
            **kwargs
        )

def drake_cc_test(
        name,
        *,
        allow_network = None,
        num_threads = None,
        disable_in_compilation_mode_dbg = False,
        **kwargs):
    """Creates a rule to declare a C++ unit test.  Note that for almost all
    cases, drake_cc_googletest should be used, instead of this rule.

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.
    Unconditionally forces testonly=True.

    @param allow_network (optional, default is ["meshcat"])
        See drake/tools/skylark/README.md for details.

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.

    @param disable_in_compilation_mode_dbg (optional, default is False)
        If set, then in debug-mode builds all test cases will be suppressed, so
        the test will trivially pass. This option should be used only rarely,
        and the reason should always be documented.

    @param gcc_copts, clang_copts (optional, default is []).
        See drake/tools/skylark/README.md for details.
    """
    kwargs = amend(kwargs, "size", default = "small")
    kwargs = amend(kwargs, "srcs", default = ["test/%s.cc" % name])
    kwargs = amend(kwargs, "testonly", implicit_value = True)
    kwargs = incorporate_allow_network(kwargs, allow_network = allow_network)
    kwargs = incorporate_num_threads(kwargs, num_threads = num_threads)
    kwargs = _amend_copts(kwargs, cc_test = True)
    kwargs = _amend_private_hdrs(kwargs, base_name = name)
    if disable_in_compilation_mode_dbg:
        kwargs = amend(kwargs, "args", append = select({
            "//tools/cc_toolchain:debug": ["--gtest_filter=-*"],
            "//conditions:default": [],
        }))
        kwargs = amend(kwargs, "tags", append_nodup = [
            "no_asan",
            "no_kcov",
            "no_lsan",
            "no_memcheck",
            "no_tsan",
            "no_ubsan",
        ])
    # We should deduplicate symbols while linking (for a ~6% reduction in disk
    # use), to conserve space in CI; see #18545 for details.
    kwargs = amend(kwargs, "features", append = [
        "-no_deduplicate",
    ])
    # kcov is only appropriate for small-sized unit tests. If a test needs a
    # shard_count or a special timeout, we assume it is not small.
    if "shard_count" in kwargs or "timeout" in kwargs:
        kwargs = amend(kwargs, "tags", append_nodup = ["no_kcov"])
    cc_test(
        name = name,
        **kwargs
    )

def drake_cc_googletest(
        name,
        *,
        use_default_main = True,
        **kwargs):
    """Creates a rule to declare a C++ unit test using googletest.

    This suports all of the same options that drake_cc_test, as well as one
    more googletest-specific option:
    - When use_default_main is True, this links a default C++ main() function.
      Otherwise, this only links @gtest//:without_main and you need to write
      or link your own custom main() function.
    """
    if use_default_main:
        gtest = "//common/test_utilities:drake_cc_googletest_main"
    else:
        gtest = "@gtest//:without_main"
    kwargs = amend(kwargs, "deps", append = [gtest])
    drake_cc_test(
        name = name,
        **kwargs
    )

def drake_cc_library_linux_only(
        name,
        *,
        interface_deps = None,
        deps = [],
        linkopts = [],
        enable_condition = "@drake//tools/skylark:linux",
        **kwargs):
    """Declares a platform-specific drake_cc_library.

    When building on non-Linux, the interface_deps and deps and linkopts are
    nulled out. Note that we do NOT null out srcs; using a select() on srcs
    would cause the linter to skip them, even on Linux builds.

    The tags will have "manual" added so that the library compile is skipped
    unless some other target depends on this library.

    Because this library is not cross-platform, the visibility defaults to
    private and internal is forced to True (so that, e.g., the headers are
    excluded from the installation).
    """
    kwargs = amend(kwargs, "tags", append_nodup = ["manual"])
    kwargs = amend(kwargs, "visibility", default = ["//visibility:private"])
    kwargs = amend(kwargs, "internal", implicit_value = True)
    drake_cc_library(
        name = name,
        interface_deps = None if interface_deps == None else select({
            enable_condition: interface_deps,
            "//conditions:default": [],
        }),
        deps = select({
            enable_condition: deps,
            "//conditions:default": [],
        }),
        linkopts = select({
            enable_condition: linkopts,
            "//conditions:default": [],
        }),
        **kwargs
    )

def drake_cc_googletest_linux_only(
        name,
        *,
        data = [],
        deps = [],
        linkopts = [],
        tags = [],
        timeout = None,
        visibility = ["//visibility:private"],
        enable_condition = "@drake//tools/skylark:linux"):
    """Declares a platform-specific drake_cc_googletest. When not building on
    Linux, the deps and linkopts are nulled out. When only a subset of linuxen
    are supported, the enable_condition can be used to narrow even further.

    Because this test is not cross-platform, the visibility defaults to
    private.
    """
    # Note that we do NOT allow any kwargs-based forwarding in this macro,
    # unlike our other drake_cc_foo wrappers. That's because we must very
    # carefully consider how each argument should map into the "linux only"
    # regime, so any time we need to support a new option we want to force
    # ourselves to write out the logic for it specifically.

    # We need add the source file to an intermediate cc_library so that our
    # linters will find it. The library will not be compiled on non-Linux.
    srcs = ["test/{}.cc".format(name)]
    drake_cc_library(
        name = "_{}_compile".format(name),
        srcs = srcs,
        testonly = True,
        tags = ["manual"],
        deps = select({
            enable_condition: deps + [
                "@gtest//:without_main",
            ],
            "//conditions:default": [],
        }),
        linkopts = select({
            enable_condition: linkopts,
            "//conditions:default": [],
        }),
        alwayslink = True,
        visibility = ["//visibility:private"],
    )

    # Now link the unit test (but on non-Linux, skip over the actual code).
    # We need to use a dummy header file to disable the default 'srcs = ...'
    # inference from drake_cc_googletest.
    generate_file(
        name = "_{}_empty.cc".format(name),
        content = "",
        visibility = ["//visibility:private"],
    )
    drake_cc_googletest(
        name = name,
        srcs = ["_{}_empty.cc".format(name)],
        tags = tags + ["nolint"],
        timeout = timeout,
        data = data,
        deps = select({
            enable_condition: [":_{}_compile".format(name)],
            "//conditions:default": [],
        }),
        visibility = visibility,
    )
