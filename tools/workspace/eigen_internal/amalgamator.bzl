def _amalgamate_impl(ctx):
    # Collect the list of the header files mentioned in `deps`.
    details = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    ).compilation_context
    headers = depset(direct = (
        details.direct_headers +
        details.direct_private_headers +
        details.direct_public_headers +
        details.direct_textual_headers
    ), transitive = [
        details.headers,
    ])

    # Prepare to call the amalgamator.
    args = ctx.actions.args()
    args.add("--output=" + ctx.outputs.out.path)
    args.add_all(ctx.attr.roots, format_each = "--root=%s")
    args.add_all(headers, format_each = "--header=%s")

    # N.B. Add these path in "first one wins" order.
    args.add_all(details.quote_includes, format_each = "--include=%s")
    args.add_all(details.includes, format_each = "--include=%s")
    args.add_all(details.system_includes, format_each = "--include=%s")

    # Call it.
    ctx.actions.run(
        executable = ctx.executable.tool,
        inputs = headers,
        arguments = [args],
        outputs = [ctx.outputs.out],
    )

amalgamate = rule(
    implementation = _amalgamate_impl,
    attrs = {
        "roots": attr.string_list(mandatory = True),
        "deps": attr.label_list(mandatory = True, providers = [CcInfo]),
        "out": attr.output(mandatory = True),
        "tool": attr.label(
            executable = True,
            cfg = "exec",
            default = Label("//:amalgamator"),
        ),
    },
    fragments = ["cpp"],
)
