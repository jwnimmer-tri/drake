load(":metadata.bzl", "generate_repository_metadata")

def _impl(repo_ctx):
    name = repo_ctx.attr.name
    aliases = repo_ctx.attr.aliases

    build = "package(default_visibility = [\"//visibility:public\"])\n"
    for name, actual in aliases.items():
        build += "alias({})\n".format(", ".join([
            "name = " + repr(name),
            "actual = " + repr(actual),
        ]))

    subdir = repo_ctx.attr.package
    if subdir == "":
        repo_ctx.file("BUILD.bazel", build)
    else:
        repo_ctx.file("{}/BUILD.bazel".format(subdir), build)

    generate_repository_metadata(
        repo_ctx,
        repository_rule_type = "alias",
    )

alias_repository = repository_rule(
    doc = """Adds a repository with aliases to other repositories.""",
    attrs = {
        "aliases": attr.string_dict(
            doc = """
            Dictionary of aliases to create. The keys are target names,
            the values are the destination labels (i.e., the 'actual').
            """,
        ),
        "package": attr.string(
            default = "",
            doc = """
            Optional package (subdirectory) to place the alias into.
            """,
        ),
    },
    implementation = _impl,
)
