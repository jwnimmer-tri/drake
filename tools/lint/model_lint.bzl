def model_lint_test(name, srcs, data = None):
    tool = "//multibody/parsing:parser_manual_test"
    native.sh_test(
        name = name,
        srcs = [tool],
        data = [tool] + srcs + (data or []),
        args = [
            "--strict=true",
            "--werror=true",
        ] + [
            "$(locations {})".format(item)
            for item in srcs
        ],
        tags = ["lint", "model_lint"],
    )
