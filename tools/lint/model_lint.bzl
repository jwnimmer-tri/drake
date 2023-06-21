def model_lint_test(name, data):
    tool = "//multibody/parsing:parser_manual_test"
    native.sh_test(
        name = name,
        srcs = [tool],
        data = [tool] + data,
        args = [
            "$(locations {})".format(item)
            for item in data
        ],
        tags = ["lint", "model_lint"],
    )
