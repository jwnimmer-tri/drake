load("//tools/workspace:github.bzl", "github_release_attachments")

def ktx_software_internal_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "KhronosGroup/KTX-Software",
        commit = "v4.3.1",
        attachments = {
            "KTX-Software-4.3.1-Linux-arm64.tar.bz2": "f7c7b10e16ab4055eec92b0f722f405f1a03f1779b7c12c561951dce3453e9d1",  # noqa
            "KTX-Software-4.3.1-Linux-x86_64.tar.bz2": "369492285936f4ef1a9d7d775052ccd5b06ede55102a407d097d55d0d3b16287",  # noqa
        },
        extract = [
            "KTX-Software-4.3.1-Linux-arm64.tar.bz2",
            "KTX-Software-4.3.1-Linux-x86_64.tar.bz2",
        ],
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
