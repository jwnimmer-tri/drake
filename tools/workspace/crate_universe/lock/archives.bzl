# This file is automatically generated by upgrade.sh.
ARCHIVES = [
    dict(
        name = "crate__amd-0.2.2",
        sha256 = "a679e001575697a3bd195813feb57a4718ecc08dc194944015cbc5f6213c2b96",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/amd/0.2.2/download"],
        strip_prefix = "amd-0.2.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.amd-0.2.2.bazel"),
    ),
    dict(
        name = "crate__autocfg-1.3.0",
        sha256 = "0c4b4d0bd25bd0b74681c0ad21497610ce1b7c91b1022cd21c80c6fbdd9476b0",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/autocfg/1.3.0/download"],
        strip_prefix = "autocfg-1.3.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.autocfg-1.3.0.bazel"),
    ),
    dict(
        name = "crate__blas-0.22.0",
        sha256 = "ae980f75c3215bfe8203c349b28149b0f4130a262e072913ccb55f877cd239dc",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/blas/0.22.0/download"],
        strip_prefix = "blas-0.22.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.blas-0.22.0.bazel"),
    ),
    dict(
        name = "crate__blas-sys-0.7.1",
        sha256 = "13b1b279ceb25d7c4faaea95a5f7addbe7d8c34f9462044bd8e630cebcfc2440",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/blas-sys/0.7.1/download"],
        strip_prefix = "blas-sys-0.7.1",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.blas-sys-0.7.1.bazel"),
    ),
    dict(
        name = "crate__cfg-if-1.0.0",
        sha256 = "baf1de4339761588bc0619e3cbc0120ee582ebb74b53b4efbf79117bd2da40fd",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/cfg-if/1.0.0/download"],
        strip_prefix = "cfg-if-1.0.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.cfg-if-1.0.0.bazel"),
    ),
    dict(
        name = "crate__clarabel-0.7.1",
        patches = [
            "@drake//tools/workspace/crate_universe:patches/clarabel_blas.patch",
        ],
        sha256 = "e4c0e3ebbd6441dcc7f879e89e727fe90b4e4fcecf6295f283a0d02077fb8365",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/clarabel/0.7.1/download"],
        strip_prefix = "clarabel-0.7.1",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.clarabel-0.7.1.bazel"),
    ),
    dict(
        name = "crate__darling-0.14.4",
        sha256 = "7b750cb3417fd1b327431a470f388520309479ab0bf5e323505daf0290cd3850",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/darling/0.14.4/download"],
        strip_prefix = "darling-0.14.4",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.darling-0.14.4.bazel"),
    ),
    dict(
        name = "crate__darling_core-0.14.4",
        sha256 = "109c1ca6e6b7f82cc233a97004ea8ed7ca123a9af07a8230878fcfda9b158bf0",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/darling_core/0.14.4/download"],
        strip_prefix = "darling_core-0.14.4",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.darling_core-0.14.4.bazel"),
    ),
    dict(
        name = "crate__darling_macro-0.14.4",
        sha256 = "a4aab4dbc9f7611d8b55048a3a16d2d010c2c8334e46304b40ac1cc14bf3b48e",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/darling_macro/0.14.4/download"],
        strip_prefix = "darling_macro-0.14.4",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.darling_macro-0.14.4.bazel"),
    ),
    dict(
        name = "crate__derive_builder-0.11.2",
        sha256 = "d07adf7be193b71cc36b193d0f5fe60b918a3a9db4dad0449f57bcfd519704a3",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/derive_builder/0.11.2/download"],
        strip_prefix = "derive_builder-0.11.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.derive_builder-0.11.2.bazel"),
    ),
    dict(
        name = "crate__derive_builder_core-0.11.2",
        sha256 = "1f91d4cfa921f1c05904dc3c57b4a32c38aed3340cce209f3a6fd1478babafc4",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/derive_builder_core/0.11.2/download"],
        strip_prefix = "derive_builder_core-0.11.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.derive_builder_core-0.11.2.bazel"),
    ),
    dict(
        name = "crate__derive_builder_macro-0.11.2",
        sha256 = "8f0314b72bed045f3a68671b3c86328386762c93f82d98c65c3cb5e5f573dd68",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/derive_builder_macro/0.11.2/download"],
        strip_prefix = "derive_builder_macro-0.11.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.derive_builder_macro-0.11.2.bazel"),
    ),
    dict(
        name = "crate__either-1.11.0",
        sha256 = "a47c1c47d2f5964e29c61246e81db715514cd532db6b5116a25ea3c03d6780a2",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/either/1.11.0/download"],
        strip_prefix = "either-1.11.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.either-1.11.0.bazel"),
    ),
    dict(
        name = "crate__enum_dispatch-0.3.13",
        sha256 = "aa18ce2bc66555b3218614519ac839ddb759a7d6720732f979ef8d13be147ecd",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/enum_dispatch/0.3.13/download"],
        strip_prefix = "enum_dispatch-0.3.13",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.enum_dispatch-0.3.13.bazel"),
    ),
    dict(
        name = "crate__fnv-1.0.7",
        sha256 = "3f9eec918d3f24069decb9af1554cad7c880e2da24a9afd88aca000531ab82c1",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/fnv/1.0.7/download"],
        strip_prefix = "fnv-1.0.7",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.fnv-1.0.7.bazel"),
    ),
    dict(
        name = "crate__ident_case-1.0.1",
        sha256 = "b9e0384b61958566e926dc50660321d12159025e767c18e043daf26b70104c39",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/ident_case/1.0.1/download"],
        strip_prefix = "ident_case-1.0.1",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.ident_case-1.0.1.bazel"),
    ),
    dict(
        name = "crate__itertools-0.11.0",
        sha256 = "b1c173a5686ce8bfa551b3563d0c2170bf24ca44da99c7ca4bfdab5418c3fe57",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/itertools/0.11.0/download"],
        strip_prefix = "itertools-0.11.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.itertools-0.11.0.bazel"),
    ),
    dict(
        name = "crate__lapack-0.19.0",
        sha256 = "ad676a6b4df7e76a9fd80a0c50c619a3948d6105b62a0ab135f064d99c51d207",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/lapack/0.19.0/download"],
        strip_prefix = "lapack-0.19.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.lapack-0.19.0.bazel"),
    ),
    dict(
        name = "crate__lapack-sys-0.14.0",
        sha256 = "447f56c85fb410a7a3d36701b2153c1018b1d2b908c5fbaf01c1b04fac33bcbe",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/lapack-sys/0.14.0/download"],
        strip_prefix = "lapack-sys-0.14.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.lapack-sys-0.14.0.bazel"),
    ),
    dict(
        name = "crate__lazy_static-1.4.0",
        sha256 = "e2abad23fbc42b3700f2f279844dc832adb2b2eb069b2df918f455c4e18cc646",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/lazy_static/1.4.0/download"],
        strip_prefix = "lazy_static-1.4.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.lazy_static-1.4.0.bazel"),
    ),
    dict(
        name = "crate__libc-0.2.154",
        sha256 = "ae743338b92ff9146ce83992f766a31066a91a8c84a45e0e9f21e7cf6de6d346",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/libc/0.2.154/download"],
        strip_prefix = "libc-0.2.154",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.libc-0.2.154.bazel"),
    ),
    dict(
        name = "crate__num-complex-0.4.5",
        sha256 = "23c6602fda94a57c990fe0df199a035d83576b496aa29f4e634a8ac6004e68a6",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/num-complex/0.4.5/download"],
        strip_prefix = "num-complex-0.4.5",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.num-complex-0.4.5.bazel"),
    ),
    dict(
        name = "crate__num-traits-0.2.18",
        sha256 = "da0df0e5185db44f69b44f26786fe401b6c293d1907744beaa7fa62b2e5a517a",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/num-traits/0.2.18/download"],
        strip_prefix = "num-traits-0.2.18",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.num-traits-0.2.18.bazel"),
    ),
    dict(
        name = "crate__once_cell-1.19.0",
        sha256 = "3fdb12b2476b595f9358c5161aa467c2438859caa136dec86c26fdd2efe17b92",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/once_cell/1.19.0/download"],
        strip_prefix = "once_cell-1.19.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.once_cell-1.19.0.bazel"),
    ),
    dict(
        name = "crate__proc-macro2-1.0.81",
        sha256 = "3d1597b0c024618f09a9c3b8655b7e430397a36d23fdafec26d6965e9eec3eba",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/proc-macro2/1.0.81/download"],
        strip_prefix = "proc-macro2-1.0.81",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.proc-macro2-1.0.81.bazel"),
    ),
    dict(
        name = "crate__quote-1.0.36",
        sha256 = "0fa76aaf39101c457836aec0ce2316dbdc3ab723cdda1c6bd4e6ad4208acaca7",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/quote/1.0.36/download"],
        strip_prefix = "quote-1.0.36",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.quote-1.0.36.bazel"),
    ),
    dict(
        name = "crate__strsim-0.10.0",
        sha256 = "73473c0e59e6d5812c5dfe2a064a6444949f089e20eec9a2e5506596494e4623",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/strsim/0.10.0/download"],
        strip_prefix = "strsim-0.10.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.strsim-0.10.0.bazel"),
    ),
    dict(
        name = "crate__syn-1.0.109",
        sha256 = "72b64191b275b66ffe2469e8af2c1cfe3bafa67b529ead792a6d0160888b4237",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/syn/1.0.109/download"],
        strip_prefix = "syn-1.0.109",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.syn-1.0.109.bazel"),
    ),
    dict(
        name = "crate__syn-2.0.60",
        sha256 = "909518bc7b1c9b779f1bbf07f2929d35af9f0f37e47c6e9ef7f9dddc1e1821f3",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/syn/2.0.60/download"],
        strip_prefix = "syn-2.0.60",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.syn-2.0.60.bazel"),
    ),
    dict(
        name = "crate__thiserror-1.0.59",
        sha256 = "f0126ad08bff79f29fc3ae6a55cc72352056dfff61e3ff8bb7129476d44b23aa",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/thiserror/1.0.59/download"],
        strip_prefix = "thiserror-1.0.59",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.thiserror-1.0.59.bazel"),
    ),
    dict(
        name = "crate__thiserror-impl-1.0.59",
        sha256 = "d1cd413b5d558b4c5bf3680e324a6fa5014e7b7c067a51e69dbdf47eb7148b66",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/thiserror-impl/1.0.59/download"],
        strip_prefix = "thiserror-impl-1.0.59",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.thiserror-impl-1.0.59.bazel"),
    ),
    dict(
        name = "crate__unicode-ident-1.0.12",
        sha256 = "3354b9ac3fae1ff6755cb6db53683adb661634f67557942dea4facebec0fee4b",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/unicode-ident/1.0.12/download"],
        strip_prefix = "unicode-ident-1.0.12",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.unicode-ident-1.0.12.bazel"),
    ),
]
