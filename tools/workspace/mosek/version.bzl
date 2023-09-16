# When this value is updated:
# - tools/dynamic_analysis/tsan.supp may also need updating
# - LICENSE.third_party may also need updating to match
#     https://docs.mosek.com/latest/licensing/license-agreement-info.html
VERSION = "10.0.46"

# These are architecture-specific checksums of the mosektools.tar.bz2 download.
SHA256 = {
    "osx64x86": "16885bbee2c1d86e0a3f9d9a2c60bbab1bb88e6f1b843ac1fb8da0c62292344f",  # noqa
    "osxaarch64": "85724bd519d5fe120b4e8d2676b65143b9ce6dce666a07ca4f44ec54727b5ab5",  # noqa
    "linux64x86": "a6862954137493b74f55c0f2745b7f1672e602cfe9cd8974a95feaf9993f06bf",  # noqa
    "linuxaarch64": "",
}
