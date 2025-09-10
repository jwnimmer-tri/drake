# This file governs the contents of libdrake.so.

# Do not update this list by hand; instead, from the drake workspace run
#
#   tools/install/libdrake/build_components_refresh.py
#
# and it will rewrite this file automatically for you.  If the refresh made any
# edits, then `git status` will show this file as modified; in that case, you
# should commit the changes made by the refresh script.
LIBDRAKE_COMPONENTS = [
    "//common",
    "//common/proto",
    "//common/schema",
    "//common/symbolic",
    "//common/trajectories",
    "//common/yaml",
    "//common:drake_marker_shared_library",  # unpackaged
    "//geometry",
    "//geometry/proximity",
    "//geometry/query_results",
    "//geometry/render",
    "//math",
    "//multibody/contact_solvers",
    "//multibody/contact_solvers/sap",
    "//multibody/fem",
    "//multibody/hydroelastics",
    "//multibody/inverse_kinematics",
    "//multibody/math",
    "//multibody/parsing",
    "//multibody/plant",
    "//multibody/rational",
    "//multibody/topology",
    "//multibody/tree",
    "//multibody/triangle_quadrature",
    "//planning",
    "//planning/graph_algorithms",
    "//planning/iris",
    "//planning/locomotion",
    "//planning/trajectory_optimization",
    "//solvers",
    "//systems/framework",
    "//systems/primitives",
]
