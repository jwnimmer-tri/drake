import unittest

from bazel_tools.tools.python.runfiles import runfiles


class TestXacroCrossCheck(unittest.TestCase):
    """We offer many different flavors of the WSG model, which are all fairly
    similar to each other. For convenience of the users of the model files,
    all models are directly committed into git; but for the sanity of the model
    file maintainers and code reviewers, all of the models are generated from
    the same xacro files. This test checks that the generated `*.sdf` outputs
    in git are up to date with the `*.xacro` source code.
    """

    def setUp(self):
        self.maxDiff = None

    def test_xacro_cross_check(self):
        for name in [
                "schunk_wsg_50.sdf",
                "schunk_wsg_50_ball_contact.sdf",
                # XXX add more here
        ]:
            with self.subTest(name=name):
                self._check(name=name)

    def _check(self, *, name):
        # Load both input files.
        base = "drake/manipulation/models/wsg_50_description"
        manifest = runfiles.Create()
        with open(manifest.Rlocation(f"{base}/sdf/{name}")) as f:
            actual = f.read()
        with open(manifest.Rlocation(f"{base}/sources/{name}")) as f:
            expected = f.read()
        self.assertMultiLineEqual(expected, actual)
