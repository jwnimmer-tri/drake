import pydrake.common.eigen_geometry as mut

import numpy as np
import pickle
import unittest

from pydrake.common.test_utilities import numpy_compare


class TestEigenGeometry(unittest.TestCase):

    def test_legacy_unpickle(self):
        """Checks that data pickled as Quaternion_[float] in Drake v1.12.0
        can be unpickled as Quaternion_𝓣float𝓤 in newer versions of Drake.

        Since the unpickling shim lives at the module level, testing one class
        is sufficient even though our module has several pickle-able classes.
        """
        legacy_data = b"\x80\x04\x95\xe3\x00\x00\x00\x00\x00\x00\x00\x8c\x1dpydrake.common.eigen_geometry\x94\x8c\x12Quaternion_[float]\x94\x93\x94)\x81\x94\x8c\x15numpy.core.multiarray\x94\x8c\x0c_reconstruct\x94\x93\x94\x8c\x05numpy\x94\x8c\x07ndarray\x94\x93\x94K\x00\x85\x94C\x01b\x94\x87\x94R\x94(K\x01K\x04\x85\x94h\x07\x8c\x05dtype\x94\x93\x94\x8c\x02f8\x94\x89\x88\x87\x94R\x94(K\x03\x8c\x01<\x94NNNJ\xff\xff\xff\xffJ\xff\xff\xff\xffK\x00t\x94b\x89C \x00\x00\x00\x00\x00\x00\xf0?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x94t\x94bb."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, mut.Quaternion_[float])
        expected = np.array([1.0, 0.0, 0.0, 0.0])
        numpy_compare.assert_float_equal(obj.wxyz(), expected)

    def test_math_aliases(self):
        """Checks that the forwarding alises are intact."""
        from pydrake.common.eigen_geometry import (
            AngleAxis,
            AngleAxis_,
            Isometry3,
            Isometry3_,
            Quaternion,
            Quaternion_,
        )
