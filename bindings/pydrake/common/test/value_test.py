import copy
import unittest

from pydrake.common.cpp_param import List
from pydrake.common.value import AbstractValue, Value

from pydrake.common.test.value_test_util import (
    make_abstract_value_cc_type_unregistered,
    CustomType,
    MoveOnlyType,
)


class TestValue(unittest.TestCase):
    def test_abstract_value_type_conversion(self):
        """Tests type-conversion types (passed by value, not reference)."""
        expected = "Hello world"
        value = Value[str](expected)
        self.assertIsInstance(value, AbstractValue)
        self.assertEqual(value.get_value(), expected)
        expected_new = "New value"
        value.set_value(expected_new)
        self.assertEqual(value.get_value(), expected_new)
        # Test docstring.
        self.assertFalse("unique_ptr" in value.set_value.__doc__)
        with self.assertRaises(RuntimeError) as cm:
            value.get_mutable_value()
        self.assertEqual(
            str(cm.exception),
            "Cannot get mutable value (or reference) for a type-conversion "
            "type: <class 'str'>")

    def test_abstract_value_registered_class(self):
        """Tests registered class types (passable by reference and value). Also
        tests a move-only class type."""
        obj = MoveOnlyType(10)
        self.assertEqual(
            str(Value[MoveOnlyType]),
            "<class 'pydrake.common.value.Value[MoveOnlyType]'>")
        # This *always* clones `obj`.
        value = Value[MoveOnlyType](obj)
        self.assertTrue(value.get_value() is not obj)
        self.assertEqual(value.get_value().x(), 10)
        # Set value (via `get_mutable_value`).
        value.get_mutable_value().set_x(20)
        self.assertEqual(value.get_value().x(), 20)
        # Test custom emplace constructor.
        emplace_value = Value[MoveOnlyType](30)
        self.assertEqual(emplace_value.get_value().x(), 30)
        # Test docstring.
        self.assertTrue("unique_ptr" in value.set_value.__doc__)

    def test_abstract_value_py_object(self):
        # The contained object is captured by reference, *not* by copying.
        contained = {"a": 10}
        value = Value[object](contained)
        self.assertTrue(value.get_value() is contained)

        # Change the conatined object; the reference remains intact.
        value.get_mutable_value()["b"] = 20
        self.assertTrue(value.get_value() is contained)
        self.assertEqual(value.get_value().get("b"), 20)

        # Cloning the value should perform a deep copy of the Python object.
        value_clone = value.Clone()
        self.assertTrue(value.get_value() is contained)
        self.assertTrue(value_clone.get_value() is not contained)
        self.assertEqual(value_clone.get_value(), contained)
        value_clone.get_mutable_value()["c"] = 30
        self.assertEqual(value.get_value(), {"a": 10, "b": 20})
        self.assertEqual(value_clone.get_value(),
                         {"a": 10, "b": 20, "c": 30})

        # Deep copying the value should perform a deep copy of the Python
        # object.
        value_deepcopy = copy.deepcopy(value)
        self.assertTrue(value.get_value() is contained)
        self.assertTrue(value_deepcopy.get_value() is not contained)
        self.assertEqual(value_deepcopy.get_value(), contained)
        value_deepcopy.get_mutable_value()["d"] = 40
        self.assertEqual(value.get_value(), {"a": 10, "b": 20})
        self.assertEqual(value_deepcopy.get_value(),
                         {"a": 10, "b": 20, "d": 40})

        # Using `set_value` on the original value changes object reference.
        new_contained = {"a": 20}
        value.set_value(new_contained)
        self.assertTrue(value.get_value() is new_contained)
        self.assertTrue(value.get_value() is not contained)

    def test_abstract_value_py_object_user(self):
        # The contained object is captured by reference.
        class Stuff:
            def __init__(self):
                self.member = None
        contained = Stuff()
        contained.member = ["a"]
        value = Value[object](contained)
        self.assertTrue(value.get_value() is contained)

        # Cloning the value should perform a deep copy of the Python object.
        value_clone = value.Clone()
        self.assertTrue(value.get_value() is contained)
        self.assertTrue(value_clone.get_value() is not contained)
        value_clone.get_mutable_value().member.append("b")
        self.assertEqual(value.get_value().member, ["a"])
        self.assertEqual(value_clone.get_value().member, ["a", "b"])

    def assert_equal_but_not_aliased(self, a, b):
        self.assertEqual(a, b)
        self.assertIsNot(a, b)

    def test_abstract_value_py_list(self):
        value_cls = Value[List[CustomType]]
        empty = []
        nonempty = [CustomType()]
        # Empty construction.
        value = value_cls()
        self.assertEqual(value.get_value(), empty)
        value = value_cls(empty)
        self.assert_equal_but_not_aliased(value.get_value(), empty)
        value = value_cls(nonempty)
        self.assert_equal_but_not_aliased(value.get_value(), nonempty)
        # Test mutation.
        value = value_cls(nonempty)
        self.assertNotEqual(value.get_value(), empty)
        value.set_value(empty)
        self.assert_equal_but_not_aliased(value.get_value(), empty)

    def test_abstract_value_make(self):
        value = AbstractValue.Make("Hello world")
        self.assertIsInstance(value, Value[str])
        value = AbstractValue.Make(1.0)
        self.assertIsInstance(value, Value[float])
        value = AbstractValue.Make(MoveOnlyType(10))
        self.assertIsInstance(value, Value[MoveOnlyType])
        value = AbstractValue.Make({"x": 10})
        self.assertIsInstance(value, Value[object])
        # N.B. Empty lists cannot have their type inferred, so the type will
        # default to `Value[object]`.
        value = AbstractValue.Make([])
        self.assertIsInstance(value, Value[object])
        # N.B. Non-empty lists can have their type inferred.
        value = AbstractValue.Make([CustomType()])
        self.assertIsInstance(value, Value[List[CustomType]])

    def test_abstract_value_cc_type_unregistered(self):
        value = make_abstract_value_cc_type_unregistered()
        self.assertIsInstance(value, AbstractValue)
        with self.assertRaises(RuntimeError) as cm:
            value.get_value()
        self.assertTrue(all(
            s in str(cm.exception) for s in [
                "AbstractValue",
                "UnregisteredType",
                "get_value",
                "AddValueInstantiation",
            ]), cm.exception)

    def test_value_registration(self):
        # Existence check.
        Value[object]
        Value[str]
        Value[bool]
