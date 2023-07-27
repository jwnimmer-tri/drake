import logging
import multiprocessing as mp
import unittest

from pydrake.common import use_native_cpp_logging
from pydrake.common.test.text_logging_test_helpers import (
    do_log_test_async_with_gil_release,
    do_log_test_async_without_gil_release,
)


def run_in_separate_process(target):
    ctx = mp.get_context("fork")
    process = ctx.Process(target=target)
    process.start()
    process.join()


def check_log_test_async_use_native_cpp_logging():
    # If we comment out the next line, then the test will fail (by hanging
    # indefinitely until Bazel's test timeout kills it).
    use_native_cpp_logging()
    do_log_test_async_without_gil_release()


def check_log_test_async_with_gil_release():
    do_log_test_async_with_gil_release()


class TestTextLoggingThreading(unittest.TestCase):
    """
    These tests exercise Drake's native C++ logging sink in the case where
    Python code calls a bound C++ function that logs on a different C++ thread.
    """

    def test_python_threaded_logging_use_native_cpp_logging(self):
        """
        Avoids GIL deadlock by undoing our Python spdlog redirection.

        This may be useful to avoid explicitly decorating function bindings
        with `py::call_guard<py::gil_scoped_release>()`, and also where calling
        back to Python via our sink redirection may slow things down.
        """
        run_in_separate_process(check_log_test_async_use_native_cpp_logging)

    def test_python_threaded_logging_gil_release(self):
        """
        Avoids GIL deadlock by explicitly releasing the GIL in binding of a C++
        method that may call logging in a separate thread.

        This may be useful to avoid having consistent logging, at the cost of
        having to decorate each and every entry point in C++ bindings that may
        call logging from different threads, and at the cost of calling back to
        Python via our sink redirection.
        """
        run_in_separate_process(check_log_test_async_with_gil_release)
