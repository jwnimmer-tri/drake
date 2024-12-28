from pydrake.common import _MangledName


def __getattr__(name):
    """Rewrites requests for Foo[bar] into their mangled form, for backwards
    compatibility with unpickling.
    """
    return _MangledName.module_getattr(
        module_name=__name__, module_globals=globals(), name=name)


def _wrapped_trajectory_repr(wrapped_trajectory):
    return f"WrappedTrajectory({wrapped_trajectory._unwrap()!r})"


def _add_wrapped_trajectory_repr():
    for cls in [WrappedTrajectory]:
        setattr(cls, "__repr__", _wrapped_trajectory_repr)


_add_wrapped_trajectory_repr()
