import copy

import yaml


class _SchemaLoader(yaml.loader.SafeLoader):
    """Customizes SafeLoader for the purposes of this module."""

    @staticmethod
    def _handle_multi_variant(loader, tag, node):
        """Parses a variant-like YAML node (akin to std::variant<> in C++).

        Returns a dict with the inner mapping, plus one extra item with key
        `_tag` that holds the YAML tag.

        For example, a !Gaussian YAML node is returned as:
        {
          "_tag": "!Gaussian",
          "mean": 2.0,
          "std": 4.0,
        }.
        """
        assert type(loader) == _SchemaLoader, loader
        result = loader.construct_mapping(node)
        result.update({"_tag": tag})
        return result


_SchemaLoader.add_multi_constructor("", _SchemaLoader._handle_multi_variant)


def _remove_aliases(data, *, finished_ids=None, working_ids=None):
    """Given YAML data (a dict, list, or primitive), replaces any aliased lists
    or dicts with copies, so that only read-only data is aliased.  Returns the
    (possibly copied) data.
    """
    # Bootstrap the worklists upon the first call.
    if finished_ids is None:
        assert working_ids is None
        finished_ids = set()
        working_ids = set()

    # Guard against cycles in the data. Our YAML reader only supports trees,
    # not graphs.
    data_id = id(data)
    if data_id in working_ids:
        raise RuntimeError("Infinite recursion")

    # Check for compound objects (i.e., collections).
    is_sequence = isinstance(data, list)
    is_mapping = isinstance(data, dict)
    is_compound = is_sequence or is_mapping

    # Any non-compound objects should be read-only (str, float, etc).
    # Their copy operator should be a no-op.
    if not is_compound:
        check = copy.copy(data)
        assert id(check) == data_id, f"Unexpected mutable data {data!r}"
        return data

    # Handle compound objects.
    assert is_sequence or is_mapping
    if data_id in finished_ids:
        # We have seen this object once before.  We need to return a copy,
        # instead of the alias.
        return copy.deepcopy(data)

    # This is the first time we've seen it.
    working_ids.add(data_id)
    if is_sequence:
        new_data = list()
        for old_item in data:
            new_item = _remove_aliases(
                data=old_item,
                finished_ids=finished_ids,
                working_ids=working_ids)
            new_data.append(new_item)
    else:
        assert is_mapping
        new_data = dict()
        for key, old_item in data.items():
            new_item = _remove_aliases(
                data=old_item,
                finished_ids=finished_ids,
                working_ids=working_ids)
            new_data[key] = new_item
    working_ids.remove(data_id)
    finished_ids.add(id(new_data))
    return new_data


def yaml_load_data(data, *, private=False):
    """Loads and returns the given `data` str as a yaml object, while also
    accounting for variant-like type tags.  The known variant yaml tags
    are reported as an extra "_tag" field in the returned dictionary.

    (Alternatively, `data` may be a file-like stream instead of a str.)

    By default, removes any root-level keys that begin with an underscore, so
    that yaml anchors and templates are invisibly easy to use.  Callers that
    wish to receive the private data may pass `private=True`.
    """
    result = yaml.load(data, Loader=_SchemaLoader)
    if not private:
        keys_to_remove = [
            key for key in result.keys()
            if key.startswith("_")
        ]
        for key in keys_to_remove:
            del result[key]
    result = _remove_aliases(result)
    return result


def yaml_load_file(filename, *, private=False):
    """Loads and returns the given `filename` as a yaml object, while also
    accounting for variant-like type tags.

    See yaml_load_data for full details; this function differs only by reading
    the yaml data from a file instead of memory.
    """
    with open(filename, "r") as data:
        return yaml_load_data(data, private=private)


def yaml_load(*, data=None, filename=None, private=False):
    """Loads either a `data` str xor a `filename` (exactly one must be
    specified) and returns the value as a yaml object.

    This is sugar for calling either yaml_load_data or yaml_load_file;
    refer to those functions for additional details.
    """
    if sum(bool(x) for x in [data, filename]) != 1:
        raise RuntimeError("Must specify exactly one of data= and filename=")
    if data:
        return yaml_load_data(data, private=private)
    else:
        return yaml_load_file(filename, private=private)


# This is a magic tribool value described at
#   https://github.com/yaml/pyyaml/pull/256
# and must be set this way for post- and pre- pyyaml#256 yaml dumpers to give
# the same output.
_FLOW_STYLE = None


class _SchemaDumper(yaml.dumper.SafeDumper):
    """Customizes SafeDumper for the purposes of this module."""

    def _represent_dict(self, data):
        """Permits a reverse of _SchemaLoader."""
        if "_tag" in data:
            tag = data["_tag"]
            data = copy.copy(data)
            del data["_tag"]
            return self.represent_mapping(tag, data)
        else:
            return super().represent_dict(data)


_SchemaDumper.add_representer(dict, _SchemaDumper._represent_dict)


def yaml_dump(data, *, filename=None):
    """Dumps a yaml object to a string or `filename` (if specified).

    This is the counterpart to `yaml_load(..., private=True)`, and will permit
    re-expressing the same tagged data.
    """
    if filename is not None:
        with open(filename, "w") as f:
            yaml.dump(data, f, Dumper=_SchemaDumper,
                      default_flow_style=_FLOW_STYLE)
    else:
        return yaml.dump(data, Dumper=_SchemaDumper,
                         default_flow_style=_FLOW_STYLE)
