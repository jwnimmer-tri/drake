def _strlen(s):
    return len(s)

def glob_two_files_sorted_by_filename_length(pattern):
    """..."""
    files = native.glob([pattern], allow_empty = True)
    if len(files) != 2:
        fail("Expected 2 libraries matching '{}' but instead found {}".format(
            files,
        ))
    return sorted(files, key = _strlen)
