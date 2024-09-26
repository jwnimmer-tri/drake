# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.


# XXX This isn't good enough -- systems registered in C++ need to be in
# __register_systems also.

def _retrofit_diagram_builder_add_system(cls):
    def _diagram_builder_add_system(self, system):
        self._AddSystem(system=system)
        try:
            self.__registered_systems.append(system)
        except AttributeError:
            self.__registered_systems = []
            self.__registered_systems.append(system)
        return system
    cls.AddSystem = _diagram_builder_add_system
    new_doc = cls._AddSystem.__doc__
    new_doc = new_doc.replace("_AddSystem", "AddSystem")
    tokens = new_doc.split()
    assert tokens[2] == "system:", tokens[2]
    system_type = tokens[3][:-1]
    new_doc = new_doc.replace("-> None", f"-> {system_type}")
    setattr(cls.AddSystem, "__doc__", new_doc)


def _retrofit_diagram_builder_build(cls):
    def _diagram_builder_build(self):
        diagram = self._Build()
        diagram.__registered_systems = self.__registered_systems
        self.__registered_systems = []
        return diagram
    cls.Build = _diagram_builder_build
    new_doc = cls._Build.__doc__
    new_doc = new_doc.replace("_Build", "Build")
    setattr(cls.Build, "__doc__", new_doc)


def _retrofit_diagram_builder():
    for x in DiagramBuilder_.param_list:
        _retrofit_diagram_builder_add_system(DiagramBuilder_[x])
        _retrofit_diagram_builder_build(DiagramBuilder_[x])


_retrofit_diagram_builder()
