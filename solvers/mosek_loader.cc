#include <dlfcn.h>

#include <stdexcept>

#include "drake/common/find_runfiles.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace solvers {

extern "C" void* drake_solvers_dlopen_mosek(const char*) {
  RlocationOrError lib = FindRunfile("mosek/bin/libtbb.so.12.6");
  if (!lib.error.empty()) {
    throw std::runtime_error("Cannot find libtbb");
  }
  log()->info("Found {}", lib.abspath);
  void* h = ::dlopen(lib.abspath.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (h == nullptr) {
    throw std::runtime_error(fmt::format("dlopen error on libtbb: {}", dlerror()));
  }
  lib = FindRunfile("mosek/bin/libmosek64.so.10.0");
  if (!lib.error.empty()) {
    throw std::runtime_error("Cannot find libmosek64");
  }
  log()->info("Found {}", lib.abspath);
  h = ::dlopen(lib.abspath.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (h == nullptr) {
    throw std::runtime_error(fmt::format("dlopen error on libmosek: {}", dlerror()));
  }
  return h;
}

}  // namespace solvers
}  // namespace drake
