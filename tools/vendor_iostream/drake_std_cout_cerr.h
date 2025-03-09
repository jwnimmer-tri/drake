#pragma once

// These includes are required for us to be a work-alike for <iostream>.
// See https://en.cppreference.com/w/cpp/header/iostream.
#include <ios>
#include <istream>
#include <ostream>
#include <streambuf>

#include "drake_replacement_stream.h"

// A replacement for <iostream> that redirects to drake::log().
// Opening the std namespace is undefined behavior. Oh well.
namespace std {
using drake::vendor_iostream::internal::cerr;
using drake::vendor_iostream::internal::cout;
}  // namespace std
