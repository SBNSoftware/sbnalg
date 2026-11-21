/**
 * @file   sbnalg/Utilities/EnumNameRegistryFHiCL.h
 * @brief  Extensions to use enumerators with `EnumNameRegistry` in FHiCL.
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * @date   November 20, 2026
 * @see    sbnobj/Common/Utilities/EnumNameRegistry.h
 * 
 * The functions in this library are extensions to allow for easier interface of
 * enumerators that already support the naming system from `EnumNameRegistry.h`
 * with FHiCL API (read and write in configuration enumerator values).
 * 
 * Note that these functions are not directly called by users, yet their mere
 * inclusion guarantees the additional functionality.
 */

#ifndef SBNALG_UTILITIES_ENUMNAMEREGISTRYFHICL_H
#define SBNALG_UTILITIES_ENUMNAMEREGISTRYFHICL_H

// SBN libraries
#include "sbnobj/Common/Utilities/EnumNameRegistry.h"
#include "canvas/Utilities/Exception.h"

// framework libraries
#include "fhiclcpp/coding.h" // ::fhicl::detail::decode()

// C/C++ standard libraries
#include <any>
#include <string>


// -----------------------------------------------------------------------------
namespace util::fhicl {
  
  /**
   * @name Enumerator helpers for FHiCL
   * 
   * For a type `Enum` to be seamlessly integrated to FHiCL input and output
   * (e.g. in `fhicl::ParameterSet::get<Enum>()` and `fhicl::Atom<Enum>`),
   * there must be two functions available:
   * 1. `encode(Enum)` returning something convertible to a `std::string` that
   *    turns the `value` into a text description;
   * 2. `decode(std::any, Enum)` taking something from the FHiCL system ("any")
   *    and converting it into an `Enum` value.
   * 
   * Any function with those name and appropriate parameters will do.
   * 
   * Custom is to drop them as free functions in the same namespace where `Enum`
   * is defined, so that the argument-dependent lookup will find them
   * (e.g. `encode(value)`). The helper functions here provide a standard
   * implementation for both functions serving scoped enumerators which support
   * the protocol in `EnumNameRegistry.h`.
   * 
   */
  /// @{
  
  /// Helper for generic encoding of a `NamedEnum` type in FHiCL.
  std::string encode(NamedEnum auto const& value);
  
  /// Helper for generic encoding of `RegisteredEnum` types that are not
  /// `NamedEnum` (Bad!!) in FHiCL.
  template <typename Enum>
  requires (RegisteredEnum<Enum> && !NamedEnum<Enum>)
  std::string encode(Enum const& value);

  /// Helper for generic decoding of a `RegisteredEnum` enumerator in FHiCL.
  template <RegisteredEnum Enum>
  void decode(std::any const& src, Enum& value);
    
  /// @}

} // namespace util::fhicl


// -----------------------------------------------------------------------------
// ---  template implementation
// -----------------------------------------------------------------------------
std::string util::fhicl::encode(NamedEnum auto const& value)
  { return name(value); }


// -----------------------------------------------------------------------------
template <typename Enum>
requires (util::RegisteredEnum<Enum> && !util::NamedEnum<Enum>)
std::string util::fhicl::encode(Enum const& value)
  { return ::util::name(value); }


// -----------------------------------------------------------------------------
template <util::RegisteredEnum Enum>
void util::fhicl::decode(std::any const& src, Enum& value) {
  std::string name;
  ::fhicl::detail::decode(src, name);
  auto const optValue = util::parseEnumNameAnyCase<Enum>(name);
  if (!optValue) {
    art::Exception e{ art::errors::Configuration };
    e << "Invalid enumerator name: '" << name << "'; valid names:";
    for (auto const& n: ::util::allEnumNames<Enum>())
      e << " '" << n << "'";
    throw e << ".\n";
  }
  value = optValue.value();
} // util::fhicl::decode()


// -----------------------------------------------------------------------------

#endif // SBNALG_UTILITIES_ENUMNAMEREGISTRYFHICL_H
