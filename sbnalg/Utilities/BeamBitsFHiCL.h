/**
 * @file   sbnalg/Utilities/BeamBitsFHiCL.h
 * @brief  Extensions to use beam bit enumerators in FHiCL.
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * @date   November, 2026
 * @see    sbnobj/Common/Trigger/BeamBits.h
 * 
 * Link with `sbnobj::Common_Trigger`.
 * 
 * Note that these functions are not directly called by users, yet their mere
 * inclusion guarantees the additional functionality.
 */

#ifndef SBNALG_UTILITIES_BEAMBITSFHICL_H
#define SBNALG_UTILITIES_BEAMBITSFHICL_H

// SBN libraries
#include "sbnalg/Utilities/EnumNameRegistryFHiCL.h" // util::fhicl::decode()...
#include "sbnobj/Common/Trigger/BeamBits.h"

// C/C++ standard libraries
#include <any>
#include <ostream>


namespace sbn::bits {
  //
  // These functions might have been imported directly from `util`,
  // e.g. `using ::util::encode;`, but that would bring in all the `encode()`
  // functions from `util`, which may yield to surprises.
  //
  
  /// Helper for generic encoding of a `NamedEnum` type in FHiCL.
  decltype(auto) encode(::util::RegisteredEnum auto const& value)
    { return ::util::fhicl::encode(value); }

  /// Helper for generic decoding of a `NamedEnum` enumerator in FHiCL.
  void decode(std::any const& src, util::RegisteredEnum auto& value)
    { ::util::fhicl::decode(src, value); }

} // namespace sbn::bits


// -----------------------------------------------------------------------------

#endif // SBNALG_UTILITIES_BEAMBITSFHICL_H
