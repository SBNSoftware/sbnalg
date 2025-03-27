/**
 * @file   sbnalg/Geometry/CachedPointTracker.cxx
 * @brief  Provides the `sbn::CachedPointTracker` algorithm.
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * @date   March 12, 2025
 * @see    sbnalg/Geometry/CachedPointTracker.h
 */

// library header
#include "sbnalg/Geometry/CachedPointTracker.h"

// LArSoft libraries
#include "larcorealg/Geometry/GeometryCore.h"

// C/C++ standard libraries
#include <utility> // std::exchange()


// -----------------------------------------------------------------------------
sbn::CachedPointTracker::CachedPointTracker(geo::GeometryCore const& geom)
  : fGeom{ &geom }
{}


// -----------------------------------------------------------------------------
geo::TPCGeo const* sbn::CachedPointTracker::PositionToTPCptr
  (geo::Point_t const& point)
{
  if (fLastTPC && fLastTPC->ContainsPosition(point)) return fLastTPC;
  return fLastTPC = fGeom->PositionToTPCptr(point);
}


// -----------------------------------------------------------------------------
geo::TPCGeo const* sbn::CachedPointTracker::forgetTPC() {
  
  return std::exchange(fLastTPC, nullptr);
  
}


// -----------------------------------------------------------------------------
