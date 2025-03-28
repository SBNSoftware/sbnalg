/**
 * @file   sbnalg/Geometry/CachedPointTracker.h
 * @brief  Provides the `sbn::CachedPointTracker` algorithm.
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * @date   March 12, 2025
 * @see    sbnalg/Geometry/CachedPointTracker.cxx
 */

#ifndef SBNALG_GEOMETRY_CACHEDPOINTTRACKER_H
#define SBNALG_GEOMETRY_CACHEDPOINTTRACKER_H

// LArSoft libraries
#include "larcorealg/Geometry/TPCGeo.h"
#include "larcoreobj/SimpleTypesAndConstants/geo_vectors.h" // geo::Point_t

// C++ standard libraries
#include <set>


// -----------------------------------------------------------------------------
// forward declarations
namespace geo { class GeometryCore; }

// -----------------------------------------------------------------------------
namespace sbn { class CachedPointTracker; }
/**
 * @brief Algorithm to speed up TPC containment checks.
 * 
 * This algorithm is based on the functionality provided by
 * `geo::GeometryCore::PositionToTPCptr()`, to return the TPC a point falls in
 * (or none at all).
 * It is optimized by assuming that contiguous points from trajectories are
 * tested in sequence, thus privileging a check in the TPC where the last point
 * was found. However, it is always as reliable as
 * `geo::GeometryCore::PositionToTPCptr()`, although in the worst case (random
 * point locations) it may be slower than directly using
 * `geo::GeometryCore::PositionToTPCptr()`.
 */
class sbn::CachedPointTracker {
  
  geo::GeometryCore const* fGeom = nullptr; ///< Cached geometry provider.
  
  geo::TPCGeo const* fLastTPC = nullptr; ///< Where the last point was found.
  
    public:
      
  
  /// Constructor: acquires the geometry provider.
  CachedPointTracker(geo::GeometryCore const& geom);
  
  
  //@{
  /**
   * @brief Returns the TPC containing the specified `point`.
   * @param point the point to be found
   * @return pointer to the TPC containing `point`, or `nullptr` if none.
   * @see `geo::GeometryCore::PositionToTPCptr()`
   * 
   * The `point` is tested against the volume of all TPC
   * (`geo::TPCGeo::ContainsPosition()`, which includes the formal TPC volume
   * in the GDML geometry description) and a pointer to the one containing
   * it is returned; if no TPC contains the point, `nullptr` is returned.
   * In both cases, the result is cached to potentially speed up the next call.
   * 
   * This function does not use the TPC active volume. If such test is desired,
   * one more step is needed:
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
   * geo::TPCGeo const* TPC = pointTracker.PositionToTPCptr(point);
   * bool const inActive = TPC && TPC->ActiveBoundingBox().ContainsPosition(point);
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   * 
   */
  geo::TPCGeo const* PositionToTPCptr(geo::Point_t const& point);
  
  geo::TPCGeo const* operator() (geo::Point_t const& point)
    { return PositionToTPCptr(point); }
  //@}
  
  
  /**
   * @brief Returns all the TPC the trajectory crosses.
   * @tparam BIter type of (forward) iterator to the points to check
   * @tparam EIter type of (forward) iterator to the end of points to check
   * @param pointBegin iterator to the first point to check
   * @param pointEnd iterator past the last point to check
   * @param includeNoTPC if set, `nullptr` is included in the result if any
   *                     point does not belong to any TPC
   * @return a set of pointers to all the TPC the points fall into, without
   *         duplicates
   * 
   * This method returns the list of all TPC crossed by the list of points
   * between `pointBegin` (included) and `pointEnd` (excluded).
   * If `includeNoTPC` is set, points not falling in any TPC will add a
   * `nullptr` to the set of TPC; if not, these points will have no effect.
   * As a consequence, a trajectory completely outside any TPC will return an
   * empty list if `includeNoTPC` is not set, and `{ nullptr }` otherwise.
   * 
   * The method does not provide any information about how many points fall in
   * each TPC, nor which individual points fall in which TPC.
   * 
   * Also note that there is no ambition to interpolate between the points in
   * the sequence: only the points in the sequence will be considered.
   * 
   */
  template <typename BIter, typename EIter>
  std::set<geo::TPCGeo const*> TrajectoryToTPCptrs
    (BIter const pointBegin, EIter const pointEnd, bool includeNoTPC = false);
  
  
  /**
   * @brief Returns all the TPC the trajectory crosses.
   * @tparam BIter type of (forward) iterator to the points to check
   * @tparam EIter type of (forward) iterator to the end of points to check
   * @param pointBegin iterator to the first point to check
   * @param pointEnd iterator past the last point to check
   * @param allowNoTPC if set, points not belonging to any TPC will not
   *                   contribute to the result
   * @return a pointer to the TPC all the points belong to, or `nullptr`
   * 
   * This method returns the single TPC crossed by the list of points
   * between `pointBegin` (included) and `pointEnd` (excluded).
   * If points fall into different TPC, `nullptr` is returned.
   * If `allowNoTPC` is not set and there is any point not belonging to any TPC,
   * that point invalidates the result and `nullptr` is returned.
   * Otherwise, such points are ignored.
   * 
   * Therefore, to interpret the result:
   *  * With `allowNoTPC` not set: a valid TPC pointer means that all the points
   *    belong to that TPC, while `nullptr` may mean that there is at least one
   *    point without all the TPC (and possibly all of them), and/or that
   *    different points belong to different TPC.
   *  * With `allowNoTPC` set: a valid TPC pointer means that no point falls
   *    into any other TPC than the returned one, but some may fall in no TPC
   *    at all.; while `nullptr` means that different points belong to different
   *    TPC.
   * 
   * The method does not provide any information about how many points fall in
   * the TPC, nor which individual points fall in it.
   * 
   * Also note that there is no ambition to interpolate between the points in
   * the sequence: only the points in the sequence will be considered.
   * 
   */
  template <typename BIter, typename EIter>
  geo::TPCGeo const* TrajectoryToTPCptr
    (BIter const pointBegin, EIter const pointEnd, bool allowNoTPC = false);
  
  
  /// Forgets the TPC the last point was found in.
  /// @return the TPC the last point was found in (or `nullptr` if none)
  geo::TPCGeo const* forgetTPC();
  
}; // sbn::CachedPointTracker


// -----------------------------------------------------------------------------
// ---  Template implementation
// -----------------------------------------------------------------------------
template <typename BIter, typename EIter>
std::set<geo::TPCGeo const*> sbn::CachedPointTracker::TrajectoryToTPCptrs(
  BIter const pointBegin, EIter const pointEnd,
  bool includeNoTPC /* = false */
) {
  
  std::set<geo::TPCGeo const*> TPCs;
  
  geo::TPCGeo const* lastInsertedTPC = nullptr;
  auto itPoint = pointBegin;
  while (itPoint != pointEnd) {
    
    geo::TPCGeo const* TPC = PositionToTPCptr(*itPoint++);
    if (!TPC && !includeNoTPC) continue;
    
    if (!lastInsertedTPC || (TPC != lastInsertedTPC)) TPCs.insert(TPC);
    lastInsertedTPC = TPC;
    
  } // while
  
  return TPCs;
} // sbn::CachedPointTracker::TrajectoryToTPCptrs()


// -----------------------------------------------------------------------------
template <typename BIter, typename EIter>
geo::TPCGeo const* sbn::CachedPointTracker::TrajectoryToTPCptr(
  BIter const pointBegin, EIter const pointEnd,
  bool allowNoTPC /* = false */
) {
  
  geo::TPCGeo const* trjTPC = nullptr;
  
  auto itPoint = pointBegin;
  while (itPoint != pointEnd) {
    
    geo::TPCGeo const* TPC = PositionToTPCptr(*itPoint++);
    if (!TPC) {
      if (!allowNoTPC) return nullptr;
      continue;
    }
    if (!trjTPC) {
      trjTPC = TPC;
      continue;
    }
    if (TPC != trjTPC) return nullptr;
    
  } // while
  
  return trjTPC;
} // sbn::CachedPointTracker::TrajectoryToTPCptr()


// -----------------------------------------------------------------------------


#endif // SBNALG_GEOMETRY_CACHEDPOINTTRACKER_H
