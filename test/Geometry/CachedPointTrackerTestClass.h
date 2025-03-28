/**
 * @file   test/Geometry/CachedPointTrackerTestClass.cxx
 * @brief  Unit test for `sbn::CachedPointTracker`.
 * @date   March 12, 2025
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * @see    test/Geometry/CachedPointTrackerTestClass.h,
 *         test/Geometry/CachedPointTracker_test.cc
 * 
 * Test class for `geo::CachedPointTracker`.
 * It is provided as an algorithm because it may make sense to run it on
 * more complex geometries than the simple one available in `sbnalg`.
 * 
 */

#ifndef SBNALG_TEST_GEOMETRY_CACHEDPOINTTRACKERTESTCLASS_H
#define SBNALG_TEST_GEOMETRY_CACHEDPOINTTRACKERTESTCLASS_H

// SBN software
#include "sbnalg/Geometry/CachedPointTracker.h"

// LArSoft libraries
#include "larcorealg/Geometry/GeometryCore.h"
#include "larcorealg/Geometry/TPCGeo.h"
#include "larcorealg/Geometry/BoxBoundedGeo.h"

// Boost and C++ standard libraries
#include <boost/test/unit_test.hpp>

#include <iterator> // std::rbegin(), ...
#include <vector>


//------------------------------------------------------------------------------
//--- Test class
//---

namespace sbn { class CachedPointTrackerTestClass; }
/**
 * @brief `sbn::CachedPointTracker` test class.
 * 
 * These tests assume that the right answer is the one directly given by
 * `geo::GeometryCore::PositionToTPCptr()`, and rate the results accordingly.
 * 
 */
class sbn::CachedPointTrackerTestClass {
  
  geo::GeometryCore const* fGeom = nullptr;
  
    public:
  
  CachedPointTrackerTestClass(geo::GeometryCore const& geom)
    : fGeom{ &geom }
    {}
  
  /// Test of random ordered points around the TPC corners.
  void testTPCcorners();
  
  /// Test of random ordered points around the cryostat corners.
  void testCryostatCorners();
  
  /// Test of points contained in a single TPC.
  void testContainedPoints();
  
  /// Test including points not contained in a single TPC.
  void testExtraTPCpoints();
  
  /// Test of points crossing from a TPC to another one.
  void testTPCcrossingPoints();
  
    protected:

  /// Returns `nPoints` points (minimum 2), equally spaced between `start` and
  /// `stop` (both included).
  static std::vector<geo::Point_t> makeIntermediatePoints
    (geo::Point_t const& start, geo::Point_t const& stop, unsigned int nPoints);

  /// Returns `nPoints` points, equally spaced within the specified `box`.
  static std::vector<geo::Point_t> makePointsInsideBox
    (geo::BoxBoundedGeo const& box, unsigned int nPoints);

  /// Returns a ordered sequence of coordinates applying `shifts` to `min` and `max`.
  template <typename T, typename Shifts>
  static std::vector<T> makeShiftedCoords(T min, T max, Shifts const& shifts);
    
  /// Returns a list of points based on the 8 vertices of the box, shifted by
  /// all possible combinations of shifts.
  /// The order of the points is not guaranteed.
  template <typename XShifts, typename YShifts, typename ZShifts>
  static std::vector<geo::Point_t> makeCornerPoints(
    geo::BoxBoundedGeo const& box,
    XShifts const& xShifts = {},
    YShifts const& yShifts = {},
    ZShifts const& zShifts = {}
    );

  template <typename GeoObj, typename XShifts, typename YShifts, typename ZShifts>
  void testGeoObjectCornerPoints(
    sbn::CachedPointTracker& tracker, GeoObj const& geoObj,
    XShifts const& xShifts = {},
    YShifts const& yShifts = {},
    ZShifts const& zShifts = {}
    );

}; // sbn::CachedPointTrackerTestClass


// -----------------------------------------------------------------------------
// ---  Template implementation
// -----------------------------------------------------------------------------
template <typename T, typename Shifts>
std::vector<T> sbn::CachedPointTrackerTestClass::makeShiftedCoords
  (T min, T max, Shifts const& shifts)
{
  using std::rbegin, std::rend, std::empty;
  
  if (empty(shifts)) return { min, max }; // special case

  std::vector<T> coords;
  coords.reserve(2 * shifts.size());
  for(auto itShift = rbegin(shifts); itShift != rend(shifts); ++itShift)
    coords.push_back(min - *itShift);
  for(auto itShift = begin(shifts); itShift != end(shifts); ++itShift)
    coords.push_back(max + *itShift);
  assert(coords.size() == coords.capacity());
  return coords;
} // sbn::CachedPointTrackerTestClass::makeShiftedCoords()


// -----------------------------------------------------------------------------
template <typename XShifts, typename YShifts, typename ZShifts>
std::vector<geo::Point_t> sbn::CachedPointTrackerTestClass::makeCornerPoints
(
  geo::BoxBoundedGeo const& box,
  XShifts const& xShifts /* = {} */,
  YShifts const& yShifts /* = {} */,
  ZShifts const& zShifts /* = {} */
) {
  
  std::vector<geo::Point_t> points;
  points.reserve(8 * xShifts.size() * yShifts.size() * zShifts.size());
  if (points.capacity() == 0) return points; // nothing to do!
  
  for (double const x: makeShiftedCoords(box.MinX(), box.MaxX(), xShifts)) {
    for (double const y: makeShiftedCoords(box.MinY(), box.MaxY(), yShifts)) {
      for (double const z: makeShiftedCoords(box.MinZ(), box.MaxZ(), zShifts)) {
        points.emplace_back(x, y, z);
      } // z
    } // y
  } // x
  
  return points;
  
} // sbn::CachedPointTrackerTestClass::makeCornerPoints()


// -----------------------------------------------------------------------------
template <typename GeoObj, typename XShifts, typename YShifts, typename ZShifts>
void sbn::CachedPointTrackerTestClass::testGeoObjectCornerPoints(
  sbn::CachedPointTracker& tracker, GeoObj const& geoObj,
  XShifts const& xShifts /* = {} */,
  YShifts const& yShifts /* = {} */,
  ZShifts const& zShifts /* = {} */
) {
  
  BOOST_TEST_MESSAGE("Testing " << geoObj.ID());
  for (geo::Point_t const& point
    : makeCornerPoints(geoObj.BoundingBox(), xShifts, yShifts, zShifts)
  ) {
    
    geo::TPCGeo const* expectedTPC = fGeom->PositionToTPCptr(point);
    
    BOOST_TEST(tracker.PositionToTPCptr(point) == expectedTPC);
    BOOST_TEST(tracker(point) == expectedTPC);
    
  } // points
  
} // sbn::CachedPointTrackerTestClass::testGeoObjectCornerPoints()


// -----------------------------------------------------------------------------

#endif // SBNALG_TEST_GEOMETRY_CACHEDPOINTTRACKERTESTCLASS_H
