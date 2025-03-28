/**
 * @file   test/Geometry/CachedPointTrackerTestClass.cxx
 * @brief  Unit test for `sbn::CachedPointTracker`.
 * @date   March 12, 2025
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * @see    test/Geometry/CachedPointTrackerTestClass.h
 */


// library header
#include "test/Geometry/CachedPointTrackerTestClass.h"

// LArSoft libraries
#include "larcorealg/CoreUtils/counter.h"

// C++ standard libraries
#include <array>


//------------------------------------------------------------------------------
std::vector<geo::Point_t>
sbn::CachedPointTrackerTestClass::makeIntermediatePoints
  (geo::Point_t const& start, geo::Point_t const& stop, unsigned int nPoints)
{
  if (nPoints < 2) nPoints = 2;
  std::vector<geo::Point_t> points;
  points.reserve(nPoints);
  
  points.push_back(start);
  geo::Vector_t const step = (stop - start) / (nPoints - 1);
  for (auto const iStep: util::counter<unsigned int >(nPoints - 2))
    points.push_back(start + (iStep + 1) * step);
  points.push_back(stop); // less numerical error
  return points;
} // sbn::CachedPointTrackerTestClass::makeIntermediatePoints()


//------------------------------------------------------------------------------
std::vector<geo::Point_t> sbn::CachedPointTrackerTestClass::makePointsInsideBox
  (geo::BoxBoundedGeo const& box, unsigned int nPoints)
{
  if (nPoints == 0) return {};
  
  geo::Point_t const& c = box.Center();
  
  if (nPoints == 1) return { c };
  
  // split the z direction in `nPoints` equal intervals, and return the center
  // of each of them
  geo::Vector_t const halfRange
    = geo::Zaxis() * (box.HalfSizeZ() * (1.0 - 1.0 / nPoints));
  return makeIntermediatePoints(c - halfRange, c + halfRange, nPoints);
  
} // sbn::CachedPointTrackerTestClass::makePointsInsideBox()


//------------------------------------------------------------------------------
void sbn::CachedPointTrackerTestClass::testTPCcorners() {
  
  sbn::CachedPointTracker tracker{ *fGeom };
  
  std::array const shifts{ -30.0, -20.0, -10.0, +10.0, +20.0, +30.0 };
  for (geo::TPCGeo const& TPC: fGeom->Iterate<geo::TPCGeo>())
    testGeoObjectCornerPoints(tracker, TPC, shifts, shifts, shifts);
  
}


//------------------------------------------------------------------------------
void sbn::CachedPointTrackerTestClass::testCryostatCorners() {
  
  sbn::CachedPointTracker tracker{ *fGeom };
  
  std::array const shifts{ -50.0, -30.0, -10.0, +10.0, +30.0, +50.0 };
  for (geo::CryostatGeo const& cryo: fGeom->Iterate<geo::CryostatGeo>())
    testGeoObjectCornerPoints(tracker, cryo, shifts, shifts, shifts);
  
}


//------------------------------------------------------------------------------
void sbn::CachedPointTrackerTestClass::testContainedPoints() {
  
  sbn::CachedPointTracker tracker{ *fGeom };
  
  for (geo::TPCGeo const& TPC: fGeom->Iterate<geo::TPCGeo>()) {
    
    auto const& points = makePointsInsideBox(TPC, 10);
    
    std::array const expectedTPCs = { &TPC };
    
    // first test: get all TPCs, skip outside points
    {
      auto const TPCs
        = tracker.TrajectoryToTPCptrs(cbegin(points), cend(points), false);
      BOOST_CHECK_EQUAL_COLLECTIONS
        (cbegin(TPCs), cend(TPCs), cbegin(expectedTPCs), cend(expectedTPCs));
    }
    
    // second test: get all TPCs, include outside points (but there is none)
    {
      auto const TPCs
        = tracker.TrajectoryToTPCptrs(cbegin(points), cend(points), true);
      BOOST_CHECK_EQUAL_COLLECTIONS
        (cbegin(TPCs), cend(TPCs), cbegin(expectedTPCs), cend(expectedTPCs));
    }
    
    // third test: are all points in the same TPC; outside points invalidate
    {
      BOOST_TEST(
        tracker.TrajectoryToTPCptr(cbegin(points), cend(points), false) == &TPC
        );
    }
    
    // fourth test: are all points in the same TPC; outside points ignored
    {
      BOOST_TEST(
        tracker.TrajectoryToTPCptr(cbegin(points), cend(points), true) == &TPC
        );
    }
    
  } // for
  
} // sbn::CachedPointTrackerTestClass::testContainedPoints()


// -----------------------------------------------------------------------------
void sbn::CachedPointTrackerTestClass::testExtraTPCpoints() {
  
  
  sbn::CachedPointTracker tracker{ *fGeom };
  
  for (geo::TPCGeo const& testTPC: fGeom->Iterate<geo::TPCGeo>()) {
    
    auto points = makePointsInsideBox(testTPC, 10);
    geo::Vector_t const step = points[1] - points[0];
    
    // add 4 points outside the test TPC
    for (auto const i [[maybe_unused]]: util::counter(4))
      points.push_back(points.back() + step);
    
    // extract the result expectation
    bool uncontainedPoints = false;
    std::set<geo::TPCGeo const*> expectedTPCs; // excluding nullptr
    for (geo::Point_t const& point: points) {
      geo::TPCGeo const* TPC = fGeom->PositionToTPCptr(point);
      if (!TPC) uncontainedPoints = true;
      else expectedTPCs.insert(TPC);
    }
    std::set<geo::TPCGeo const*> expectedTPCsAndNull = expectedTPCs;
    if (uncontainedPoints) expectedTPCsAndNull.insert(nullptr);
    
    // first test: get all TPCs, skip outside points
    {
      auto const& TPCs
        = tracker.TrajectoryToTPCptrs(cbegin(points), cend(points), false);
      BOOST_CHECK_EQUAL_COLLECTIONS
        (cbegin(TPCs), cend(TPCs), cbegin(expectedTPCs), cend(expectedTPCs));
    }
    
    // second test: get all TPCs, include outside points
    {
      auto const& TPCs
        = tracker.TrajectoryToTPCptrs(cbegin(points), cend(points), true);
      BOOST_CHECK_EQUAL_COLLECTIONS(
        cbegin(TPCs), cend(TPCs),
        cbegin(expectedTPCsAndNull), cend(expectedTPCsAndNull)
        );
    }
    
    // third test: get all TPCs, ignore outside points
    {
      geo::TPCGeo const* expectedTPC
        = (expectedTPCs.size() == 1)? *expectedTPCs.begin(): nullptr;
      assert(expectedTPCsAndNull.size() > 1); // in this situation outcome is set
      
      geo::TPCGeo const* TPC
        = tracker.TrajectoryToTPCptr(cbegin(points), cend(points), true);
      BOOST_TEST(TPC == expectedTPC);
    }
    
    // fourth test: get all TPCs, outside points invalidate the result
    {
      geo::TPCGeo const* expectedTPC
        = (expectedTPCs.size() == 1 && !uncontainedPoints)
        ? *expectedTPCs.begin(): nullptr;
      assert(expectedTPC == nullptr); // in this situation outcome is set
      
      geo::TPCGeo const* TPC
        = tracker.TrajectoryToTPCptr(cbegin(points), cend(points), false);
      BOOST_TEST(TPC == expectedTPC);
    }
    
  } // for TPC
  
} // sbn::CachedPointTrackerTestClass::testExtraTPCpoints()


// -----------------------------------------------------------------------------
void sbn::CachedPointTrackerTestClass::testTPCcrossingPoints() {
  
  auto isAbeforeB = [](geo::TPCID const& A, geo::TPCID const& B)
    {
      return (A.Cryostat != B.Cryostat)? A.Cryostat < B.Cryostat: A.TPC < B.TPC;
    };
  
  
  sbn::CachedPointTracker tracker{ *fGeom };
  
  for (geo::TPCGeo const& startTPC: fGeom->Iterate<geo::TPCGeo>()) {
    for (geo::TPCGeo const& stopTPC: fGeom->Iterate<geo::TPCGeo>()) {
      if (!isAbeforeB(startTPC.ID(), stopTPC.ID())) continue;
      
      // we might add more points... here we do not
      auto const& points
        = makeIntermediatePoints(startTPC.Center(), stopTPC.Center(), 2);
      
      // extract the result expectation
      bool uncontainedPoints = false;
      std::set<geo::TPCGeo const*> expectedTPCs; // excluding nullptr
      for (geo::Point_t const& point: points) {
        geo::TPCGeo const* TPC = fGeom->PositionToTPCptr(point);
        if (!TPC) uncontainedPoints = true;
        else expectedTPCs.insert(TPC);
      }
      std::set<geo::TPCGeo const*> expectedTPCsAndNull = expectedTPCs;
      if (uncontainedPoints) expectedTPCsAndNull.insert(nullptr);
      
      // first test: get all TPCs, skip outside points
      {
        auto const& TPCs
          = tracker.TrajectoryToTPCptrs(cbegin(points), cend(points), false);
        BOOST_CHECK_EQUAL_COLLECTIONS
          (cbegin(TPCs), cend(TPCs), cbegin(expectedTPCs), cend(expectedTPCs));
      }
      
      // second test: get all TPCs, include outside points
      {
        auto const& TPCs
          = tracker.TrajectoryToTPCptrs(cbegin(points), cend(points), true);
        BOOST_CHECK_EQUAL_COLLECTIONS(
          cbegin(TPCs), cend(TPCs),
          cbegin(expectedTPCsAndNull), cend(expectedTPCsAndNull)
          );
      }
      
      // third test: get all TPCs, ignore outside points
      {
        geo::TPCGeo const* expectedTPC
          = (expectedTPCs.size() == 1)? *expectedTPCs.begin(): nullptr;
        
        geo::TPCGeo const* TPC
          = tracker.TrajectoryToTPCptr(cbegin(points), cend(points), false);
        BOOST_TEST(TPC == expectedTPC);
      }
      
      // fourth test: get all TPCs, outside points invalidate the result
      {
        geo::TPCGeo const* expectedTPC
          = (expectedTPCs.size() == 1 && !uncontainedPoints)
          ? *expectedTPCs.begin(): nullptr;
        
        geo::TPCGeo const* TPC
          = tracker.TrajectoryToTPCptr(cbegin(points), cend(points), false);
        BOOST_TEST(TPC == expectedTPC);
      }
      
    } // for second TPC
  } // for first TPC
  
} // sbn::CachedPointTrackerTestClass::testTPCcrossingPoints()


// -----------------------------------------------------------------------------
