/**
 * @file   CachedPointTracker_test.cc
 * @brief  Unit test for `sbn::CachedPointTracker`.
 * @date   March 12, 2025
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * 
 * This unit test uses the standard LArSoft geometry (single TPC) because that's
 * the only one available in `sbnalg`. That geometry has a single TPC.
 * A unit test in `icarusalg` may provide better validation.
 * 
 * Usage:
 *     
 *     CachedPointTracker_test  [ConfigurationFile]
 *     
 */

// Boost test libraries; defining this symbol tells boost somehow to generate
// a main() function; Boost is pulled in by boost_unit_test_base.h
#define BOOST_TEST_MODULE CachedPointTrackerTest

// SBN software
#include "test/Geometry/CachedPointTrackerTestClass.h"

// LArSoft libraries
#include "larcorealg/Geometry/GeometryCore.h"
#include "larcorealg/Geometry/GeoObjectSorterStandard.h"
#include "larcorealg/Geometry/StandaloneGeometrySetup.h"
#include "larcorealg/TestUtils/boost_unit_test_base.h"
#include "larcorealg/TestUtils/geometry_unit_test_base.h"

// C++ standard libraries
#include <memory> // std::make_shared()


//------------------------------------------------------------------------------
//---  The test environment
//---

// we define here all the configuration that is needed; in the specific, the type of the
// channel mapping and a proper test name, used for output only;
// BasicGeometryEnvironmentConfiguration can read the configuration file name from command
// line, and BoostCommandLineConfiguration<> makes it initialize in time for Boost to
// catch it when instanciating the fixture.
struct StandardGeometryConfiguration
  : public testing::BoostCommandLineConfiguration<testing::BasicGeometryEnvironmentConfiguration> {
  StandardGeometryConfiguration() { SetApplicationName("CachedPointTrackerTest"); }
};

/**
 * Our fixture is based on GeometryTesterEnvironment, configured with the object above.
 * It provides to the testing environment:
 * - `Tester()`, returning a configured instance of the test algorithm;
 * - `GlobalTester()`, (static) returning a global configured instance of the test
 *   algorithm.
 *
 * The testing::TestSharedGlobalResource<> facility provides a singleton instance of the
 * tester algorithm, shared through the whole program and with this object too.
 *
 * This sharing allows the fixture to be used either as global or as per-suite.  In the
 * former case, the BOOST_AUTO_TEST_CASE's will access the global test algotithm instance
 * through the static call to `CachedPointTrackerTestFixture::GlobalTester()`; in the latter
 * case, it will access the local tester via the member function `Tester()`.  In this
 * case, whether `GlobalTester()` and `Tester()` point to the same tester depends on Boost
 * unit test implementation.
 */

class CachedPointTrackerTestFixture
  : private testing::GeometryTesterEnvironment<StandardGeometryConfiguration,
                                               geo::GeoObjectSorterStandard> {
  using Tester_t = sbn::CachedPointTrackerTestClass;
  using TesterRegistry_t = testing::TestSharedGlobalResource<Tester_t>;

public:
  /// Constructor: initialize the tester with the Geometry from base class
  CachedPointTrackerTestFixture()
  {
    tester_ptr = std::make_shared<Tester_t>(*Geometry());
    TesterRegistry_t::ProvideDefaultSharedResource(tester_ptr);
  }

  /// Retrieves the local tester
  Tester_t& Tester() { return *tester_ptr; }

  /// Retrieves the global tester
  static Tester_t& GlobalTester() { return TesterRegistry_t::Resource(); }

private:
  std::shared_ptr<Tester_t> tester_ptr;
}; // class CachedPointTrackerTestFixture


//------------------------------------------------------------------------------
//---  The tests
//---

BOOST_GLOBAL_FIXTURE(CachedPointTrackerTestFixture);

BOOST_AUTO_TEST_CASE(RandomPointTests)
{
  CachedPointTrackerTestFixture::GlobalTester().testTPCcorners();
  CachedPointTrackerTestFixture::GlobalTester().testCryostatCorners();
}

BOOST_AUTO_TEST_CASE(ContainedPointTests)
{
  CachedPointTrackerTestFixture::GlobalTester().testContainedPoints();
}

BOOST_AUTO_TEST_CASE(UncontainedPointTests)
{
  CachedPointTrackerTestFixture::GlobalTester().testExtraTPCpoints();
}

BOOST_AUTO_TEST_CASE(CrossingPointTests)
{
  // no-op here since this geometry has a single TPC
  CachedPointTrackerTestFixture::GlobalTester().testTPCcrossingPoints();
}


