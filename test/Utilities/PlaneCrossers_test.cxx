/**
 * @file   PlaneCrossers_test.cc
 * @brief  Unit test for `util::PlaneCrossers`.
 * @date   June 13, 2025
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 */

// Boost test libraries; defining this symbol tells boost somehow to generate
// a main() function; Boost is pulled in by boost_unit_test_base.h
#define BOOST_TEST_MODULE PlaneCrossersTest

// SBN software
#include "sbnalg/Utilities/PlaneCrossers.h"

// LArSoft libraries
#include "larcorealg/Geometry/geo_vectors_utils.h" // geo::vect namespace
#include "larcoreobj/SimpleTypesAndConstants/geo_vectors.h" // geo::Point_t

// Boost libraries
#include <boost/test/unit_test.hpp>

// C++ standard libraries
#include <array>
#include <cassert>
#include <cmath> // std::abs()


//------------------------------------------------------------------------------
//---  The tests
//---
namespace details {
  
  template <typename T>
  constexpr bool isZero(T v, T tol = T{1e-8}) { return (v >= -tol) && (v <= tol); }
  template <typename T>
  constexpr bool isNonzero(T v, T tol = T{1e-8}) { return !isZero(v, tol); }
  
  
  template <typename T>
  constexpr T cycleUp(T var, T min, T max) { return (++var >= max)? min: var; }
  template <typename T>
  constexpr T cycleUp(T var, T max) { return cycleUp(var, T{}, max); }
  
  template <typename Vector>
  std::size_t findSmallestCoord(Vector const& v) {
    auto it = geo::vect::vector_cbegin(v), cend = geo::vect::vector_cend(v);
    auto minCoordVal = std::abs(*it++);
    std::size_t minCoord = 0;
    for (std::size_t coord = 1; it != cend; ++it, ++coord) {
      if (minCoordVal < std::abs(*it)) continue;
      minCoord = coord;
      minCoordVal = std::abs(*it);
    }
    return minCoord;
  } // findSmallestCoord()


  geo::Vector_t makeAnOrthogonalVector(geo::Vector_t const& v) {
    using Vector_t = geo::Vector_t;
    
    using geo::vect::dimension, geo::vect::mag2, geo::vect::coord,
      geo::vect::dot, geo::vect::normalize;
    
    constexpr std::size_t dim = dimension<Vector_t>();
    
    assert(!isZero(geo::vect::mag2(v))); // v must not be null
    
    // The most important step is to find a direction that is not parallel to v.
    // If we exchange the two largest coordinates, we win (unless they are the same)
    
    // So: find the smallest coordinate, then the other two.
    std::size_t smallestCoord = findSmallestCoord(v);
    std::size_t largestCoord = cycleUp(smallestCoord, dim);
    std::size_t middleCoord = cycleUp(largestCoord, dim);
    if (std::abs(coord(v, largestCoord)) < std::abs(coord(v, middleCoord)))
      std::swap(middleCoord, largestCoord);
    
    Vector_t o = v; // orthogonal
    if (isZero(coord(v, largestCoord) - coord(v, middleCoord))) {
      // the two largest coordinates have the same value;
      // that value can't be 0 (or else the smallest would also be 0),
      // so setting one to `0` will change the direction of the vector
      coord(o, largestCoord) = 0.0;
    }
    else {
      // the two largest coordinates have the different value;
      // swapping them will change the direction of the vector
      coord(o, largestCoord) = coord(v, middleCoord);
      coord(o, middleCoord) = coord(v, largestCoord);
    }
    
    o -= dot(o, v) * v; // remove the component of `o` along `v`
    
    return geo::vect::normalize(o);
  } // makeAnOrthogonalVector()
  

  std::array<geo::Vector_t, 3> makeOrthogonalBase(geo::Vector_t const& normal) {
    using Vector_t = geo::Vector_t;
    
    Vector_t const Uaxis = makeAnOrthogonalVector(normal);
    Vector_t const Vaxis = geo::vect::normalize(geo::vect::cross(normal, Uaxis));
    return { Uaxis, Vaxis, geo::vect::normalize(normal) };
  } // makeOrthogonalBase()

} // namespace details


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void lineTest(
  util::PlaneCrossers<geo::Point_t> const& plane,
  geo::Point_t const& lineStart, geo::Vector_t const& lineDir
) {
  
  bool const expectResult = !details::isZero
    (geo::vect::mixedProduct(plane.Uaxis(), plane.Vaxis(), lineDir));
  
  auto const& crossing = plane(lineStart, lineDir);
  
  BOOST_TEST(crossing.present() == expectResult);
  BOOST_TEST(bool(crossing) == expectResult);
  if (crossing && expectResult) {
    geo::Point_t const intersectionPlane = plane.intersectionPoint(crossing);
    geo::Point_t const intersectionLine = lineStart + crossing.line * lineDir;
    
    BOOST_CHECK_SMALL(intersectionPlane.X() - intersectionLine.X(), 1e-6);
    BOOST_CHECK_SMALL(intersectionPlane.Y() - intersectionLine.Y(), 1e-6);
    BOOST_CHECK_SMALL(intersectionPlane.Z() - intersectionLine.Z(), 1e-6);
  }
  
} // lineTest()


void planeTest(geo::Point_t const& center, geo::Vector_t const& dir) {
  
  auto const& [ Uaxis, Vaxis, planeDir ] = details::makeOrthogonalBase(dir);
  
  util::PlaneCrossers const plane{ center, Uaxis, Vaxis };
  
  std::array const dirSteps { -2.0, -1.0, 0.0, +1.0, +2.0 };
  std::array const posSteps { -1.0, 0.0, +1.0 };
  
  BOOST_TEST_CONTEXT(
    "Test on plane at " << center << " with normal=" << plane.normalAxis()
      << ", u=" << plane.Uaxis() << ", v=" << plane.Vaxis()
  ) {
    for (double const dirStepX: dirSteps) {
      for (double const dirStepY: dirSteps) {
        for (double const dirStepZ: dirSteps) {
          
          geo::Vector_t const lineDir{ dirStepX, dirStepY, dirStepZ };
          if (geo::vect::mag2(lineDir) < 1e-6) continue;
          
          for (double const posStepX: posSteps) {
            for (double const posStepY: posSteps) {
              for (double const posStepZ: posSteps) {
                
                geo::Point_t const lineStart{ posStepX, posStepY, posStepZ };
                
                BOOST_TEST_CONTEXT("Test with line at " << lineStart << " with normal " << lineDir) {
                  lineTest(plane, lineStart, lineDir);
                }
                
              } // for z
            } // for y
          } // for x
          
        } // for dz
      } // for dy
    } // for dx
  } // BOOST_TEST_CONTEXT()
  
} // planeTest()


void gridTest() {
  /*
   * We test with 124 plane orientations and 27 plane reference points.
   * 
   * Each test will run different lines.
   */
  
  std::array const dirSteps { -2.0, -1.0, 0.0, +1.0, +2.0 };
  std::array const posSteps { -2.0, 0.0, +2.0 };
  
  for (double const dirStepX: dirSteps) {
    for (double const dirStepY: dirSteps) {
      for (double const dirStepZ: dirSteps) {
        
        geo::Vector_t const planeDir{ dirStepX, dirStepY, dirStepZ };
        if (geo::vect::mag2(planeDir) < 1e-6) continue;
        
        for (double const posStepX: posSteps) {
          for (double const posStepY: posSteps) {
            for (double const posStepZ: posSteps) {
              
              geo::Point_t const planeReference{ posStepX, posStepY, posStepZ };
              
              planeTest(planeReference, planeDir);
              
            } // for z
          } // for y
        } // for x
        
      } // for dz
    } // for dy
  } // for dx
  
} // gridTest()


//------------------------------------------------------------------------------
void PlaneCrossers_classDoc_1() {
  
  std::cout << "Documentation test (" << __func__ << ")" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  
  // The code:
  // ---------------------------------------------------------------------------
  util::PlaneCrossers plane{ geo::origin(), geo::Xaxis(), 2. * geo::Zaxis() };

  geo::Point_t const lineStart{ 0.0, 1.0, 0.0 };
  geo::Vector_t const lineDir{ 0.0, 1.0, 1.0 };
  util::PlaneCrossers<geo::Point_t>::CrossingInfo const crossing = plane(lineStart, lineDir);
  std::cout << "Intersection point: ";
  if (crossing) std::cout << (lineStart + crossing.line * lineDir);
  else          std::cout << " undefined.";
  std::cout << std::endl;
  // ---------------------------------------------------------------------------
  
  std::cout << std::string(80, '-') << std::endl;
  
  static_assert(std::is_same_v<decltype(plane), util::PlaneCrossers<geo::Point_t>>,
                "Unexpected deduced type");
  
  // the promise: ( 0, 0, -1)
  geo::Point_t const expectedCrossingPoint{ 0, 0, -1 };
  BOOST_TEST(bool(crossing));
  if (crossing) {
    geo::Point_t const crossingPoint = lineStart + crossing.line * lineDir;
    BOOST_CHECK_SMALL(crossingPoint.X() - expectedCrossingPoint.X(), 1e-6);
    BOOST_CHECK_SMALL(crossingPoint.Y() - expectedCrossingPoint.Y(), 1e-6);
    BOOST_CHECK_SMALL(crossingPoint.Z() - expectedCrossingPoint.Z(), 1e-6);
  }
  
} // PlaneCrossers_classDoc_1()


//------------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(GridTestCase)
{
  gridTest();
}


//------------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(DocumentationTestCase)
{
  PlaneCrossers_classDoc_1();
}


//------------------------------------------------------------------------------
