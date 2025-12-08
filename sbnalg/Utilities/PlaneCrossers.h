/**
 * @file   sbnalg/Utilities/PlaneCrossers.h
 * @brief  Defines the `util::PlaneCrossers` algorithm for plane/line intersection.
 * @author Gianluca Petrillo (petrillo@slac.stanford.edu)
 * @date   June 12, 2025
 */

#ifndef SBNALG_UTILITIES_PLANECROSSERS_H
#define SBNALG_UTILITIES_PLANECROSSERS_H

// LArSoft libraries
#include "larcorealg/Geometry/geo_vectors_utils.h" // geo::vect namespace

// C/C++ standard libraries
#include <cmath> // std::isinf(), std::abs()
#include <limits> // std::numeric_limits
#include <type_traits> // std::decay_t
#include <utility> // std::declval(), std::move()


//------------------------------------------------------------------------------
namespace util {
  template <typename Point> class PlaneCrossers;
}

/**
 * @brief Algorithm to intersect a plane and a line.
 * @tparam Point type representing a point in space
 * 
 * This algorithm computes the intersection between a 2D plane and a 1D straight
 * line in a Cartesian metric space.
 * 
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
 * util::PlaneCrossers plane{ geo::origin(), geo::Xaxis(), 2. * geo::Zaxis() };
 * 
 * geo::Point_t const lineStart{ 0.0, 1.0, 0.0 };
 * geo::Vector_t const lineDir{ 0.0, 1.0, 1.0 };
 * util::PlaneCrossers::CrossingInfo const crossing = plane(lineStart, lineDir);
 * std::cout << "Intersection point: ";
 * if (crossing) std::cout << (lineStart + crossing.line * lineDir);
 * else          std::cout << " undefined.";
 * std::cout << std::endl;
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * compute the intersection between the x/z plane and a line; it will print
 * something like:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Intersection point: ( 0, 0, -1)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * and the values in `crossing` will be `line = -1`, `u = 0` and `v = -0.5`.
 * Note that both the second axis of the plane and the direction of the line
 * were not unit vectors, and the coefficients `line` and `v` reflect that.
 * Also note that the same intersection point is also returned by
 * `plane.intersectionPoint(crossing)`.
 * 
 */
template <typename Point>
class util::PlaneCrossers {
  
    public:
  
  using InnerCoord_t = double; ///< Internally we always use double precision.
  
  using Point_t = Point; ///< Type of the geometry point.
  
  /// Type of the direction vector.
  using Dir_t = decltype(std::declval<Point_t>() - std::declval<Point_t>());
  
  static constexpr unsigned int Dim = geo::vect::dimension<Point_t>();
  
  // ---------------------------------------------------------------------------
  /// Information on the crossing point.
  struct CrossingInfo {
    
    /// Special value identifying an invalid coefficient.
    static constexpr InnerCoord_t Infinite
      = std::numeric_limits<InnerCoord_t>::infinity();
    
    /// Coefficient of the intersection point on the first plane axis (_u_).
    InnerCoord_t u    = Infinite;
    
    /// Coefficient of the intersection point on the second plane axis (_v_).
    InnerCoord_t v    = Infinite;
    
    /// Coefficient of the intersection point on the line direction.
    InnerCoord_t line = Infinite;
    
    /// @{
    /// Returns whether there is a finite intersection point.
    bool present() const noexcept { return !std::isinf(line); }
    operator bool() const noexcept { return present(); }
    /// @}
    
  }; // CrossingInfo
  
  
  // ---------------------------------------------------------------------------
  /**
   * @brief Constructor: sets the plane.
   * @param center reference point on the plane
   * @param udir direction representing the first axis of the plane (and metric)
   * @param vdir direction representing the second axis of the plane (and metric)
   * 
   * The plane is defined by points in the set `center + u * Uaxis + v * Vaxis`,
   * with `u` and `v` all possible real numbers.
   */
  PlaneCrossers(Point_t center, Dir_t Uaxis, Dir_t Vaxis)
    : fCenter{ std::move(center) }
    , fUaxis{ std::move(Uaxis) }
    , fVaxis{ std::move(Vaxis) }
    {}
  
  // --- BEGIN ---  Access to plane information  -------------------------------
  /// @name Access to plane information
  /// @{
  
  /// Returns the reference point of the plane ("center").
  Point_t const& center() const noexcept { return fCenter; }
  
  /// Returns the vector of the first axis of the plane.
  Dir_t const& Uaxis() const noexcept { return fUaxis; }
  
  /// Returns the vector of the second axis of the plane.
  Dir_t const& Vaxis() const noexcept { return fVaxis; }
  
  /// Returns the normal to the plane (positive-defined with _u_ and _v_).
  Dir_t normalAxis() const { return geo::vect::cross(Uaxis(), Vaxis()); }
  
  /// Returns a displacement from the plane origin by the specified coordinates.
  Dir_t displacementOnPlane(InnerCoord_t u, InnerCoord_t v) const
    { return u * Uaxis() + v * Vaxis(); }
  
  /// Returns the point on the plane with the specified coordinates.
  Point_t pointOnPlane(InnerCoord_t u, InnerCoord_t v) const
    { return center() + displacementOnPlane(u, v); }
  
  /**
   * @brief Returns the point of intersection identified by the specified `crossing`.
   * @param crossing a valid result from `this->findCrossing()`
   * @return the intersection point represented by `crossing`
   * 
   * The `crossing` result must come from this object (because it assumed its
   * plane).
   * If `crossing.present()` is `false`, the behaviour is undefined.
   */
  Point_t intersectionPoint(CrossingInfo const& crossing) const
    { return pointOnPlane(crossing.u, crossing.v); }
  
  /// @}
  // ---  END  ---  Access to plane information  -------------------------------
  
  
  // --- BEGIN ---  Algorithm execution  ---------------------------------------
  /// @name Algorithm execution
  /// @{
  
  /**
   * @brief Returns the intersection of the plane with the specified line.
   * @param point the origin of the line
   * @param dir the positive direction of the line
   * @return the intersection information (converts to `false` if no intersection)
   * 
   * The intersection of the line in the argument and the plane of the object
   * is computed and returned.
   * If there is no intersection (the plane and line are parallel), the result's
   * `present()` method returns `false` and the rest of the information is
   * invalid. Otherwise, the returned values include `u`, `v` and `line` so that
   * the intersection point can be written as `point + line * dir` and also as
   * `center() + u * Uaxis() + v * Vaxis()`.
   */
  CrossingInfo findCrossing(Point_t const& point, Dir_t const& dir) const;
  
  
  /// Calls `util::PlaneCrossers::findCrossing()`.
  CrossingInfo operator()(Point_t const& point, Dir_t const& dir) const
    { return findCrossing(point, dir); }
  
  
  /// @}
  // ---  END  ---  Algorithm execution  ---------------------------------------
  
  
    private:
  
  static_assert(Dim == 3, "Currently only dimension 3 is supported.");
  
  Point_t fCenter; ///< Center (reference point) of the plane.
  Dir_t fUaxis; ///< Direction of the first axis of the plane.
  Dir_t fVaxis; ///< Direction of the second axis of the plane.
  
}; // util::PlaneCrossers

namespace util {
  template <typename Point, typename Vector>
  PlaneCrossers(Point, Vector) -> PlaneCrossers<std::decay_t<Point>>;
}


// -----------------------------------------------------------------------------
// ---  template implementation
// -----------------------------------------------------------------------------
template <typename Point>
auto util::PlaneCrossers<Point>::findCrossing
  (Point_t const& startPoint, Dir_t const& dir) const -> CrossingInfo
{
  /*
   * Plane point:   P(u,v) = C + u U + v V
   * Line point:    P(d)   = S + d D        (as in Start and Direction)
   * 
   * P(u,v) = P(d)
   *   <=> C + u U + v V = S + d D
   *   <=> u U + v V - d D = S - C
   *   <=> { U, V, D } x { u, v, -d } = { S-C }
   * with { U, V, D } = A the matrix with U, V and D as columns,
   * { u, v, -d } = X and { S-C } = SminusC two column vectors.
   * The following is the solution in X of A X = SminusC.
   * 
   * We solve using Cramer's rule (strictly 3D) and the fact that the
   * determinant of column-made matrix { U, V, D } is equivalent to the triple
   * product ( U x V , D ).
   */
  
  // these "directions" are not built to be unit vectors.
  Dir_t const& U = Uaxis();
  Dir_t const& V = Vaxis();
  Dir_t const& D = dir;
  
  CrossingInfo crossing;
  
  using geo::vect::mixedProduct;
  
  double const detA = mixedProduct(U, V, D);
  if (std::abs(detA) < 1e-5) return {}; // return the default, invalid
  
  Dir_t const& SminusC = startPoint - center();
  return {
    /* .u    = */  mixedProduct(SminusC, V,       D      ) / detA,
    /* .v    = */  mixedProduct(U,       SminusC, D      ) / detA,
    /* .line = */ -mixedProduct(U,       V,       SminusC) / detA
  };
  
} // util::PlaneCrossers::findCrossing()


// -----------------------------------------------------------------------------

#endif // SBNALG_UTILITIES_PLANECROSSERS_H
