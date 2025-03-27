cet_make_library(LIBRARY_NAME sbnalg::test_Geometry
  USE_BOOST_UNIT
  SOURCE
    CachedPointTrackerTestClass.cxx
  LIBRARIES
    sbnalg::Geometry
    larcorealg::Geometry
  )


# unit test (use the hard-coded channel mapping for ICARUS geometry)
cet_test(CachedPointTracker_test USE_BOOST_UNIT
  SOURCE CachedPointTracker_test.cxx
  TEST_ARGS -- test_geometry.fcl  # any job configuration defining LArTPCdetector's services.Geometry
  LIBRARIES PRIVATE
    sbnalg::test_Geometry
    sbnalg::Geometry
    larcorealg::Geometry
    larcorealg::geometry_unit_test_base
    fhiclcpp::fhiclcpp
)


install_headers()
install_source()
