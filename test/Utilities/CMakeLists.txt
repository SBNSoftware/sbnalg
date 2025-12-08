cet_test(PlaneCrossers_test USE_BOOST_UNIT
  SOURCE PlaneCrossers_test.cxx
  LIBRARIES PRIVATE
    larcoreobj::headers
    larcorealg::Geometry
)

install_source()
