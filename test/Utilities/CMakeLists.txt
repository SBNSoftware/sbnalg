cet_test(AssnsCrosser_test_sbnalg
  SOURCE AssnsCrosser_test.cc
  LIBRARIES
    sbnalg::Utilities
    sbnalg::Test_sbnalg
    canvas::canvas
    cetlib::cetlib
  USE_BOOST_UNIT
  )
