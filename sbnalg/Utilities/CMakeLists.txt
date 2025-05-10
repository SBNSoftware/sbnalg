cet_make_library(INTERFACE
  SOURCE
    "AssnsCrosser.h"
  LIBRARIES INTERFACE
    larcorealg::CoreUtils
    canvas::canvas
  )

install_headers()
install_source()
install_fhicl()
