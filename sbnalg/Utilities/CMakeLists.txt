cet_make_library(INTERFACE
  SOURCE
    "PlaneCrossers.h"
  LIBRARIES
    larcorealg::Geometry
  )

install_headers()
install_source()
install_fhicl()
