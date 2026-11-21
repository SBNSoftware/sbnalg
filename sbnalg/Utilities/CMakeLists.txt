cet_make_library(INTERFACE
  SOURCE
    "AssnsCrosser.h"
    "PlaneCrossers.h"
    "EnumNameRegistryFHiCL.h"
    "BeamBitsFHiCL.h"
  LIBRARIES INTERFACE
    sbnobj::Common_Trigger
    sbnobj::Common_Utilities
    larcorealg::Geometry
    larcorealg::CoreUtils
    canvas::canvas
    fhiclcpp::fhiclcpp
  )

install_headers()
install_source()
