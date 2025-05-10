# cmake driver file for testing from CET build tools
include(CetTest)

cet_make_library(
  LIBRARY_NAME Test
  INTERFACE
  SOURCE
    FrameworkEventMockup.h
  LIBRARIES INTERFACE
    canvas::canvas
)

# Always enable asserts (for tests only)
cet_enable_asserts()

add_subdirectory(Geometry)
add_subdirectory(Utilities)
