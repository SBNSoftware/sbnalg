# cmake driver file for testing from CET build tools
include(CetTest)

# Always enable asserts (for tests only)
cet_enable_asserts()

add_subdirectory(Geometry)
