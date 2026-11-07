# ======================================================================
# sbnalg main build file
# ======================================================================


cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

find_package(cetmodules 3.27.00 REQUIRED)
project(sbnalg VERSION 10.31.00.02 LANGUAGES CXX)

message(STATUS "\n\n  ==========================   ${PROJECT_NAME}   ==========================")

include(CetCMakeEnv)
cet_cmake_env()

cet_set_compiler_flags(DIAGS PARANOID
  WERROR
  NO_UNDEFINED
  ALLOW_DEPRECATIONS
  EXTRA_FLAGS -pedantic -Wno-unused-local-typedefs
)
cet_report_compiler_flags(REPORT_THRESHOLD VERBOSE)

# these are minimum required versions, not the actual product versions
find_package( messagefacility REQUIRED EXPORT )
find_package( nusimdata REQUIRED EXPORT )
find_package( lardataobj REQUIRED EXPORT )
find_package( larcoreobj REQUIRED EXPORT )
find_package( larcorealg REQUIRED EXPORT )
find_package( lardataalg REQUIRED EXPORT )
find_package( sbnobj REQUIRED EXPORT )
find_package( sbnanaobj REQUIRED EXPORT )
find_package( CLHEP REQUIRED EXPORT )
find_package( ROOT REQUIRED EXPORT )
find_package( Boost COMPONENTS system filesystem REQUIRED )

# macros for dictionary and simple_plugin
include(ArtDictionary)

add_subdirectory(sbnalg)
add_subdirectory(test)
add_subdirectory(fcl)

#generate cmake config files
cet_cmake_config()
