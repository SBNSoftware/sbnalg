# ======================================================================
# sbnalg main build file
# ======================================================================


cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

find_package(cetmodules 3.27.00 REQUIRED)
project(sbnalg VERSION 10.12.02 LANGUAGES CXX)

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
find_package( messagefacility REQUIRED )
find_package( nusimdata REQUIRED )
find_package( lardataobj REQUIRED )
find_package( larcoreobj REQUIRED )
find_package( larcorealg REQUIRED )
find_package( lardataalg REQUIRED )
find_package( sbnobj REQUIRED )
find_package( sbnanaobj REQUIRED )
find_package( CLHEP REQUIRED )
find_package( ROOT REQUIRED )
# find_package( Boost COMPONENTS system filesystem REQUIRED )

# macros for dictionary and simple_plugin
include(ArtDictionary)
include(BuildPlugins)

add_subdirectory(sbnalg)
add_subdirectory(test)
add_subdirectory(fcl)

#generate cmake config files
cet_cmake_config()
