#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "zlac8015d_serial::zlac8015d" for configuration ""
set_property(TARGET zlac8015d_serial::zlac8015d APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(zlac8015d_serial::zlac8015d PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libzlac8015d.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS zlac8015d_serial::zlac8015d )
list(APPEND _IMPORT_CHECK_FILES_FOR_zlac8015d_serial::zlac8015d "${_IMPORT_PREFIX}/lib/libzlac8015d.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
