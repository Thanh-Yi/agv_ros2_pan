#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "wt901_driver::wt901_driver" for configuration ""
set_property(TARGET wt901_driver::wt901_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(wt901_driver::wt901_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libwt901_driver.so"
  IMPORTED_SONAME_NOCONFIG "libwt901_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS wt901_driver::wt901_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_wt901_driver::wt901_driver "${_IMPORT_PREFIX}/lib/libwt901_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
