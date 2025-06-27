#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hoverboard_driver::hoverboard_driver" for configuration ""
set_property(TARGET hoverboard_driver::hoverboard_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hoverboard_driver::hoverboard_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhoverboard_driver.so"
  IMPORTED_SONAME_NOCONFIG "libhoverboard_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hoverboard_driver::hoverboard_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_hoverboard_driver::hoverboard_driver "${_IMPORT_PREFIX}/lib/libhoverboard_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
