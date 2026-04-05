#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ee4308_turtle2::map_loader_component" for configuration ""
set_property(TARGET ee4308_turtle2::map_loader_component APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ee4308_turtle2::map_loader_component PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmap_loader_component.so"
  IMPORTED_SONAME_NOCONFIG "libmap_loader_component.so"
  )

list(APPEND _cmake_import_check_targets ee4308_turtle2::map_loader_component )
list(APPEND _cmake_import_check_files_for_ee4308_turtle2::map_loader_component "${_IMPORT_PREFIX}/lib/libmap_loader_component.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
