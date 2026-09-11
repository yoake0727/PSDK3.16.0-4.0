#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "miniocpp::miniocpp" for configuration "Release"
set_property(TARGET miniocpp::miniocpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(miniocpp::miniocpp PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libminio.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS miniocpp::miniocpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_miniocpp::miniocpp "${_IMPORT_PREFIX}/lib/libminio.a" )

# Import target "miniocpp::miniocpp_inih" for configuration "Release"
set_property(TARGET miniocpp::miniocpp_inih APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(miniocpp::miniocpp_inih PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libminiocpp_inih.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS miniocpp::miniocpp_inih )
list(APPEND _IMPORT_CHECK_FILES_FOR_miniocpp::miniocpp_inih "${_IMPORT_PREFIX}/lib/libminiocpp_inih.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
