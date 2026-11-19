# ?? miniocpp / httplib ??? zstd::libzstd ????
# ? cmake ??????????? zstd ??

if(NOT TARGET zstd::libzstd)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(zstd QUIET IMPORTED_TARGET libzstd)
        if(TARGET PkgConfig::zstd)
            set_property(TARGET PkgConfig::zstd PROPERTY IMPORTED_GLOBAL TRUE)
            add_library(zstd::libzstd ALIAS PkgConfig::zstd)
        endif()
    endif()
endif()
