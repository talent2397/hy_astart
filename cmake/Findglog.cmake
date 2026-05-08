include(FindPackageHandleStandardArgs)

set(GLOG_ROOT_DIR "" CACHE PATH "Folder contains Google glog")

find_path(GLOG_INCLUDE_DIR glog/logging.h
    PATHS ${GLOG_ROOT_DIR})

find_library(GLOG_LIBRARY glog
    PATHS ${GLOG_ROOT_DIR}
    PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(glog DEFAULT_MSG GLOG_INCLUDE_DIR GLOG_LIBRARY)

if(glog_FOUND)
    set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
    set(GLOG_LIBRARIES ${GLOG_LIBRARY})
    message(STATUS "Found glog (include: ${GLOG_INCLUDE_DIR}, library: ${GLOG_LIBRARY})")
    mark_as_advanced(GLOG_ROOT_DIR GLOG_LIBRARY GLOG_INCLUDE_DIR)
endif()
