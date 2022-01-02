
macro(ADD_GLOBAL_INCLUDE_DIR include_dir)
    set(temp ${SLAM_INCLUDE_DIR} ${include_dir})
    set(SLAM_INCLUDE_DIR ${temp} CACHE PATH "add include dir" FORCE )
endmacro()

macro(ADD_GLOBAL_LIBRARY_DIR library_dir)
    set(temp ${SLAM_LIBRARIES_DIR} ${library_dir})
    set(SLAM_LIBRARIES_DIR ${temp} CACHE PATH "add library dir " FORCE)
endmacro()

macro(ADD_GLOBAL_LIBRARY library)
   set(temp ${SLAM_LIBRARIES} ${library})
   set(SLAM_LIBRARIES ${temp} CACHE INTERNAL "add library" FORCE)
endmacro()



