

add_library(MultisensorFusionLib SHARED
Sensor/Gnss_data.cpp
)
target_link_libraries( MultisensorFusionLib
  ${ALL_TARGET_LIBRARIES}
)