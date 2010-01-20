# Make sure linker and compiler can find their way
include_directories(${IPVisualization_SOURCE_DIR}/tools)
link_directories(${IPVisualization_SOURCE_DIR}/tools)
message(STATUS "Adding ${IP_Visualization_SOURCE_DIR}/tools")

# Color IP
add_executable(color_ip color_ip.cc)
target_link_libraries(color_ip
  ${Boost_LIBRARIES}
  ${VisionWorkbench_LIBRARIES})
set(PROGRAM_LISTING ${PROGRAM_LISTING} color_ip)

# Color Match
add_executable(color_match color_match.cc)
target_link_libraries(color_match
  ${Boost_LIBRARIES}
  ${VisionWorkbench_LIBRARIES})
set(PROGRAM_LISTING ${PROGRAM_LISTING} color_match)

# Color Match Pts
add_executable(color_match_pts color_match_pts.cc)
target_link_libraries(color_match_pts
  ${Boost_LIBRARIES}
  ${VisionWorkbench_LIBRARIES})
set(PROGRAM_LISTING ${PROGRAM_LISTING} color_match_pts)

# Localized Norm
add_executable(localized_norm localized_norm.cc)
target_link_libraries(localized_norm
  ${Boost_LIBRARIES}
  ${VisionWorkbench_LIBRARIES})
set(PROGRAM_LISTING ${PROGRAM_LISTING} localized_norm)

# Statistics
add_executable(statistics_find statistics_find.cc)
target_link_libraries(statistics_find
  ${Boost_LIBRARIES}
  ${VisionWorkbench_LIBRARIES}
  ${LAPACK_LIBRARIES})
set(PROGRAM_LISTING ${PROGRAM_LISTING} statistics_find)

# Surf2Vwip
add_executable(surf2vwip surf2vwip.cc)
target_link_libraries(surf2vwip
  ${Boost_LIBRARIES}
  ${VisionWorkbench_LIBRARIES})
set(PROGRAM_LISTING ${PROGRAM_LISTING} surf2vwip)

# ip_print
add_executable(ip_print ip_print.cc)
target_link_libraries(ip_print
  ${Boost_LIBRARIES}
  ${VisionWorkbench_LIBRARIES})
set(PROGRAM_LISTING ${PROGRAM_LISTING} ip_print)

foreach( PROGRAM ${PROGRAM_LISTING} )

  message("Working:")
  message(${PROGRAM})
  install( TARGETS ${PROGRAM}
    DESTINATION bin )

endforeach( PROGRAM )