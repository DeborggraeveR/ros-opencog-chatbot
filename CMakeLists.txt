cmake_minimum_required(VERSION 2.8.3)
project(ros_opencog_chatbot)

add_compile_options(-std=c++11)

find_package(
  catkin
  REQUIRED
  COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

#roslaunch_add_file_check(launch)

find_library(cogutil_LIBRARY cogutil /usr/local/lib/opencog)
find_library(atomcore_LIBRARY atomcore /usr/local/lib/opencog)
find_library(atombase_LIBRARY atombase /usr/local/lib/opencog)
find_library(atomspace_LIBRARY atomspace /usr/local/lib/opencog)
find_library(smob_LIBRARY smob /usr/local/lib/opencog)
find_path(atomspace_INCLUDE_DIR NAMES opencog/atomspace/AtomSpace.h)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${atomspace_INCLUDE_DIR}
)
include_directories(
  "/usr/local/include/guile/2.2"
)

link_directories(
  /usr/local/lib/opencog
)

add_definitions(
  -DHAVE_GUILE
)
add_executable(
  opencog_chatbot
  src/chatbot_node.cpp
  src/chatbot_api/chatbot_api.cpp
  src/chatbot_api/sentence_info.cpp
)
add_dependencies(
  opencog_chatbot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  opencog_chatbot
  ${catkin_LIBRARIES}
  ${cogutil_LIBRARY}
  ${atomcore_LIBRARY}
  ${atombase_LIBRARY}
  ${atomspace_LIBRARY}
  ${smob_LIBRARY}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)