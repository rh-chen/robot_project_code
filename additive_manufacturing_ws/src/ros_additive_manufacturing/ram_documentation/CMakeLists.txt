cmake_minimum_required(VERSION 3.9)
project(
  ram_documentation
  VERSION
  0.0.3
  DESCRIPTION
  "ROS Additive Manufacturing (RAM) documentation"
)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

list(
  APPEND
  CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/
)

find_package(Sphinx 1.7.4 REQUIRED)
find_package(PdfLatex REQUIRED)

# Sphinx cache with pickled ReST documents
set(
  SPHINX_CACHE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/_doctrees
)
# HTML output directory
set(
  SPHINX_HTML_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/html
)
set(
  SPHINX_PDF_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/latex
)

configure_file(
   ${CMAKE_CURRENT_SOURCE_DIR}/sphinx/conf.py.in
   conf.py
   @ONLY
)
add_custom_target(
  ${PROJECT_NAME}-doc
  ALL DEPENDS
  ${PROJECT_NAME}-doc-html
)

add_custom_target(
  ${PROJECT_NAME}-doc-latex
  ${SPHINX_EXECUTABLE} -q -c . -b latex -d ${SPHINX_CACHE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/sphinx ${SPHINX_PDF_DIR}
)
add_custom_target(
  ${PROJECT_NAME}-doc-pdf
  make -C ${SPHINX_PDF_DIR} all-pdf DEPENDS ${PROJECT_NAME}-doc-latex
)
add_dependencies(
  ${PROJECT_NAME}-doc
  ${PROJECT_NAME}-doc-pdf
)
add_custom_target(
  ${PROJECT_NAME}-doc-html
  ${SPHINX_EXECUTABLE} -q -c . -b html -d ${SPHINX_CACHE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/sphinx ${SPHINX_HTML_DIR}
)

#############
## Install ##
#############

if("${CMAKE_INSTALL_PREFIX}" MATCHES "^/opt/")
  # Handle release case
  set(DESKTOP_APP_DIR "/usr/share/applications")
  set(DESKTOP_ICON_DIR "/usr/share/icons")
  set(DESKTOP_APP_SUFFIX "")
else()
  set(DESKTOP_APP_DIR "$ENV{HOME}/.local/share/applications")
  set(DESKTOP_ICON_DIR ${DESKTOP_APP_DIR})
  set(DESKTOP_APP_SUFFIX " (debug)")
endif()
set(ROS_DISTRO $ENV{ROS_DISTRO})

# Create directory for the desktop file/icon
install(DIRECTORY DESTINATION
  ${DESKTOP_APP_DIR}
)
install(DIRECTORY DESTINATION
  ${DESKTOP_ICON_DIR}
)

# Configure / install desktop launcher
configure_file(
  desktop/${PROJECT_NAME}.desktop.in
  desktop/${PROJECT_NAME}.desktop
)
install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/desktop/${PROJECT_NAME}.desktop
  DESTINATION
  ${DESKTOP_APP_DIR}
)

# Install icon
install(
  FILES
  desktop/ros_additive_manufacturing_documentation.png
  DESTINATION
  ${DESKTOP_ICON_DIR}
)

install(
  DIRECTORY
  ${SPHINX_HTML_DIR}
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
  ${SPHINX_PDF_DIR}/${PROJECT_NAME}.pdf
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
