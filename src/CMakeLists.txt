# Copyright 2013, Francois Keith, JRL, CNRS/AIST

INCLUDE(../cmake/python.cmake)
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Add the features
SET(CONTROLLER_NAME rh_cop_interface)
ADD_LIBRARY(${CONTROLLER_NAME}
  SHARED
  new_operator.cpp
)


# Link the dynamic library containing the SoT with its dependencies.
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")


IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
ENDIF(UNIX AND NOT APPLE)


INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

# build python submodule
STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${CONTROLLER_NAME})
SET(NEW_ENTITY_CLASS ${ENTITIES})
DYNAMIC_GRAPH_PYTHON_MODULE("sot/rh_cop_interface/${PYTHON_LIBRARY_NAME}"
  ${CONTROLLER_NAME}
  new_operator-${PYTHON_LIBRARY_NAME}-wrap
  )


# Install empty __init__.py files in intermediate directories.
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/rh_cop_interface/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/rh_cop_interface
  )

