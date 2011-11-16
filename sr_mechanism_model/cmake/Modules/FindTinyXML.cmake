# - Find TinyXML
# Find the native TinyXML includes and library
#
#   TINYXML_FOUND       - True if TinyXML found.
#   TINYXML_INCLUDE_DIR - where to find tinyxml.h, etc.
#   TINYXML_LIBRARIES   - List of libraries when using TinyXML.
#

IF( TINYXML_INCLUDE_DIR )
    # Already in cache, be silent
    SET( TinyXML_FIND_QUIETLY TRUE )
ENDIF( TINYXML_INCLUDE_DIR )

find_path( TINYXML_INCLUDE_DIR tinyxml.h
	 PATHS /opt/ros/diamondback/stacks/common/tinyxml/include/tinyxml /opt/ros/common/tinyxml/include/tinyxml )
#           PATH_SUFFIXES tinyxml )

FIND_LIBRARY( TINYXML_LIBRARIES
              NAMES tinyxml
	      PATHS /opt/ros/common/tinyxml /opt/ros/diamondback/stacks/common/tinyxml
              PATH_SUFFIXES tinyxml )

# handle the QUIETLY and REQUIRED arguments and set TINYXML_FOUND to TRUE if
# all listed variables are TRUE
include( FindPackageHandleStandardArgs )
Find_Package_Handle_Standard_Args( TinyXML "Did not find TinyXML" TINYXML_INCLUDE_DIR TINYXML_LIBRARIES )

MARK_AS_ADVANCED( TINYXML_INCLUDE_DIR TINYXML_LIBRARIES )
