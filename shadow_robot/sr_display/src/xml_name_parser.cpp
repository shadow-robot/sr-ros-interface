/**
*
* @file   xml_name_parser.cpp
* @author Nicolas Goossaert <nicolas@shadowrobot.com>
* @date   Wed Jun  23 14:18:51 2010
* @brief This parser is a gateway between the
* links in the hand and the joints related to those links.
* As the formalism is not the same for the links and the joints, 
* it remaps the names so that they can be sent in the correct
* formalism.
*
* @todo finish this parser.
* 
* The names file should have that form:
*
*<NamesRemapper>
*  <Name joint="MFJ0" link="mfmiddle" />
*  <Name joint="MFJ3" link="mfproximal" />
*  <Name joint="MFJ4" link="mfknuckle" />
*</NamesRemapper>
*
*/



//ROS include
#include <ros/ros.h>

#include "cyberglove/xml_name_parser.h"

#include <stdio.h>

#include <string>
#include <vector>

namespace xml_name_parser{

  XmlCNameParser::XmlNameParser(string path_to_file){
    TiXmlDocument doc(path_to_file.c_str());
    bool loadOkay = doc.LoadFile();
    if (loadOkay)
      {
	ROS_DEBUG("loading names file %s", path_to_file.c_str());
	parse_xml_file( doc.RootElement() );
      }
    else
      {
	ROS_ERROR("Failed to load file \"%s\"", path_to_file.c_str());
      }
  }
  
  void XmlNameParser::parse_xml_file( TiXmlNode* pParent ){
    if ( !pParent ) return;
    
    TiXmlElement* child = pParent->FirstChildElement("Name");

    //no Joint elements => error
    if( !child )
      {
	ROS_ERROR( "The file seems to be broken: there's no Name elements." );
	return;
      }
						  
    bool has_sibling = true;
    while( has_sibling )
      {
	// a Name element was found
	string link = child->Attribute("link");
	FinalNames.push_back(link);

	//get the next Joint element
	child = child->NextSiblingElement("Name");
	//no more Joint elements => stop
	if( !child )
	    has_sibling = false;
      }
  }
  
  
  
  
} //end namespace