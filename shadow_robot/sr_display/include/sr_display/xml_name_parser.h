/**
 * @file   xml_name_parser.h
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date    Wed Jun  23 14:17:15 2010
 * 
 * @brief  header of the xml_name_parser.cpp file
 * 
 * 
 * 
 */


#ifndef   	XML_NAME_PARSER_H_
# define   	XML_NAME_PARSER_H_

//xml parser library
#include <tinyxml/tinyxml.h>

//generic C/C++ include
#include <string>
#include <vector>
#include <map>

using namespace std;

namespace xml_name_parser{

class XmlNameParser
{
 public:
  XmlNameParser(){};
  XmlNameParser(string path_to_file);
  ~XmlNameParser(){};
  
  std::vector<string> mapNames();

 protected:
  void parse_xml_file( TiXmlNode* pParent );
  vector<string> parse_names( TiXmlNode* pParent );

  /// The vector containing the names to display in Rviz
  vector<string> FinalNames;

}; // end class XmlNameParser

} // end namespace
#endif 	    /* !XML_NAME_PARSER_H_ */