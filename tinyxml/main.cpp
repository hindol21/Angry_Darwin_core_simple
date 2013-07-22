/*
   Test program for TinyXML.
*/


#ifdef TIXML_USE_STL
	#include <iostream>
	#include <sstream>
	using namespace std;
#else
	#include <stdio.h>
#endif


#include "tinyxml.h"
#include <iostream>

int main(){
    
    TiXmlDocument doc;
    if ( !doc.LoadFile( "CBR-LfD.xml" ) ){
        std::cout << "No File" << std::endl;
        return 0;
    }
    
    TiXmlHandle docHandle( &doc );
    TiXmlHandle probHandle = docHandle.FirstChildElement( "Problem" ).FirstChildElement( "Feature" );

    for( TiXmlElement* feature = probHandle.Element(); feature ; feature = feature->NextSiblingElement()){
    
        TiXmlElement* pElem =feature->FirstChildElement("Type");
        std::cout << pElem->FirstChild()->ToText()->Value()
        << pElem->NextSiblingElement("Weight")->FirstChild()->ToText()->Value()<< std::endl;
   // std::cout << doc.FirstChildElement()->FirstChildElement()->FirstChild()->ToText()->Value() << std::endl;
    }
    

    
}
