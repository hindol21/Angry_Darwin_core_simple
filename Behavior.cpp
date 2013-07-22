/*
 * Behavior.cpp
 *
 * Created on: 2013. 4. 11.
 * Author: Hae Won Park
 * Description: Implementation for robot behavior generation
 * Last modified: 2013. 6. 5.
 */
#include "Behavior.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::ifstream;
using std::getline;
using namespace Robot;

//Declare static instance
Behavior* Behavior::mBehavior = new Behavior();

Behavior::Behavior(){
    
    srand(time(NULL));
    ConstructBehaviorGroup("./Behavior.asc"); //Construct behavior group from file.
}

Behavior::~Behavior(){
    
}

//Retrive behavior group and motion_id/sound_file_path
//Each line in file contains "{Behavior_Group, motion_id/sound_file}"
vector< string > Behavior::ParseLine(string line)
{
    vector< string > SplitVec;
	
    boost::algorithm::trim_if( line, boost::is_any_of("{}") );
	boost::algorithm::split( SplitVec, line, boost::is_any_of("{},\t\" "), boost::token_compress_on );
    
    return SplitVec;
}

//Assign Behavior ID
int Behavior::ParseID(string idstring)
{
    int id = -1;
    
    if(idstring == "G_STARTUP")
        id = 0;
    else if(idstring == "G_VICTORY")
        id = 1;
    else if(idstring == "G_LOST")
        id = 2;
    else if(idstring == "G_IDLE")
        id = 3;
    else if(idstring == "G_NEUTRAL")
        id = 4;
    else if(idstring == "G_AIM")
        id = 5;
    else if(idstring == "G_SHOOT")
        id = 6;
    else if(idstring == "V_STARTUP")
        id = 7;
    else if(idstring == "V_VICTORY")
        id = 8;
    else if(idstring == "V_LOST")
        id = 9;
    else if(idstring == "V_IDLE")
        id = 10;
    else if(idstring == "V_NEUTRAL")
        id = 11;
    else if(idstring == "V_AIM")
        id = 12;
    else if(idstring == "V_SHOOT")
        id = 13;
    
    return id;
}

int Behavior::ConstructBehaviorGroup(const char* data)
{
    string line;
    vector< string > splitline;
    
    GestureGroup.resize(SHOOT+1);
    SpeechGroup.resize(SHOOT+1);
    
    ifstream infile(data);
    
    while(getline(infile, line))
    {
        splitline=ParseLine(line);  //Retrive behavior group and motion_id/sound_file_path
        
        if(!splitline.empty())
        {
            //Retrieve behavior group id
            int id = ParseID(splitline[0]);
            
            //Store motion_id and sound_file_path
            if (id > -1){
                if(id < 7)
                    GestureGroup[id].push_back(atoi(splitline[1].c_str()));
                else
                    SpeechGroup[id-7].push_back(splitline[1]);
            }
        }
    }
    
    return 0;
}

//Retrieve gesture group g
GestureSet Behavior::RetrieveGestureSet(BehaviorSet g){
    
    return GestureGroup[g];
}

//Retrieve random gesture from gesture group g
int Behavior::RetrieveRandomGesture(BehaviorSet g){
    
    int r = rand() % GestureGroup.at(g).size();
    
    return GestureGroup.at(g).at(r);
}

//Retrieve speech group s
SpeechSet Behavior::RetrieveSpeechSet(BehaviorSet s){
    
    return SpeechGroup[s];
}

//Retrieve random gesture from speech group s
const char* Behavior::RetrieveRandomSpeech(BehaviorSet s){
    
    int r = rand() % SpeechGroup.at(s).size();
    
    return SpeechGroup.at(s).at(r).c_str();
}



