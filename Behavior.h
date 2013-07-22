/*
 * Behavior.h
 *
 * Created on: 2013. 4. 11.
 * Author: Hae Won Park
 * Description: Declarations for robot behavior generation
 * Last modified: 2013. 6. 5.
 */
#ifndef _BEHAVIOR_MODULE_H_
#define _BEHAVIOR_MODULE_H_

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <cstdlib>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

using std::vector;
using std::string;

// Robot behavior consists of gesture-speech primitive pairs.
typedef vector< int > GestureSet;
typedef vector< string > SpeechSet;

namespace Robot
{
    class Behavior{
        
    public:
        
        //Behavior categories. Gesture and speech primitives are divided into these categories.
        enum BehaviorSet{
            STARTUP,
            VICTORY,
            LOST,
            IDLE,
            NEUTRAL,
            AIM,
            SHOOT
        };
        
        Behavior();
        ~Behavior();
        
        //Return Behavior instance
        static Behavior* GetInstance() { return mBehavior; }
        
        //Retrieve gesture group g
        GestureSet RetrieveGestureSet(BehaviorSet g);
        //Retrieve random gesture from gesture group g
        int RetrieveRandomGesture(BehaviorSet g);
        
        //Retrieve speech group s
        SpeechSet RetrieveSpeechSet(BehaviorSet s);
        //Retrieve random gesture from speech group s
        const char* RetrieveRandomSpeech(BehaviorSet s);
        
    private:
        
        //Behavior instance
        static Behavior *mBehavior;
        
        //2-dimensional gesture and speech group vectors
        vector< GestureSet > GestureGroup;
        vector< SpeechSet > SpeechGroup;
        
        //Construct behavior group from file.
        //Each line in file contains "{Behavior_Group, motion_id/sound_file_path}"
        //Motions are defined in 'motion_1024.bin' and sound files are in sound folder.
        int ConstructBehaviorGroup(const char* filename);
        vector< string > ParseLine(string line);
        int ParseID(string idstring);
        
    };
}

#endif