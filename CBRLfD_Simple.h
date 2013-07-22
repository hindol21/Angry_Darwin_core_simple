/*
 * CBRLfD_Simple.h
 *
 * Created on: 2013. 7. 12.
 * Author: Hae Won Park
 * Description: Declarations for simplified CBRLfD routines. 
 *  Case indexing and case-base maintenance using ANN (approximate nearest neighbor searching) is omitted.
 *  Recommended for use with small-size dataset.
 * Last modified: 2013. 7. 14.
 */

/*
 * CBR-LfD provides framework for case-based reasoning (CBR) approach to Learning from Demonstration (LfD).
 *  LfD during human-robot social interaction provides solution to the following problems of CBR:
 *  - Auto-population of the case base: Demonstrations are converted into cases and stored in memory.
 *  - The data-driven problem of CBR and other lazy-learning methods: Data-driven methods always face a 
 *      challenge of how well their data span the problem space. Real-time LfD observes the learners
 *      performance in real-time and provides necessary addition or modification to the case base. This is
 *      possible because lazy learners store "raw" data.
 *
 *	CBRLfD class provides tools for extracting case-feature information from CBRLfD_Simple.xml,
 *      a list of basic distance-measure functions, and case retrieval and reuse methods.
 *      (For training feature weights, refer to Humanoids 2013 paper and matlab code in a separate folder.)
 *
 */

#ifndef _CBRLFD_MODULE_H_
#define _CBRLFD_MODULE_H_

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <algorithm> 
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "tinyxml.h"
#include "Log.h"

#define CBRLfD_CONFIG_FILE  "./CBRLfD_Simple.xml"

#define RETAIN_T1  0.2          // low similarity score for retaining
#define RETAIN_T2  0.8          // high similarity score for retaining

using std::string;
using std::vector;


//----------------------------------------------------------------------
//	FEATURE_TYPES
//		Data types supported for representing case features. They are
//		configurable by the user through "CBRLfD_Simple.xml".
//
//  FEATURE_TYPES       Data Type
//  -------------       ---------
//  INT                 int
//  FLOAT               float
//  INT_VECTOR          vector< int >
//  FLOAT_VECTOR        vector< float >
//  STRING              string
//  STRING_VECTOR       vector< string >
//----------------------------------------------------------------------
enum FEATURE_TYPES {
	INT,
	FLOAT,
    STRING,
    INT_VECTOR,
    FLOAT_VECTOR,
    STRING_VECTOR
};

//Gaussian weighting coefficients. 
const float GAUSSIAN[] = {0.398942322, 0.24197075, 0.053990972, 0.004431849};

//----------------------------------------------------------------------
//  A Case consists of a Problem descriptor and a Solution descriptor.
//  Following are the problem and solution descriptors for Angry Darwin application.
//----------------------------------------------------------------------

struct Problem{
    int level;
    int round;
    int enemy;
    vector< float > enemyLocation;
    int score;
};

struct Solution{
    int xTouch;
    int yTouch;
};

//----------------------------------------------------------------------
//  Case
//      Following are the problem and solution descriptors for Angry Darwin application.
//----------------------------------------------------------------------
struct Case{
    
    int ID;
    
    vector< string > vProblem;
    vector< string > vSolution;
    
    Problem *mProblem;
    Solution *mSolution;
    
    float distance;
    
    Case(void){
        mProblem = NULL;
        mSolution = NULL;
    }
    
    Case(vector< string > problem, vector< string > solution, int idNum){
        vProblem = problem;
        vSolution = solution;
        ID = idNum;
    }
    
    Case(Problem *problem, Solution *solution, int idNum){
        mProblem = problem;
        mSolution = solution;
        ID = idNum;
    }
    
};

//----------------------------------------------------------------------
// less_than_distance(): distance comparison for sorting cases.
//  input: two cases
//  output: returns less distance of the two.
//----------------------------------------------------------------------
struct less_than_distance
{
    inline bool operator() (const Case* c1, const Case* c2)
    {
        return (c1->distance < c2->distance);
    }
};


typedef vector< Case* > caseVector;

//----------------------------------------------------------------------
//  distFunction
//      distFunction consists of a pointer to a distance-metric method,
//      and pointers to two optional variables. A distFunction is declared for
//      each case feature.
//
//  Distance-Metric methods
//      Distance between the two variables v1 and v2 is returned.
//      Distance values are normalized and ranges from 0 to 1.
//----------------------------------------------------------------------
template <class T>
struct distFunction{
    float (*pFunc)(T a, T b, T *v1, T *v2); //pointer to a distance-metric function
    T *var1;                                //optional variable 1
    T *var2;                                //optional variable 2
};

//----------------------------------------------------------------------
//  distEqual()
//      Feature selection method. Returns 1 when a == b, and 0 when a != b.
//----------------------------------------------------------------------
template <class T>
float distEqual (T a, T b, T *var1, T *var2) {
    if ( abs(a - b) < 5.96e-08 ) return 0;
    else return 1.0f;
}

// operator - definition for string data types. Used with distEqual()
inline float operator-(string & s1, const string & s2) {
    if( s1 == s2 ) return 0;
    else return 1;
}

//----------------------------------------------------------------------
//  distMaxValue()
//      Returns the absolute distance between a and b divided by the maximum value defined in var1.
//      Used when the maximum range of |a-b| is uncertain in which var1 is used as a normalization factor.
//----------------------------------------------------------------------
template <class T>
float distMaxValue (T a, T b, T *var1, T *var2){
    
    if ( abs((*var1) - 0) < 5.96e-08 ) return 1.0f;     //if var1 is 0, return the highest distance.
    
    float diff =  (float) (a - b)/(*var1);
    if(diff < 0) diff = -1 * diff;
    
    if(diff > 1.0f)	return 1.0f;
    else return diff;
}

//----------------------------------------------------------------------
//  distMinValue()
//      Distance is smaller when a is further apart from the minimum value var1.
//----------------------------------------------------------------------
template <class T>
float distMinValue (T a, T b, T *var1, T *var2){
    if( abs((*var1) - a) < 5.96e-08 )               //if a == var1, return the highest distance.
        return 1.0f;
    
    float diff = (float) (*var1) / (a - (*var1));
    if(diff < 0) diff = -1 * diff;
    
    return diff;

}

//----------------------------------------------------------------------
//  distMinVectorAvg()
//      For every (x,y) pair in vector b, find the minimum distance to vectors in a.
//----------------------------------------------------------------------
template <class T>
vector<float> distMinVector (vector<T> a, vector<T> b){
    vector<float> d;
    
    for(unsigned i=0; i<b.size()/2; i++)
		d.push_back(-1);
    
	for(unsigned i=0; i < a.size()/2; i++){
		for(unsigned j=0; j < b.size()/2; j++){
			float delta = (a[2*i]-b[2*j])*(a[2*i]-b[2*j])+(a[2*i+1]-b[2*j+1])*(a[2*i+1]-b[2*j+1]);
			if((delta < d[j]) || (d[j] < 0))
				d[j] = delta;
		}
	}
    
    
	return d;
}

template <class T>
float distMinVectorAvg (vector<T> a, vector<T> b, vector<T> *var1, vector<T> *var2){
	vector<float> minD = distMinVector <T> (a, b);
	float avg = 0.0;
	
	for(unsigned i=0; i<minD.size(); i++){
		float d = minD[i]/var1->at(0);
		if (d > 1.0f)
			avg += 1.0f;
		else
			avg += d;
	}
	
	avg /= minD.size();
	
	return avg;
}

//----------------------------------------------------------------------
//  CBRLfD
//      1. Load XML: Constructs case-feature structures by synthesizing the xml file.
//      2. Build case base: Stores incoming cases in case base.
//      3. Retrieve: A retrieval function is created in #1. Returns a root node of a kdtree.
//              (Simplified CBRLfD uses brute-force search instead of kdtree and returns a sorted case vector.)
//      4. Reuse: Builds a new solution from retrieved cases using gaussian weighting.
//      5. Revise: Builds a new case from newly created problem-solution pair.
//      6. Retain: Analyzes the new case and decides whether to retain the new case in case base.
//----------------------------------------------------------------------
class CBRLfD{
    
public:
    CBRLfD();
    ~CBRLfD();
    
	caseVector	casebase;       //case base for storing cases
    static int nIDGenerator;    //keeps tract of the case id sequence.
    
    // BuildCase() creates a case from a problem-solution pair. 
    void BuildCase(Case *newCase);
    Case* BuildCase(Problem *p, Solution *s);
    void BuildCase(int level, int round, int enemy, vector< float > location, int score, int xTouch, int yTouch);
    
    // Implementation of CBR-4R steps
    caseVector Retrieve(Problem *p);            //Cases in the case base is sorted using Distance()
    Solution* Reuse(caseVector result);         //Builds a new solution from retrieved cases using gaussian weighting.
    Case* Revise(Problem *p, Solution *s);      //Builds a new case from newly created problem-solution pair.
    void Retain(Case *c);                        //Analyzes the new case and decides whether to retain the new case in case base.

    
private:
            
    // Raw incoming variables from xml. The size of each vector is the number of case features.
    vector < string > npValue;                  //<Value>: name of the feature
    vector < string > npDataType;               //<Type>: data type of the feature
    vector < string > npMetric;                 //<Metric>: suggested distance metric for the feature
    vector < string > npVariable1;              //<Variable1>: optional variable for distance function
    vector < string > npVariable2;              //<Variable2>: optional variable for distance function
    vector < float >  npWeight;                 //<Weight>: feature weight. Significant features have larger weights. 

    // Distance-function vector for each data type. The size of each vector is the number of case features.
    vector < distFunction <int> > npDistFunc_i;
    vector < distFunction <float> > npDistFunc_f;
    vector < distFunction < vector<int> > > npDistFunc_iv;
    vector < distFunction < vector<float> > > npDistFunc_fv;
    
    int LoadXML(const char* filename);              // Parse XML
    int AssignDistMetric();                         // Assign pointer to distance metric for each feature.
    float Distance(Problem *p1, Problem *p2);       // Nearest-neighbor distance function used for case retrieval.
};

#endif

