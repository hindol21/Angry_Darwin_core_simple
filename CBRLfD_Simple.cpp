/*
 * CBRLfD_Simple.cpp
 *
 * Created on: 2013. 7. 12.
 * Author: Hae Won Park
 * Description: Implementation for simplified CBRLfD routines.
 * Last modified: 2013. 7. 15.
 */

#include "CBRLfD_Simple.h"

using  std::cout;
using  std::endl;
using  std::string;
using  std::vector;
using  std::sort;
using  std::ifstream;
using  std::getline;
using  std::stringstream;
using  std::ifstream;
using  boost::lexical_cast;

CBRLfD::CBRLfD(){
    
    LoadXML(CBRLfD_CONFIG_FILE);
    
}

CBRLfD::~CBRLfD(){
    
    for(unsigned i=0; i < casebase.size(); i++)
        delete casebase[i];
    
}

//----------------------------------------------------------------------
// BuildCase() creates a case from a problem-solution pair.
//  void BuildCase(Case *newCase);
//  Case* BuildCase(Problem *p, Solution *s);
//  void BuildCase(int level, int round, int enemy, vector< float > location, int score, int xTouch, int yTouch);
//----------------------------------------------------------------------
void CBRLfD::BuildCase(Case *newCase){
    casebase.push_back(newCase);
    nIDGenerator++;
}


Case* CBRLfD::BuildCase(Problem *p, Solution *s){
    
    Case *newCase = new Case(p, s, nIDGenerator);
    
    casebase.push_back(newCase);
    nIDGenerator++;
	
	return newCase;
}

void CBRLfD::BuildCase(int level, int round, int enemy, vector< float > location, int score, int xTouch, int yTouch){
    
    Problem *p = new Problem();
    p->level = level;
    p->round = round;
    p->enemy = enemy;
    p->enemyLocation = location;
    p->score = score;
    
    Solution *s = new Solution();
    s->xTouch = xTouch;
    s->yTouch = yTouch;
    
    BuildCase(p, s);
}

//----------------------------------------------------------------------
// Implementation of CBR-4R steps
//      Retrieve: Distance between a new problem and problems of cases in the case base is computed and sorted using Distance()
//      Reuse: Builds a new solution from retrieved cases using gaussian weighting.
//      Revise: Builds a new case from newly created problem-solution pair.
//      Retain: Analyzes the new case and decides whether to retain the new case in case base.
//----------------------------------------------------------------------
caseVector CBRLfD::Retrieve(Problem *p){
    
    caseVector result = casebase;
    
    // Compute distance between p and problems of cases in casebase
    for(unsigned i=0; i<result.size(); i++)
        result[i]->distance = Distance(result[i]->mProblem, p);
    
    // Sort cases with their distance to p
    sort(result.begin(), result.end(), less_than_distance());
    
    // Print to log
#ifdef DEBUG
    stringstream sstm;
    sstm << "Given Problem: level(" << p->level << ") round(" << p->round << ") enemy(" << p->enemy << ") enemy locations (";
    
    for (int i=0; i< p->enemy; i++)
    	sstm << "(" << p->enemyLocation[2*i] << "," << p->enemyLocation[2*i+1] << ") ";
    
    sstm << ") score(" << p->score << ")\n" << endl;
    
    for(unsigned i=0; i<result.size(); i++){
        cout << "Case: " << i << " Sorted Distance to Problem: " << result[i]->distance << " Solution x: " << result[i]->mSolution->xTouch << " y: " << result[i]->mSolution->yTouch << endl;
        sstm << "Case: " << i << " Distance to Problem: " << result[i]->distance << endl;
        sstm << "Problem: level(" << result[i]->mProblem->level << ") round(" << result[i]->mProblem->round << ") enemy(" << result[i]->mProblem->enemy << ") enemy locations (";
        
        for (int j=0; j< result[i]->mProblem->enemy; j++)
            sstm <<"(" << result[i]->mProblem->enemyLocation[2*j] << "," << result[i]->mProblem->enemyLocation[2*j+1] << ") ";
        
        sstm << ") score(" << result[i]->mProblem->score << ")" << endl;
        sstm << "  Solution: x(" << result[i]->mSolution->xTouch << ") y(" <<result[i]->mSolution->yTouch << ")" << endl;
        
    }
    
    LOG::write_log(sstm.str());
#endif
    
    return result;
}

Solution* CBRLfD::Reuse(caseVector result){
	
	Solution* newSol = new Solution();
    
	float x = 0.0f;
	float y = 0.0f;
    
    float norm = 0.0f;
    
    // The most nearest 4 cases are used to generate a new solution.
    unsigned size = (result.size() <= 4) ? result.size() : 4;
    
    for(unsigned i=0; i<size; i++)
        norm += GAUSSIAN[i];
    
    for(unsigned i=0; i<size; i++){
        x += GAUSSIAN[i]/norm * result[i]->mSolution->xTouch;
        y += GAUSSIAN[i]/norm * result[i]->mSolution->yTouch;
    }
    
    
    newSol->xTouch = (int) x;
    newSol->yTouch = (int) y;
    
	return newSol;
    
}

Case* CBRLfD::Revise(Problem *p, Solution *s){
    
    // Difference from BuildCase(): newly created case is not stored in case base.
    Case *newCase = new Case(p, s, nIDGenerator);
    nIDGenerator++;
	
	return newCase;
}

void CBRLfD::Retain(Case *c){
    
    caseVector result = casebase;
    
    if (casebase.size() > 0){
        // Compute distance between c->mProblem and problems of cases in casebase
        for(unsigned i=0; i<result.size(); i++)
            result[i]->distance = Distance(result[i]->mProblem, c->mProblem);
        
        // Sort cases with their distance to p
        sort(result.begin(), result.end(), less_than_distance());
        
        if( result[0]->distance > RETAIN_T1 )
            if( result[0]->distance < RETAIN_T2 )
                casebase.push_back(c);
    }
}

// Parse XML
int CBRLfD::LoadXML(const char* filename){
    
    TiXmlDocument doc;
    
    //load file
    if ( !doc.LoadFile( filename ) ){
        cout << "No File" << endl;
        return 0;
    }
    
    //create xml handler
    TiXmlHandle docHandle( &doc );
    TiXmlHandle probHandle = docHandle.FirstChildElement( "Problem" ).FirstChildElement( "Feature" );

    //raw strings from xml are parsed.
    for( TiXmlElement* feature = probHandle.Element(); feature ; feature = feature->NextSiblingElement()){
        
        TiXmlElement* pElem =feature->FirstChildElement("Value");
        
        npValue.push_back(pElem->FirstChild()->ToText()->Value());
        npDataType.push_back(pElem->NextSiblingElement("Type")->FirstChild()->ToText()->Value());
        npMetric.push_back(pElem->NextSiblingElement("Metric")->FirstChild()->ToText()->Value());
        npVariable1.push_back(pElem->NextSiblingElement("Variable1")->FirstChild()->ToText()->Value());
        npVariable2.push_back(pElem->NextSiblingElement("Variable2")->FirstChild()->ToText()->Value());
        
        //weight is always a float
        npWeight.push_back(lexical_cast<float> (pElem->NextSiblingElement("Weight")->FirstChild()->ToText()->Value()));
    }

    AssignDistMetric();     //process Metric variables and assign pointers to distance metrics
    
    return 1;
}

// Assign pointer to distance metric for each feature.
int CBRLfD::AssignDistMetric(){
    
    //Distance-function vectors contain pointer to distance functions for each feature.
    //If the feature's data type does not match the distFunction vector, null distFunction is pushed back.
    const struct distFunction<int>                distFunc_null_i = {NULL, NULL, NULL};
    const struct distFunction<float>              distFunc_null_f = {NULL, NULL, NULL};
    const struct distFunction< vector<int> >      distFunc_null_iv = {NULL, NULL, NULL};
    const struct distFunction< vector<float> >    distFunc_null_fv = {NULL, NULL, NULL};
    
    for (unsigned i=0; i<npMetric.size(); i++){
        if (npMetric[i] == "Equal"){
            if (npDataType[i] == "int"){
                
                struct distFunction<int> D;
                
                float (*pFunc)(int,int,int*,int*) = distEqual<int>;
                
                D.pFunc = pFunc;
                
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null"))
                    D.var1 = new int(lexical_cast<int>(npVariable1[i]));
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new int(lexical_cast<int>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(D);
                npDistFunc_f.push_back(distFunc_null_f);
                npDistFunc_iv.push_back(distFunc_null_iv);
                npDistFunc_fv.push_back(distFunc_null_fv);
                
            } else if (npDataType[i] == "float"){
                
                struct distFunction<float> D;
                
                float (*pFunc)(float,float,float*,float*) = distEqual<float>;
                
                D.pFunc = pFunc;
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null"))
                    D.var1 = new float(lexical_cast<float>(npVariable1[i]));
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new float(lexical_cast<float>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(distFunc_null_i);
                npDistFunc_f.push_back(D);
                npDistFunc_iv.push_back(distFunc_null_iv);
                npDistFunc_fv.push_back(distFunc_null_fv);
                
            } else {
                cout << "Data type " << npDataType[i].c_str() << " is currently not supported for Metric <" << npMetric[i] << ">" << endl;
                exit(0);
            }
        } else if (npMetric[i] == "MaxValue"){
            if (npDataType[i] == "int"){
                
                struct distFunction<int> D;
                
                float (*pFunc)(int,int,int*,int*) = distMaxValue<int>;
                
                D.pFunc = pFunc;
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null"))
                    D.var1 = new int(lexical_cast<int>(npVariable1[i]));
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new int(lexical_cast<int>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(D);
                npDistFunc_f.push_back(distFunc_null_f);
                npDistFunc_iv.push_back(distFunc_null_iv);
                npDistFunc_fv.push_back(distFunc_null_fv);
                
                
            } else if (npDataType[i] == "float"){
                
                struct distFunction<float> D;
                
                float (*pFunc)(float,float,float*,float*) = distMaxValue<float>;
                
                D.pFunc = pFunc;
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null"))
                    D.var1 = new float(lexical_cast<float>(npVariable1[i]));
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new float(lexical_cast<float>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(distFunc_null_i);
                npDistFunc_f.push_back(D);
                npDistFunc_iv.push_back(distFunc_null_iv);
                npDistFunc_fv.push_back(distFunc_null_fv);
                
            } else {
                cout << "Data type " << npDataType[i].c_str() << " is currently not supported for Metric <" << npMetric[i] << ">" << endl;
                exit(0);
            }
        } else if (npMetric[i] == "MinValue"){
            if (npDataType[i] == "int"){
                
                struct distFunction<int> D;
                
                float (*pFunc)(int,int,int*,int*) = distMinValue<int>;
                
                D.pFunc = pFunc;
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null"))
                    D.var1 = new int(lexical_cast<int>(npVariable1[i]));
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new int(lexical_cast<int>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(D);
                npDistFunc_f.push_back(distFunc_null_f);
                npDistFunc_iv.push_back(distFunc_null_iv);
                npDistFunc_fv.push_back(distFunc_null_fv);
                
            } else if (npDataType[i] == "float"){
                
                struct distFunction<float> D;
                
                float (*pFunc)(float,float,float*,float*) = distMinValue<float>;
                
                D.pFunc = pFunc;
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null"))
                    D.var1 = new float(lexical_cast<float>(npVariable1[i]));
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new float(lexical_cast<float>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(distFunc_null_i);
                npDistFunc_f.push_back(D);
                npDistFunc_iv.push_back(distFunc_null_iv);
                npDistFunc_fv.push_back(distFunc_null_fv);
                
            } else {
                cout << "Data type " << npDataType[i].c_str() << " is currently not supported for Metric <" << npMetric[i] << ">" << endl;
                exit(0);
            }
        } else if (npMetric[i] == "MinVectorAvg"){
            if (npDataType[i] == "vector:int"){
                
                struct distFunction< vector<int> > D;
                
                float (*pFunc)(vector<int>,vector<int>,vector<int>*,vector<int>*) = distMinVectorAvg< int >;
                
                D.pFunc = pFunc;
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null")){
                    D.var1 = new vector<int>;
                    D.var1->push_back(lexical_cast<float>(npVariable1[i]));
                }
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new vector<int>(lexical_cast<int>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(distFunc_null_i);
                npDistFunc_f.push_back(distFunc_null_f);
                npDistFunc_iv.push_back(D);
                npDistFunc_fv.push_back(distFunc_null_fv);
                
                
            } else if (npDataType[i] == "vector:float"){
                struct distFunction< vector<float> > D;
                float (*pFunc)(vector<float>,vector<float>,vector<float>*,vector<float>*) = distMinVectorAvg< float >;
                
                D.pFunc = pFunc;
                if((npVariable1[i] != "n/a") && (npVariable1[i] != "null")){
                    D.var1 = new vector<float>;
                    D.var1->push_back(lexical_cast<float>(npVariable1[i]));
                }
                else
                    D.var1 = NULL;
                if((npVariable2[i] != "n/a") && (npVariable2[i] != "null"))
                    D.var2 = new vector<float>(lexical_cast<float>(npVariable2[i]));
                else
                    D.var2 = NULL;
                
                npDistFunc_i.push_back(distFunc_null_i);
                npDistFunc_f.push_back(distFunc_null_f);
                npDistFunc_iv.push_back(distFunc_null_iv);
                npDistFunc_fv.push_back(D);
                
            } else {
                cout << "Data type " << npDataType[i].c_str() << " is currently not supported for Metric <" << npMetric[i] << ">" << endl;
                exit(0);
            }
            
        }
        
        
    }
    
    //    //test
    //    for(unsigned i=0; i < npWeight.size(); i++){
    //
    //        float result = 1.0f;//npWeight[i];
    //
    //        vector< int > vI;
    //        vI.push_back(3); vI.push_back(3); vI.push_back(5); vI.push_back(6);
    //        vector< int > vII;
    //        vII.push_back(3); vII.push_back(3); vII.push_back(5); vII.push_back(6);
    //
    //        vector< float > vIII;
    //        vIII.push_back(773.99884033203); vIII.push_back(250.55619812012); vIII.push_back(823.2060546875); vIII.push_back(23.611347198486);
    //        vector< float > vIV;
    //        vIV.push_back(773.99884033203); vIV.push_back(250.55619812012); vIV.push_back(823.2060546875); vIV.push_back(23.611347198486);
    //
    //        if((*npDistFunc_i[i].pFunc)){
    //            result *= (*npDistFunc_f[i].pFunc)(17135, 17135, npDistFunc_i[i].var1, npDistFunc_i[i].var2);
    //        } else if((*npDistFunc_f[i].pFunc)){
    //            result *= (*npDistFunc_f[i].pFunc)(3.123, 2.432, npDistFunc_f[i].var1, npDistFunc_f[i].var2);
    //        } else if((*npDistFunc_iv[i].pFunc)){
    //            result *= (*npDistFunc_iv[i].pFunc)(vI, vII, npDistFunc_iv[i].var1, npDistFunc_iv[i].var2);
    //        } else if((*npDistFunc_fv[i].pFunc)){
    //            result *= (*npDistFunc_fv[i].pFunc)(vIII, vIV, npDistFunc_fv[i].var1, npDistFunc_fv[i].var2);
    //        }
    //
    //        cout << result << endl;
    //
    //    }
    
    return 1;
}

// Nearest-neighbor distance function used for case retrieval.
float CBRLfD::Distance(Problem *p1, Problem *p2){
    
    float total = 0.0f;
    
    int i = 0;
    total += npWeight[i]*(*npDistFunc_i[i].pFunc)(p1->level, p2->level, npDistFunc_i[i].var1, npDistFunc_i[i].var2);
    
    i++;
    total += npWeight[i]*(*npDistFunc_i[i].pFunc)(p1->round, p2->round, npDistFunc_i[i].var1, npDistFunc_i[i].var2);
    
    i++;
    total += npWeight[i]*(*npDistFunc_i[i].pFunc)(p1->enemy, p2->enemy, npDistFunc_i[i].var1, npDistFunc_i[i].var2);
    
    i++;
    total += npWeight[i]*(*npDistFunc_fv[i].pFunc)(p1->enemyLocation, p2->enemyLocation, npDistFunc_fv[i].var1, npDistFunc_fv[i].var2);
    
    i++;
    total += npWeight[i]*(*npDistFunc_i[i].pFunc)(p1->score, p2->score, npDistFunc_i[i].var1, npDistFunc_i[i].var2);
    
    return total;
    
}
