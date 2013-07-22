/*
 * main.h
 *
 * Created on: 2013. 7. 12.
 * Author: Hae Won Park
 * Description: Implementation of Angry Darwin application using CBR-LfD.
 * Last modified: 2013. 7. 15.
 */


#include <unistd.h>
#include <libgen.h>
#include <termios.h>

#include "main.h"

using namespace std;
using namespace Robot;

int CBRLfD::nIDGenerator = 1;       //initialize static variable

AngryDarwin::AngryDarwin(){
    
    //////////////////// Socket Initialize ////////////////////////////
    ftsockfd=socket(AF_INET,SOCK_DGRAM,0);
	ttsockfd=socket(AF_INET,SOCK_DGRAM,0);
    
    bzero(&addr1,sizeof(addr1));
    addr1.sin_family = AF_INET;
    addr1.sin_addr.s_addr=htonl(INADDR_ANY);
    addr1.sin_port=htons(FROM_TABLET_PORT);
    bind(ftsockfd,(struct sockaddr *)&addr1,sizeof(addr1));
    
	bzero(&addr2,sizeof(addr2));
    addr2.sin_family = AF_INET;
    addr2.sin_addr.s_addr=inet_addr(TABLET_IP);
    addr2.sin_port=htons(TO_TABLET_PORT);
    bind(ttsockfd,(struct sockaddr *)&addr2,sizeof(addr2));
    
    //////////////////// Framework Initialize ////////////////////////////
    change_current_dir();
    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
    
    linux_cm730 =  new LinuxCM730("/dev/ttyUSB0");
    cm730 = new CM730(linux_cm730);
    
    if(MotionManager::GetInstance()->Initialize(cm730) == false)
        cout << "Fail to initialize Motion Manager!" << endl;
    
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    
    motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    
    MotionManager::GetInstance()->SetEnable(true);
    
    Action::GetInstance()->Start(85);    // Init(sit down) pose
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
    //////////////////////////////////////////////////////////////////////
    
    mCBR = new CBRLfD();
    
	state = STATE_ROUND_READY;
    prevstate = state;
    
    prevStatePacket.resize(2);
	curCase = NULL;
    
    bIdle = true;
    idlecount = 0;
    srand(time(NULL));
    timeout = rand() % IDLE_TIMEOUT + IDLE_TIMEOUT_OFFSET;
    
    //initialize idle thread
    bSuspendSubThread = true;
    pthread_create(&thread_t, NULL, Idle_thread, this);
    
}

AngryDarwin::~AngryDarwin(){
    
    cout<<"kill AngryDarwin"<<endl;
    
    if(thread_t)
        pthread_exit(0);
    
    if(mCBR)
        delete mCBR;
    if(linux_cm730)
        delete linux_cm730;
    if(cm730)
        delete cm730;
    if(motion_timer)
        delete motion_timer;
    if(curCase)
        delete curCase;
    
}

//Execute finite-state machine(FSM)
void AngryDarwin::RunStateMachine(){
    
    //Receive task-status packet from tablet
    len = sizeof(ftaddr);
    n = recvfrom(ftsockfd,mesg,1000,0,(struct sockaddr *)&ftaddr,&len);
    mesg[n] = 0;
    
    //Split packet to string vector
    packet = SplitPacket(mesg);
    
    //Received packet either starts with "state" for task status, and "usertouch" for touch event.
    if(packet.at(0)=="state"){
        
        prevStatePacket = packet;
        if(packet.at(5)=="trans_new_round")
            state = STATE_ROUND_READY;
    }
    
    //If FSM is stuck on some state, increase idle count.
    //If the count exceeds timeout, enter idle thread.
    if(prevstate == state) idlecount++;
    else    idlecount = 0;
    
    if (idlecount > timeout){
        state = STATE_IDLE;
        idlecount = 0;
        bIdle = true;
        
        timeout = rand() % 15 + 7;
    }
    
    prevstate = state;
    
    cout << "Current State: " << state << ", Received the following: " << mesg << endl;
    cout << "Case-base Size: " << mCBR->casebase.size() << "\n" << endl;
    
    
#ifdef DEBUG
    LOG::write_log(mesg);
#endif
    
    switch (state){
            
            //Initial task state
        case STATE_ROUND_READY:{
            
            bSuspendSubThread = true;       //suspend idle thread
            while(Action::GetInstance()->IsRunning()) usleep(8*1000);   //give robot some time to finish its motion
            
            //Restarting from idle state
            if(bIdle){
                
                //Produce start up behavior
                Action::GetInstance()->Start(Behavior::GetInstance()->RetrieveRandomGesture(Behavior::STARTUP));
                LinuxActionScript::PlayMP3(Behavior::GetInstance()->RetrieveRandomSpeech(Behavior::STARTUP));
                
                while(Action::GetInstance()->IsRunning()) usleep(8*1000);   //give robot some time to finish its motion
                
                bIdle = false;
            }
            
            curCase = NULL;
            
            Action::GetInstance()->Start(85);                           //default sitting pose
            while(Action::GetInstance()->IsRunning()) usleep(8*1000);   //give robot some time to finish its motion
            
            //If packet starts with "usertouch", the user interrupted and is attempting to provide demonstration.
            if(packet.at(0)=="usertouch"){
                int x = atoi(packet.at(3).c_str());     //extract x-coordination of touch event
                int y = atoi(packet.at(4).c_str());     //extract y-coordination of touch event
                
                //Check if touch event is inside tablet touch limit
                if((x <= xHighLimit) && (x >= xLowLimit) && (y <=yHighLimit) && (y >= yLowLimit)){
                    cout << "=================================================================================" << endl;
                    cout << "Demonstration Recording Begin" << endl;
                    
#ifdef DEBUG
                    LOG::write_log("[Demonstration Recording Begin]");
#endif
                    
                    if(curCase) curCase = NULL;
                    
                    //Create problem description from packet
                    curCase = buildCase(prevStatePacket, packet);
                    
                    state = STATE_ROUND_END;
                    bSuspendSubThread = false;
                    
                }
            }
            //If packet starts with "state", regular task-status is received.
            else if(packet.at(0)=="state"){
                if(packet.at(5)=="state_new_round"){
                    state = STATE_AIM;
                }
                else if((packet.at(5)=="state_end_round") || (packet.at(5)=="trans_call_round")){
                    state = STATE_ROUND_END;
                }
                else if(packet.at(5)=="state_end_game")
                    state = STATE_GAME_END;
                
            }
            
            break;
        }
            
        case STATE_AIM:{
            
            bSuspendSubThread = true;
            
            //User is attempting to provide demonstration. Start recording demonstration
            if(packet.at(0)=="usertouch"){
                
                int x = atoi(packet.at(3).c_str());
                int y = atoi(packet.at(4).c_str());
                
                if((x <= xHighLimit) && (x >= xLowLimit) && (y <=yHighLimit) && (y >= yLowLimit)){
                    cout << "\n=================================================================================" << endl;
                    cout << "Demonstration Recording Begin" << endl;
#ifdef DEBUG
                    LOG::write_log("[Demonstration Recording Begin]");
#endif
                    if(curCase) curCase = NULL;
                    
                    curCase = buildCase(prevStatePacket, packet);
                    
                    state = STATE_ROUND_END;
                    bSuspendSubThread = false;
                }
            }
            else if(packet.at(0)=="state"){
                
                char command[1000] = { 0 };
                
                //Build problem description from packet
                Problem *prob = buildProblem(packet);
                Solution *sol = new Solution();
                
                //RETRIEVE the nearest cases from the new problem
                caseVector result = mCBR->Retrieve(prob);
                
                //If no case is retrieved, start self training
                if(result.size() == 0){
                    
                    xCoord = rand() % 200;
                    yCoord = rand() % 200 + 95;
                    
                    sol->xTouch = xCoord;
                    sol->yTouch = yCoord;
                    
                    if (SELF_TRAIN){
                        cout << "\n=================================================================================" << endl;
                        cout << "SELF TRAINING Recording Begin" << endl;
#ifdef DEBUG
                        LOG::write_log("[Self Training Recording Begin]");
#endif
                        
                        if(curCase) curCase = NULL;
                        
                        curCase = buildCase(prob, sol);
                    }
                }
                //Create new solution
                else{
                    
                    //REUSE: create new solution using retrieved solutions
                    Solution *newSol = mCBR->Reuse(result);
                    
                    xCoord = newSol->xTouch;
                    yCoord = newSol->yTouch;
                }
                
                //Compute embodiment joint mapping for aiming
                ComputeAim(xCoord, yCoord, motion_timer);
                
                //Turn eyes red: shows robot is in aiming state
                cm730->WriteWord(CM730::P_LED_HEAD_L, CM730::MakeColor(250,0,0), 0);
                cm730->WriteWord(CM730::P_LED_EYE_L, CM730::MakeColor(250,0,0), 0);
                
                
                Action::GetInstance()->Start(80);       //start aiming motion
                while(Action::GetInstance()->IsRunning()) usleep(8*1000);
                
                //Send touch event command to tablet
                sprintf(command, "touch %d %d\n", xCoord, yCoord);
                sendto(ttsockfd,command,strlen(command),0,(struct sockaddr *)&addr2,sizeof(addr2));
                
                cout << "Sent the following: " << command << endl;
                
                state = STATE_SHOOT;
                usleep(500000);
            }
            break;
        }
        case STATE_SHOOT:{
            
            //User is attempting to provide demonstration. Start recording demonstration
            if(packet.at(0)=="usertouch"){
                
                Action::GetInstance()->Start(85);
                while(Action::GetInstance()->IsRunning()) usleep(8*1000);
                bSuspendSubThread = false;
                
                int x = atoi(packet.at(3).c_str());
                int y = atoi(packet.at(4).c_str());
                if((x <= xHighLimit) && (x >= xLowLimit) && (y <=yHighLimit) && (y >= yLowLimit)){
                    
                    cout << "\n=================================================================================" << endl;
                    cout << "Demonstration Recording Begin" << endl;
                    
#ifdef DEBUG
                    LOG::write_log("[Demonstration Recording Begin]");
#endif
                    if(curCase) curCase = NULL;
                    
                    curCase = buildCase(prevStatePacket, packet);
                    
                    state = STATE_ROUND_END;
                    
                }
            }
            else if(packet.at(0)=="state"){
                if(packet.at(5)=="state_aiming_shot"){
                    
                    Action::GetInstance()->Start(93);       //start shooting motion
                    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
                    
                    //Send release command to tablet
                    char command[1000] = { 0 };
                    sprintf(command, "release %d %d\n", xCoord, yCoord);
                    sendto(ttsockfd,command,strlen(command),0,(struct sockaddr *)&addr2,sizeof(addr2));
                    
#ifdef DEBUG
                    LOG::write_log(command);
#endif
                    
                    //Compute embodiment joint mapping for shooting
                    ComputeShoot(xCoord, yCoord, motion_timer);
                    
                    //Generate behavior (speech and gesture)
                    LinuxActionScript::PlayMP3(Behavior::GetInstance()->RetrieveRandomSpeech(Behavior::SHOOT));
                    Action::GetInstance()->Start(82);
                    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
                    
                    cout << "Sent the following: " << command << endl;
                    
                    state = STATE_ROUND_END;
                }
                else if (packet.at(5)=="state_end_round"){
                    
                    state = STATE_ROUND_END;
                }
            }
            
            break;
        }
        case STATE_ROUND_END:{
            
            bSuspendSubThread = true;
            
            if(packet.at(0)=="state"){
                
                bool bRoundTimeout = false;         //Flag for round timeout. Delays time until all game physics are settled.
                
                if(packet.at(5) == "state_end_round"){
                    
                    vector< string > prevPacket = packet;
                    
                    while(!bRoundTimeout){
                        
                        n = recvfrom(ftsockfd,mesg,1000,0,(struct sockaddr *)&ftaddr,&len);
                        mesg[n] = 0;
                        
                        packet = SplitPacket(mesg);
                        
                        cout << "STATE_ROUND_END waiting for timeout. Received the following: " << mesg << endl;
                        
                        if(packet.at(0)=="state"){
                            if (packet.at(5) == "state_end_round") {
                                if(prevPacket.at(4) == packet.at(4))
                                    bRoundTimeout = true;
                                prevPacket = packet;
                            }
                            else //other state
                                bRoundTimeout = true;
                        }
                        else { //usertouch
                            bRoundTimeout = true;
                            packet = prevPacket;
                        }
                    }
                    
                }
                
                if(bRoundTimeout || (packet.at(5)=="state_end_game")){
                    
                    bRoundTimeout = false;
                    
                    if(curCase){
                        
                        //REVISE: Update score in problem descriptor.
                        curCase->mProblem->score = atoi(packet.at(4).c_str()) - curCase->mProblem->score;  //update score
                        mCBR->Revise(curCase->mProblem, curCase->mSolution);
                        
                        //RETAIN: Check condition for retaining.
                        mCBR->Retain(curCase);
                        
                        cout << "===DEMONSTRATION DATA SAVED==== Score is " << curCase->mProblem->score << "====================================================\n\n" << endl;
#ifdef DEBUG
                        stringstream sstm;
                        
                        sstm << "ID: " << curCase->ID << endl;
                        sstm << "Problem: level(" << curCase->mProblem->level << ") round(" << curCase->mProblem->round << ") enemy(" << curCase->mProblem->enemy << ") enemy locations (";
                        
                        for (int i=0; i< curCase->mProblem->enemy; i++){
                            
                            sstm <<"(" << curCase->mProblem->enemyLocation[2*i] << "," << curCase->mProblem->enemyLocation[2*i+1] << ") ";
                        }
                        
                        sstm << ") score(" << curCase->mProblem->score << ")" << endl;
                        
                        sstm << "  Solution: x(" << curCase->mSolution->xTouch << ") y(" <<curCase->mSolution->yTouch << ")" << endl;
                        
                        LOG::write_log(sstm.str());
#endif
                    }
                    
                    cm730->WriteWord(CM730::P_LED_EYE_L, CM730::MakeColor(0,0,255), 0);
                    cm730->WriteWord(CM730::P_LED_HEAD_L, CM730::MakeColor(0,255,0), 0);
                    
                    state = RoundEndHandler(state, packet);     //Handle end of round condition
                    
                    const char *command = "fulltouch 400 100\n";    //send command to proceed to new round
                    sendto(ttsockfd,command,strlen(command),0,(struct sockaddr *)&addr2,sizeof(addr2));
                    
                    cout << "Sent the following: " << command << endl;
                }
                
            }
            break;
        }
            
        case STATE_GAME_END:{
            
            if(packet.at(0)=="state"){
                if((packet.at(5)=="trans_new_round") || (packet.at(5)=="state_new_round")){
                    bSuspendSubThread = true;
                    state = STATE_ROUND_READY;
                }
                else if(packet.at(5)=="state_end_game")
                    bSuspendSubThread = false;
            }
            
            break;
        }
            
        case STATE_IDLE:{
            
            //Start random idle behavior
            bSuspendSubThread = true;
            while(Action::GetInstance()->IsRunning()) usleep(8*1000);
            Action::GetInstance()->Start(Behavior::GetInstance()->RetrieveRandomGesture(Behavior::IDLE));
            LinuxActionScript::PlayMP3(Behavior::GetInstance()->RetrieveRandomSpeech(Behavior::IDLE));
            while(Action::GetInstance()->IsRunning()) usleep(8*1000);
            bSuspendSubThread = false;
            
            state = STATE_GAME_END;
            
            break;
        }
    }
    
}

//Split received task-status packet to string vector.
vector< string > AngryDarwin::SplitPacket (char *msg){
	
	vector< string > SplitVec;
	
	boost::algorithm::split( SplitVec, msg, boost::is_any_of("\t "), boost::token_compress_on );
	
	return SplitVec;
    
}

//Build problem from received task-status packet.
Problem* AngryDarwin::buildProblem (vector< string > packet){
    
    //packet starting with "state"
    Problem *p = new Problem();
    
    p->level = atoi(&(packet.at(1).at(5)));     //extract level number from "level1"
    p->round = 5 - atoi(packet.at(2).c_str());  //extract round number (life = 4)
    p->enemy = atoi(packet.at(3).c_str());      //extract remaining enemy number
    
    for(int i=0; i<p->enemy; i++){              //extract enemy locations
        p->enemyLocation.push_back(atof(packet.at(7+2*i).c_str()));
        p->enemyLocation.push_back(atof(packet.at(8+2*i).c_str()));
    }
    
    p->score = atoi(packet.at(4).c_str());      //extract current score: score is updated after a round.
    
	//    cout << "[Problem] level: " << p->level << ", round: " << p->round <<", remaining enemies: " << p->enemy << ", location: " << p->enemyLocation[0] << ", " << p->enemyLocation[1] << ", score: " << p->score << endl;
    
    return p;
}

//Build solution from received touch-event packet.
Solution* AngryDarwin::buildSolution(vector< string > packet){
    //packet starting with "usertouch"
    Solution *s = new Solution();
    
    s->xTouch = atoi(packet.at(1).c_str());     //extract x-coordinate of touch event
    s->yTouch = atoi(packet.at(2).c_str());     //extract y-coordinate of touch event
    
    //    cout << "[Solution] xTouch: " << s->xTouch << " yTouch: " << s->yTouch << endl;
    
    return s;
}

//Build case from task-status and touch-event packets.
Case* AngryDarwin::buildCase (vector< string > statePacket, vector< string > touchPacket){
    
    Case *newCase = new Case();
    newCase->mProblem = buildProblem(statePacket);
    newCase->mSolution = buildSolution(touchPacket);
    
    newCase->ID = CBRLfD::nIDGenerator;
    
    return newCase;
    
}

//Build case from current problem and solution
Case* AngryDarwin::buildCase (Problem *p, Solution *s){
    
    Case *newCase = new Case(p, s, CBRLfD::nIDGenerator);
    
    return newCase;
}

//Joint mapping of 2-dimensional tablet surface to robot upper-body joints.
//Each motion is defined on a "page" where joint position and timing values are stored.
int AngryDarwin::SetJointValue(LinuxMotionTimer *timer, int pageindex, int stepindex, int posindex, int value){
        
	timer->Stop();      //stop motion timer
	
	Action::PAGE Page;
	
	if(pageindex > 0 && pageindex < Action::MAXNUM_PAGE)
	{
		Action::GetInstance()->LoadPage(pageindex, &Page);      //Retrieve the motion page
	}
	else {
		return -1;
	}
    
	if(stepindex >= 0 && stepindex < Action::MAXNUM_STEP)       //Check step range
	{
        if(posindex == -1){
            Page.step[stepindex].time = value;
        }
        else{
            if(value >= 0 && value <= MX28::MAX_VALUE)          //Check joint range
            {
                if(!(Page.step[stepindex].position[posindex] & Action::INVALID_BIT_MASK))
                {
                    Page.step[stepindex].position[posindex] = value;        //modify joint position
                }
            }
            else{
                cout << "Invalid value range" << endl;          
                return -1;
            }
        }
		
	}
	else{
		cout << "Invalid step index" << endl;
		return -1;
	}
	
	if(Action::GetInstance()->SavePage(pageindex, &Page) == true){
		timer->Start();     //restart motion timer
		return 1;
	}
	
	return 0;
	
}

//Compute pitch and roll for aiming angle 
vector< int > AngryDarwin::ComputeAim(int x, int y, LinuxMotionTimer *timer){
    
    vector< int > pr;
    
    pr.push_back(0.001*y*y - 1.5699*y + 2854);  //pitch
    pr.push_back(1.72*x + 1422);                //roll
    
    //Update joint mapping
    if(timer){
        SetJointValue(timer, 93, 0, JointData::ID_R_SHOULDER_PITCH, pr[0]);
        SetJointValue(timer, 93, 0, JointData::ID_R_SHOULDER_ROLL, pr[1]);
    }
    
    return pr;
}

//Compute pitch, roll, elbow, and speed for shooting angle and power 
vector< int > AngryDarwin::ComputeShoot(int x, int y, LinuxMotionTimer *timer){
    
    vector< int > pres = ComputeAim((-2.1899*x+478.4884), (-2.1899*y+622.0349), NULL); //pitch-roll
    float distance = sqrt((x-150)*(x-150)+(y-195)*(y-195));
        
    pres.push_back(-1.8556*distance+1528);          //elbow
    pres.push_back(0.3333*distance+75);             //speed
    
    //Update joint mapping
    SetJointValue(timer, 82, 0, JointData::ID_R_SHOULDER_PITCH, pres[0]);
    SetJointValue(timer, 82, 0, JointData::ID_R_SHOULDER_ROLL, pres[1]);
    SetJointValue(timer, 82, 0, JointData::ID_R_ELBOW, pres[2]);
    SetJointValue(timer, 82, 0, JointData::ID_HEAD_PAN, max((int)(-2.1268*pres[1]+5344), 1652));
    SetJointValue(timer, 82, 0, JointData::ID_HEAD_TILT, 0.4808*pres[0]+768);
    SetJointValue(timer, 82, 0, -1, pres[3]);
    
    return pres;
}

//At the end of the round, generate robot behavior (speech and gesture)
int AngryDarwin::RoundEndHandler(int curState, vector< string > packet){
    
    int state = curState;
    
    bSuspendSubThread = true;
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
    
    if(packet.at(3) == "0"){ //victory
        Action::GetInstance()->Start(Behavior::GetInstance()->RetrieveRandomGesture(Behavior::VICTORY));
        LinuxActionScript::PlayMP3(Behavior::GetInstance()->RetrieveRandomSpeech(Behavior::VICTORY));
        while(Action::GetInstance()->IsRunning()) usleep(8*1000);
        state = STATE_GAME_END;
    } else if(packet.at(2) == "0"){ //no ghost left
        Action::GetInstance()->Start(Behavior::GetInstance()->RetrieveRandomGesture(Behavior::LOST));
        LinuxActionScript::PlayMP3(Behavior::GetInstance()->RetrieveRandomSpeech(Behavior::LOST));
        while(Action::GetInstance()->IsRunning()) usleep(8*1000);
        state = STATE_GAME_END;
    } else { //game continues
        Action::GetInstance()->Start(85);
        while(Action::GetInstance()->IsRunning()) usleep(8*1000);
        state = STATE_ROUND_READY;
    }
        
    return state;
}

//Idle thread: generate random idle behavior
void* AngryDarwin::Idle_thread(void* ptr)
{
    AngryDarwin *pMain = (AngryDarwin*) ptr;
    
    int r,g,b;
    
    while(1) {
        
        if(!pMain->bSuspendSubThread){
            
            r = rand()%255;
            g = rand()%255;
            b = rand()%255;
            
            pMain->GetCM730()->WriteWord(CM730::P_LED_HEAD_L, CM730::MakeColor(r,g,b), 0);
            
            r = rand()%255;
            g = rand()%255;
            b = rand()%255;
            
            pMain->GetCM730()->WriteWord(CM730::P_LED_EYE_L, CM730::MakeColor(r,g,b), 0);
            
            while(Action::GetInstance()->IsRunning()) usleep(8*1000);
            if(!Action::GetInstance()->IsRunning()){
                
                Action::GetInstance()->Start(Behavior::GetInstance()->RetrieveRandomGesture(Behavior::NEUTRAL));
                LinuxActionScript::PlayMP3(Behavior::GetInstance()->RetrieveRandomSpeech(Behavior::NEUTRAL));
                while(Action::GetInstance()->IsRunning()) usleep(8*1000);
            }
            
            usleep(1000);
        }
        
    }
    return NULL;
}

void AngryDarwin::change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}


int _getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

int ConvertStand2Sit(LinuxMotionTimer *timer, int pageindex){
	
	timer->Stop();
	
	Action::PAGE Page;
    Action::PAGE Page_Sit;
	
	if(pageindex > 0 && pageindex < Action::MAXNUM_PAGE)
	{
		Action::GetInstance()->LoadPage(pageindex, &Page);
        Action::GetInstance()->LoadPage(85, &Page_Sit);
        
	}
	else {
		return -1;
	}
    
    for(int i=0; i < Page.header.stepnum; i++){
        for(int j= JointData::ID_R_HIP_YAW; j <= JointData::ID_L_ANKLE_ROLL; j++){
            Page.step[i].position[j] = Page_Sit.step[0].position[j];
        }
    }
	
	if(Action::GetInstance()->SavePage(pageindex, &Page) == true){
		timer->Start();
		return 1;
	}
	
	return 0;
	
}

int AddHeadPan(LinuxMotionTimer *timer, int pageindex, int value){
	
	timer->Stop();
	
	Action::PAGE Page;
	
	if(pageindex > 0 && pageindex < Action::MAXNUM_PAGE)
	{
		Action::GetInstance()->LoadPage(pageindex, &Page);
	}
	else {
		return -1;
	}
    
    for(int i=0; i < Page.header.stepnum; i++){
        
        Page.step[i].position[JointData::ID_HEAD_PAN] += value;
        
    }
	
	if(Action::GetInstance()->SavePage(pageindex, &Page) == true){
		timer->Start();
		return 1;
	}
	
	return 0;
	
}


int main(void)
{
    
    AngryDarwin *angrydarwin = new AngryDarwin();
    
    printf( "\n===== Angry DARwIn =====\n\n");
#ifdef DEBUG
    time_t ltime = time(NULL);
    LOG::write_log(asctime(localtime(&ltime)));
    LOG::write_log("\n===== Angry DARwIn =====\n\n");
#endif
    
    while(1)
    {
		angrydarwin->RunStateMachine();
        
    }
    
    delete angrydarwin;
    
    return 0;
}


