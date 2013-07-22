/*
 * main.h
 *
 * Created on: 2013. 7. 12.
 * Author: Hae Won Park
 * Description: Declarations of Angry Darwin application using CBR-LfD.
 * Last modified: 2013. 7. 15.
 */

/*
 * Angry Darwin is an application in which we applied CBR-LfD to teach the robot (Darwin-OP) a task on a tablet.
 *
 *  Abstract:
 *  ---------
 *  Angry Darwin, presents a case-based reasoning (CBR) approach to robot learning
 *  from human demonstration. The application domain of the proposed algorithm is a turn-based
 *  interaction between human and robot on a shared tablet workspace. We find that a CBR
 *  approach to this problem, retrieving what has been observed in the past and reusing existing
 *  behaviors to respond to the current problem, generates reliable interaction behaviors. The
 *  process is composed of task demonstration to task-case conversion, task-policy derivation, and 
 *  embodiment mapping. We intend to demonstrate the proposed algorithm through learning to play a 
 *  popular tablet game: Angry Birds.
 *
 */
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <sstream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <cstdlib>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <cmath>

//Darwin-OP robot include files
#include "Action.h"
#include "Head.h"
#include "Walking.h"
#include "MX28.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "LinuxCM730.h"
#include "LinuxActionScript.h"
#include "LinuxDARwIn.h"
//#include "Camera.h"
//#include "Point.h"
#include "mjpg_streamer.h"
//#include "minIni.h"
//#include "LinuxCamera.h"
//#include "ColorFinder.h"

#include "CBRLfD_Simple.h"      //CBR-LfD (simplified) class header
#include "Behavior.h"           //Robot gesture+speech behavior class header
#include "Log.h"   

//-------------------------------------------------------------
// Robot is establshing a socket communication with the tablet.
// Robot sends synthesized touch events and receives current-task
// information from the tablet.
//-------------------------------------------------------------
#define TABLET_IP			"192.168.1.103"
#define FROM_TABLET_PORT	12345
#define TO_TABLET_PORT		8888

//-------------------------------------------------------------
// Robot is communicating with the tablet and generates
// synthesized touch events. Inverse kinematics for the robot's
// 6-DOF upper body and 2-DOF head is computed for the tablet region.
// The following limits are defining the tablet screen region where the
// robot is able to interact.
//-------------------------------------------------------------
#define xLowLimit   138
#define xHighLimit  180
#define yLowLimit   178
#define yHighLimit  212

//-------------------------------------------------------------
// Robot puts itself to an idle mode after certain period of no interaction.
//-------------------------------------------------------------
#define IDLE_TIMEOUT        15
#define IDLE_TIMEOUT_OFFSET 7

//-------------------------------------------------------------
// If this parameter is set, robot is able to store its own trial cases.
// If not, robot only stores demonstrated cases.
//-------------------------------------------------------------
#define SELF_TRAIN	true


//Angry Darwin state machine states enum.
enum{
    STATE_ROUND_READY,
    STATE_AIM,
    STATE_SHOOT,
    STATE_ROUND_END,
    STATE_GAME_END,
    STATE_DEMONSTRATION,
    STATE_SELF_TRAIN,
    STATE_IDLE
};

using std::string;
using std::vector;
using namespace Robot;

//Motion-definition file path. Created using Darwin-OP's action script.
#ifdef MX28_1024
#define MOTION_FILE_PATH    "./darwin/Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "./darwin/Data/motion_4096.bin"
#endif

//-------------------------------------------------------------
//  AngryDarwin
//      AngryDarwin class implementes "teacher-demonstration recording" and
//      "embodiment mapping" of LfD steps. It's responsible for:
//
//      - Establishing socket communication with a tablet where the task is taken place.
//      - Receiving and extracting task status and demonstration data from tablet.
//      - Computing embodiment solution (joint data and behavior) from the solution developed by the CBRLfD class.
//      - Sending sythesized touch event to the tablet.
//      - Managing these steps with a finite-state machine.
//-------------------------------------------------------------
class AngryDarwin{
    
public:
    AngryDarwin();
    ~AngryDarwin();
    
    //Angry Darwin finite-state machine
    void RunStateMachine();
    
    //Idle thread activated when human interaction is absent for certain time.
    static void *Idle_thread(void* ptr);
    
    
private:
    
    CBRLfD *mCBR;

    //Socket variables
    int ftsockfd, ttsockfd, n;
    struct sockaddr_in addr1, addr2, ftaddr;
    socklen_t len;
    char mesg[1000];

    //Robot framework variables
    LinuxCM730 *linux_cm730;
    CM730 *cm730;
    LinuxMotionTimer *motion_timer;
	
    pthread_t thread_t;    
    
	int state;              //current state
    int prevstate;          //previous state
	int xCoord, yCoord;     //tablet (x,y) coordinates

    vector< string > packet, prevStatePacket;   //received packet is splitted and stored in vector
	Case *curCase;
    
    //Idle timeout variables
    bool bIdle;
    int idlecount;
    int timeout;
    bool bSuspendSubThread;
    
    //Initialize socket and robot framework
    void change_current_dir();
    void InitSocket();
    
    //Parse received task-status packet from tablet.
    vector< string > SplitPacket (char *msg);
    
    //Convert received packet data into problem and solution.
    Problem* buildProblem (vector< string > packet);
    Solution* buildSolution(vector< string > packet);
    Case* buildCase (vector< string > statePacket, vector< string > touchPacket);
    Case* buildCase (Problem *p, Solution *s);
    
    //Edit motion file to reflect new joint data.
    int SetJointValue(LinuxMotionTimer *timer, int pageindex, int stepindex, int posindex, int value);
    
    //Compute IK of the upper 6-dof arm pitch-roll-yaw and 2-dof head pan and tilt.
    vector< int > ComputeAim(int x, int y, LinuxMotionTimer *timer);
    vector< int > ComputeShoot(int x, int y, LinuxMotionTimer *timer);
    
    
    int RoundEndHandler(int curState, vector< string > packet);
    
    LinuxMotionTimer* GetMotionTimer(){ return motion_timer;};
    CM730* GetCM730(){ return cm730;};
    
    
};
