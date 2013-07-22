Jul 22, 2013

Abstract:
----------
Angry Darwin, presents a case-based reasoning approach to robot learning 
from human demonstration (CBR-LfD). The application domain of the proposed algorithm is a turn-based
interaction between human and robot on a shared tablet workspace. We find that a CBR
approach to this problem, retrieving what has been observed in the past and reusing existing
behaviors to respond to the current problem, generates reliable interaction behaviors. The
process is composed of task demonstration to task-case conversion, task-policy derivation, and 
embodiment mapping. We intend to demonstrate the proposed algorithm through learning to play a 
popular tablet game: Angry Birds.


Folders and Files:
------------------

darwin/: robot library 		http://sourceforge.net/projects/darwinop/
tinyxml/: TinyXML-2 XML parser	http://www.grinninglizard.com/tinyxml2/
sound/: robot speech source files

CBRLfD_Simple.h: Declarations of simplified CBRLfD routines.
CBRLfD_Simple.cpp: Implementations of simplified CBRLfD routines.
CBRLfD_Simple.xml: Case-feature structure configuration.

Behavior.h: Declarations of robot gesture-speech behavior generation.
Behavior.cpp: Implementations of robot gesture-speech behavior generation.
Behavior.asc: List of gesture and speech primitive definition and source.

Log.h: Logging header and inline function.

main.h: Declarations of Angry Darwin application using CBR-LfD.
main.cpp: Implementations of Angry Darwin application using CBR-LfD.





