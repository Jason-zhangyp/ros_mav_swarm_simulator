#include "relativetracker.h"

using namespace std;

relativetracker::relativetracker(const string &tracker, const string &tracked)
{
	/* store the name of the tracker and the tracked */
	ID_tracker = tracker;
	ID_tracked = tracked;

	cout << ID_tracker 
	<< " initiated a listener for " 
	<< ID_tracked 
	<< endl;

	/* Bluetooth sensor simulator object running to collect data */
	btsim = new BTsimulator(ID_tracker,ID_tracked);

}

relativetracker::~relativetracker(){}


/* Update the contents of the simulated bluetooth message 
(Simulate receiving a message) */
void relativetracker::update(){
	btmsg = btsim->getBTmessage();
}

/* Get the latest update of the bluetooth message*/
BTmessage relativetracker::get(){
	return btmsg;
}

/* Read the message (returns the latest messages but content is not updated)*/
BTmessage relativetracker::read(){
	return btsim->getBTmessage();
}

/* Get the ID of the tracked object */
int relativetracker::getID(){

	/* Complinets of SledgeHammer_999 http://ubuntuforums.org/showthread.php?t=1530912 */

	int id;
	string temp;

	for (unsigned int i=0; i < ID_tracked.size(); i++)
	{
        //iterate the string to find the first "number" character
        //if found create another loop to extract it
        //and then break the current one
        //thus extracting the FIRST encountered numeric block
		if (isdigit( ID_tracked[i]))
		{
			temp = ID_tracked[i];
			std::istringstream stream(temp);
			stream >> id;
			return id;
		}
	}

	// return ID_tracked.c_str();
}