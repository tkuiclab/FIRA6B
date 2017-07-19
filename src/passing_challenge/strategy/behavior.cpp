#include "behavior.h"

Behavior::Behavior(int argc, char** argv)
{
	//nn = new Strategy_nodeHandle();
	init(argc, argv);
}

Behavior::~Behavior()
{

}

void Behavior::chooseLevel()
{
	
}

void Behavior::setStatus()
{
	if(status == 0){
		gameStatus = Halt;
	}else if(status == 1){
		gameStatus = Start;
	}else{
		gameStatus = Error;
	}
}

void Behavior::start()
{

}

void Behavior::halt()
{

}

void Behavior::chase()
{

}

void Behavior::aim()
{

}
