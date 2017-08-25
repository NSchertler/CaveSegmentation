#include "ICaveData.h"
#include "CaveData.h"

ICaveData* ::CreateCaveData()
{
	return new CaveData();
}


void ::DestroyCaveData(ICaveData* data)
{
	delete data;
}