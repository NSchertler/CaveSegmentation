#include "ICaveData.h"
#include "CaveData.h"
#include "ImageProc.h"

std::shared_ptr<ICaveData> CreateCaveData()
{
	return std::make_shared<CaveData>();
}

void ICaveData::StartCaveSeg()
{
	StartImageProc();
}

void ICaveData::StopCaveSeg()
{
	StopImageProc();
}

int noSegmentColor[3] = { 128, 128, 128 };
int segmentColors[10][3] =
{
	{ 166, 206, 227 },
	{ 31,120,180 },
	{ 251,154,153 },
	{ 227,26,28 },
	{ 253,191,111 },
	{ 255,127,0 },
	{ 202,178,214 },
	{ 106,61,154 },
	{ 255,255,153 },
	{ 177,89,40 }
};

const int* ICaveData::GetSegmentColor(int segmentIndex)
{
	if (segmentIndex < 0)
		return noSegmentColor;
	else
		return segmentColors[segmentIndex % 10];
}