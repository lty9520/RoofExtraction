#include "DBSCAN.h"

void point::init()
{
	cluster = 0;
	pointType = pointType_UNDO;//pointType_NOISE pointType_UNDO
	pts = 0;
	visited = 0;
	corePointID = -1;
}

int main(int argc, char** argv) {
	DBSCANCLUSTER dbscan;
	vector<point> dataset = dbscan.openFile("dataPoint2.txt");
	dbscan.DBSCAN(dataset, 0.5, 3);
	return 0;
}
