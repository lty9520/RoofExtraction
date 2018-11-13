#include "DBSCAN.h"

DBSCANCLUSTER::DBSCANCLUSTER()
{

}

DBSCANCLUSTER::~DBSCANCLUSTER()
{

}

float DBSCANCLUSTER::stringToFloat(string i){
	stringstream sf;
	float score = 0;
	sf << i;
	sf >> score;
	return score;
}

vector<point> DBSCANCLUSTER::openFile(const char* dataset){
	fstream file;
	file.open(dataset, ios::in);
	if (!file)
	{
		cout << "Open File Failed!" << endl;
		vector<point> a;
		return a;
	}
	vector<point> data;
	int i = 1;
	while (!file.eof()){
		string temp;
		file >> temp;
		int split = temp.find(',', 0);
		point p(stringToFloat(temp.substr(0, split)), stringToFloat(temp.substr(split + 1, temp.length() - 1)), i++);
		data.push_back(p);
	}
	file.close();
	cout << "successful!" << endl;
	return data;
}

float DBSCANCLUSTER::squareDistance(point a, point b){
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

void DBSCANCLUSTER::DBSCAN(vector<point> dataset, float Eps, int MinPts){
	int len = dataset.size();//数据长度
	for (int i = 0; i < len; i++)//参数初始化
	{
		dataset[i].init();
	}

	vector<vector <float>> distP2P(len);

	//calculate pts
	cout << "calculate pts" << endl;
	for (int i = 0; i < len; i++){
		for (int j = 0; j < len; j++){//i+1
			float distance = squareDistance(dataset[i], dataset[j]);
			distP2P[i].push_back(distance);//disp for debug
			if (distance <= Eps){
				dataset[i].pts++;
			}
		}
	}
	//core point 
	cout << "core point " << endl;
	vector<point> corePoint;
	for (int i = 0; i < len; i++){
		int tempPts = dataset[i].pts;
		if (tempPts >= MinPts) {
			dataset[i].pointType = pointType_CORE;
			dataset[i].corePointID = i;
			corePoint.push_back(dataset[i]);
		}
	}
	cout << "joint core point" << endl;

	//joint core point
	int numCorePoint = corePoint.size(); //core point number

	for (int i = 0; i < numCorePoint; i++){
		for (int j = 0; j < numCorePoint; j++){
			float distTemp = distP2P[corePoint[i].corePointID][corePoint[j].corePointID];//display for debug
			if (distTemp <= Eps){//squareDistance(corePoint[i],corePoint[j])
				corePoint[i].corepts.push_back(j);//other point orderID link to core point
			}
		}
	}
	for (int i = 0; i < numCorePoint; i++){
		stack<point*> ps;
		if (corePoint[i].visited == 1) continue;
		clusterID++;
		corePoint[i].cluster = clusterID; //create a new cluster
		ps.push(&corePoint[i]);
		point *v;
		while (!ps.empty()){
			v = ps.top();
			v->visited = 1;
			ps.pop();
			for (int j = 0; j < v->corepts.size(); j++){
				if (corePoint[v->corepts[j]].visited == 1) continue;
				corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
				corePoint[v->corepts[j]].visited = 1;
				ps.push(&corePoint[v->corepts[j]]);
			}
		}

	}

	cout << "border point,joint border point to core point" << endl;
	//border point,joint border point to core point
	for (int i = 0; i < len; i++){
		for (int j = 0; j < numCorePoint; j++){
			float distTemp = distP2P[i][corePoint[j].corePointID];
			if (distTemp <= Eps) {
				dataset[i].pointType = pointType_BORDER;
				dataset[i].cluster = corePoint[j].cluster;
				break;
			}
		}
	}
	cout << "output" << endl;
	//output
	//display  
	for (int i = 0; i < len; i++){
		cout << "第" << i + 1 << "个数据: " << dataset[i].x << "," << dataset[i].y << "," << dataset[i].cluster << "\n";
	}
	//save in .txt format named clustering.txt
	fstream clustering;
	clustering.open("clustering.txt", ios::out);
	for (int i = 0; i < len; i++){
		clustering << dataset[i].x << "," << dataset[i].y << "," << dataset[i].cluster << "\n";
	}
	clustering.close();
}