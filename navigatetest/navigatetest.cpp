#include <iostream>
#include "Aria.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include<fstream>
#include<typeinfo>
#include<windows.h>
#include <cstring>
#include <math.h>
#include <string>
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>  
#include <vector>  
#include <sstream>  

#include "CImg.h"
#include "low.h"
#include "stlastar.h"

#define DEBUG_LISTS 1
#define DEBUG_LIST_LENGTHS_ONLY 1

#define Threshold 230

using namespace cimg_library;
using namespace std;

int allmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
int nowmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
int findpathmap[MAP_WIDTH][MAP_HEIGHT];
int world_map[MAP_WIDTH][MAP_HEIGHT];
CImg<unsigned char> Allmapimgshow;
CImg<unsigned char> Findmapshow;

void Conv2(int filterW, int filterH, int arrW, int arrH)
{
	float temp;
	//float filter[5][5] = { {1,4,7,4,1},{4,16,26,16,4},{7,26,41,26,7},{4,16,26,16,4} ,{1,4,7,4,1} };
	float filter[5][5] = { {1,1,1,1,1},{1,1,1,1,1},{1,1,1,1,1},{1,1,1,1,1} ,{1,1,1,1,1} };
	for (int i = 0; i < arrH - filterH; i = i + filterH) {
		for (int j = 0; j < arrW - filterW; j = j + filterW) {
			temp = 0;
			for (int m = 0; m < filterH; m++) {
				for (int n = 0; n < filterW; n++) {
					temp += filter[m][n] * allmap[i][j];
				}
			}
			if (temp / 25.0 > Threshold) {
				findpathmap[(i / filterH)][(j / filterW)] = 255;
				world_map[(i / filterH)][(j / filterW)] = 1;
			}
			else {
				findpathmap[(i / filterH)][(j / filterW)] = 0;
				world_map[(i / filterH)][(j / filterW)] = 9;
			}
		}
	}

}

int GetMap(int x, int y)
{
	if (x < 0 ||
		x >= MAP_WIDTH ||
		y < 0 ||
		y >= MAP_HEIGHT
		)
	{
		return 9;
	}
	return world_map[x][y];
}


// Definitions

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;

	MapSearchNode() { x = y = 0; }
	MapSearchNode(int px, int py) { x = px; y = py; }

	float GoalDistanceEstimate(MapSearchNode &nodeGoal);
	bool IsGoal(MapSearchNode &nodeGoal);
	bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node);
	float GetCost(MapSearchNode &successor);
	bool IsSameState(MapSearchNode &rhs);

	void PrintNodeInfo();
	void PrintNodeInfofinal();

};

bool MapSearchNode::IsSameState(MapSearchNode &rhs)
{

	// same state in a maze search is simply when (x,y) are the same
	if ((x == rhs.x) &&
		(y == rhs.y))
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf(str, "Node position : (%d,%d)\n", x, y);
	cout << str;
}

void MapSearchNode::PrintNodeInfofinal()
{
	char str[100];
	sprintf(str, "Node position : (%d,%d)\n", x, y);
	allmap[x*5+2][y*5+2] = 200;
	cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal)
{
	return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal)
{

	if ((x == nodeGoal.x) &&
		(y == nodeGoal.y))
	{
		return true;
	}

	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node)
{

	int parent_x = -1;
	int parent_y = -1;

	if (parent_node)
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}


	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if ((GetMap(x - 1, y) < 9)
		&& !((parent_x == x - 1) && (parent_y == y))
		)
	{
		NewNode = MapSearchNode(x - 1, y);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x, y - 1) < 9)
		&& !((parent_x == x) && (parent_y == y - 1))
		)
	{
		NewNode = MapSearchNode(x, y - 1);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x + 1, y) < 9)
		&& !((parent_x == x + 1) && (parent_y == y))
		)
	{
		NewNode = MapSearchNode(x + 1, y);
		astarsearch->AddSuccessor(NewNode);
	}


	if ((GetMap(x, y + 1) < 9)
		&& !((parent_x == x) && (parent_y == y + 1))
		)
	{
		NewNode = MapSearchNode(x, y + 1);
		astarsearch->AddSuccessor(NewNode);
	}

	/*if ((GetMap(x + 1, y + 1) < 9)
		&& !((parent_x == x + 1) && (parent_y == y + 1))
		)
	{
		NewNode = MapSearchNode(x + 1, y + 1);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x - 1, y + 1) < 9)
		&& !((parent_x == x - 1) && (parent_y == y + 1))
		)
	{
		NewNode = MapSearchNode(x - 1, y + 1);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x - 1, y - 1) < 9)
		&& !((parent_x == x - 1) && (parent_y == y - 1))
		)
	{
		NewNode = MapSearchNode(x - 1, y - 1);
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x + 1, y - 1) < 9)
		&& !((parent_x == x + 1) && (parent_y == y - 1))
		)
	{
		NewNode = MapSearchNode(x + 1, y - 1);
		astarsearch->AddSuccessor(NewNode);
	}*/

	return true;
}

// given this node, what does it cost to moveto successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost(MapSearchNode &successor)
{
	return (float)GetMap(x, y);

}

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if (!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "navigatetest: Could not connect to the robot.");
    if (parser.checkHelpAndWarnUnparsed())
    {
      // -help not given
      Aria::logOptions();
      Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1); // exit with error code
  }

  ArLog::log(ArLog::Normal, "navigatetest: Connected to robot.");


  ifstream inFile("map//map.csv", ios::in);
  string lineStr;
  vector<string> lineArray;
  while (getline(inFile, lineStr))
  {
	  // 存成二维表结构  
	  stringstream ss(lineStr);
	  string str;
	  
	  // 按照逗号分隔  
	  while (getline(ss, str, ',')) {
		  lineArray.push_back(str);
	  }
	  for (int i = 0; i < ALLMAP_WIDTH; i++) {
		  for (int j = 0; j < ALLMAP_HEIGHT; j++) {
			  nowmap[i][j] = atoi(lineArray[i * ALLMAP_WIDTH + j].c_str());
			  allmap[i][j] = 255;
		  }
	  }
  }
  int xend = 480,yend = 255;
  float ox, oy, ot, ott;
  int rx, ry, xtest, ytest, X, Y;
  float cosrad, sinrad, cX, cY, fX, fY;
  ox = -atof(lineArray[ALLMAP_WIDTH*ALLMAP_HEIGHT ].c_str());
  oy = atof(lineArray[ALLMAP_WIDTH*ALLMAP_HEIGHT + 1].c_str());
  ott =  -atof(lineArray[ALLMAP_WIDTH*ALLMAP_HEIGHT + 2].c_str());
  ot = -atof(lineArray[ALLMAP_WIDTH*ALLMAP_HEIGHT+2].c_str());
  cout << ox << oy << ot << endl;
  cosrad = (float)std::cos(-ott);
  sinrad = (float)std::sin(-ott);
  for (int x = 0; x < ALLMAP_WIDTH; x++) {
	  for (int y = 0; y < ALLMAP_HEIGHT; y++) {
		  cX = x - ALLMAP_WIDTH / 2;
		  cY = y - ALLMAP_HEIGHT / 2;
		  fX = ALLMAP_WIDTH / 2 + cX * cosrad - cY * sinrad;
		  fY = ALLMAP_HEIGHT / 2 + cX * sinrad + cY * cosrad;
		  X = (int)fmod(fX, ALLMAP_WIDTH);
		  Y = (int)fmod(fY, ALLMAP_HEIGHT);
		  ytest = (int)(ox * (float)std::cos(ot) + oy * (float)std::sin(ot));
		  xtest = (int)(-ox * (float)std::sin(ot) + oy * (float)std::cos(ot));
		  if ((y + ytest >= 0) && (y + ytest < ALLMAP_HEIGHT))
			  ry = y + ytest;
		  else
			  ry = y;
		  if ((x + xtest < ALLMAP_WIDTH) && (x + xtest >= 0))
			  rx = x + xtest;
		  else
			  rx = x;
		  allmap[rx][ry] = nowmap[X][Y];
	  }
  }
  Allmapimgshow.assign(ALLMAP_HEIGHT, ALLMAP_WIDTH, 1, 1);
  CImgDisplay Allmap_disp;
  cimg_forXY(Allmapimgshow, x, y) {
	  Allmapimgshow(x, y, 0) = allmap[x][y];
  }
  Allmapimgshow.display(Allmap_disp, true);
  cout << "数据加载成功" << "\n";

  
  Conv2(5, 5, ALLMAP_HEIGHT, ALLMAP_WIDTH);

  Findmapshow.assign(MAP_HEIGHT, MAP_WIDTH, 1, 1);
  cimg_forXY(Findmapshow, x, y) {
	  Findmapshow(x, y, 0) = findpathmap[x][y];
  }
  Findmapshow.display(Allmap_disp, true);

  AStarSearch<MapSearchNode> astarsearch;

  // Create a start state
  MapSearchNode nodeStart;
  nodeStart.x = MAP_WIDTH / 2;
  nodeStart.y = MAP_HEIGHT / 2;

  // Define the goal state
  MapSearchNode nodeEnd;
  nodeEnd.x = int(xend / 5);
  nodeEnd.y = int(yend / 5);

  // Set Start and goal states
  astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

  unsigned int SearchState;
  unsigned int SearchSteps = 0;
  vector< MapSearchNode * > nownode;
  vector< MapSearchNode * > Closednode;
  int lesslen = 10000;
  do
  {
	  SearchState = astarsearch.SearchStep();

	  SearchSteps++;

#if DEBUG_LISTS

	  cout << "Steps:" << SearchSteps << "\n";

	  int len = 0;

	  cout << "Open:\n";
	  MapSearchNode *p = astarsearch.GetOpenListStart();
	  while (p)
	  {
		  len++;
#if !DEBUG_LIST_LENGTHS_ONLY			
		  ((MapSearchNode *)p)->PrintNodeInfo();''
#endif
		  p = astarsearch.GetOpenListNext();

	  }

	  cout << "Open list has " << len << " nodes\n";

	  len = 0;

	  cout << "Closed:\n";
	  p = astarsearch.GetClosedListStart();
	  while (p)
	  {
		  len++;
#if !DEBUG_LIST_LENGTHS_ONLY			
		  p->PrintNodeInfo();
#endif			
		  p = astarsearch.GetClosedListNext();
	  }

	  cout << "Closed list has " << len << " nodes\n";
#endif

  } while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);


  if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED)
  {
	  cout << "Search found goal state\n";

	  MapSearchNode *node = astarsearch.GetSolutionStart();

#if DISPLAY_SOLUTION
	  cout << "Displaying solution\n";
#endif
	  int steps = 0;

	  node->PrintNodeInfo();
	  for (;; )
	  {
		  node = astarsearch.GetSolutionNext();

		  if (!node)
		  {
			  break;
		  }

		  node->PrintNodeInfofinal();
		  steps++;

	  };

	  cout << "Solution steps " << steps << endl;

	  // Once you're done with the solution you can free the nodes up
	  astarsearch.FreeSolutionNodes();


  }
  else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED)
  {
	  cout << "Search terminated. Did not find goal state\n";

  }

  // Display the number of loops the search went through
  cout << "SearchSteps : " << SearchSteps << "\n";

  astarsearch.EnsureMemoryFreed();



  cimg_forXY(Allmapimgshow, x, y) {
	  Allmapimgshow(x, y, 0) = allmap[x][y];
  }
  Allmapimgshow.display(Allmap_disp, true);

  robot.enableMotors();

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  robot.lock();
  ArLog::log(ArLog::Normal, "navigatetest: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "navigatetest: Sleeping for 3 seconds...");
  ArUtil::sleep(3000);


  ArLog::log(ArLog::Normal, "simpleConnect: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "navigatetest: Exiting.");
  Aria::exit(0);
  return 0;
}
