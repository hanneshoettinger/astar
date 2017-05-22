////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "astar.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

#include <SFML/Graphics.hpp>

using namespace std;

// Global data

// The world map

const int MAP_WIDTH = 20;
const int MAP_HEIGHT = 20;

unsigned int SearchCount = 0;
const unsigned int NumSearches = 3;

// opponent position
float oppoxpos = 2, oppoypos = 2;

int world_map[ MAP_WIDTH * MAP_HEIGHT ] = 
{
	1,1,1,1,1,1,1,9,9,9,1,1,1,1,1,9,9,3,3,3,   // 00
	1,1,1,1,1,1,1,9,9,9,1,1,1,1,1,9,9,3,3,3,   // 01
	1,1,1,1,1,3,1,9,9,9,1,1,1,1,1,9,9,3,3,3,   // 02
	1,1,1,1,1,1,1,9,9,9,1,1,1,1,1,1,9,3,3,3,   // 03
	1,1,4,1,1,1,1,9,9,9,1,1,1,1,1,1,1,1,9,9,   // 04
	1,1,1,1,1,1,1,9,9,1,1,1,1,1,1,1,1,1,1,9,   // 05
	1,1,1,1,1,1,1,9,9,1,4,1,1,4,1,1,1,1,1,1,   // 06
	1,1,1,3,1,1,1,9,9,1,1,1,1,1,1,1,1,1,1,1,   // 07
	1,1,1,1,1,1,1,9,1,1,1,1,1,3,1,1,1,4,1,1,   // 08
	1,1,2,2,2,2,2,2,2,1,4,1,1,1,1,1,1,1,1,1,   // 01
	1,1,2,1,1,1,2,9,2,1,1,1,1,1,1,1,1,1,1,1,   // 10
	1,1,2,1,1,1,1,9,1,1,3,1,1,1,1,1,1,1,1,1,   // 11
	1,1,2,1,1,1,9,9,1,1,1,1,1,4,1,1,1,1,1,1,   // 12
	1,1,2,1,1,9,9,9,9,1,1,1,1,1,1,1,1,1,1,1,   // 13
	1,1,2,1,1,9,9,9,9,9,1,1,1,1,1,1,1,1,1,1,   // 14
	1,1,2,1,1,9,9,1,9,9,9,1,1,1,3,1,1,3,1,1,   // 15
	1,1,2,1,1,9,9,1,1,9,9,9,1,1,1,1,1,1,1,1,   // 16
	2,2,2,1,1,9,9,1,1,9,9,9,1,1,1,1,1,1,1,1,   // 17
	2,2,2,1,1,9,9,1,1,1,9,9,1,1,1,1,1,4,1,1,   // 18
	2,2,2,1,1,9,9,1,1,1,9,9,1,1,1,1,1,1,1,1,   // 19
};


// map helper functions

int GetMap( int x, int y )
{
	if( x < 0 ||
	    x >= MAP_WIDTH ||
		 y < 0 ||
		 y >= MAP_HEIGHT
	  )
	{
		return 9;	 
	}

	// if on opponent -> check opponent xpos & ypos
	float xd = fabsf(x - oppoxpos);
	float yd = fabsf(y - oppoypos);
	//float dist = (sqrt(xd*xd + yd*yd));
	float dist = (xd)+(yd);

	if (dist < 1)
	{
		return (world_map[(y*MAP_WIDTH) + x] + 4);
	}
	else if (dist < 2)
	{
		return (world_map[(y*MAP_WIDTH) + x] + 2);
	}
	else if (dist < 3)
	{
		return (world_map[(y*MAP_WIDTH) + x] + 1);
	}

	return world_map[(y*MAP_WIDTH)+x];
}

// Definitions

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;	

	float diag = 1.f;
	
	MapSearchNode() { x = y = 0; }
	MapSearchNode( int px, int py ) { x=px; y=py; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool AStar(AStarSearch<MapSearchNode>& astarsearch, MapSearchNode& nodeStart, MapSearchNode& nodeEnd);
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
	if( (x == rhs.x) &&
		(y == rhs.y) )
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
	std::cout << "Node position : " << x << " " << y << std::endl;
}

// heuristic function to estimate the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);	
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (x == nodeGoal.x) &&
		(y == nodeGoal.y) )
	{
		return true;
	}

	return false;
}

// successors to the AStar class
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{
	int parent_x = -1; 
	int parent_y = -1; 

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}

	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap( x-1, y ) < 9) 
		&& !((parent_x == x-1) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x, y-1 ) < 9) 
		&& !((parent_x == x) && (parent_y == y-1))
	  ) 
	{
		NewNode = MapSearchNode( x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x+1, y ) < 9)
		&& !((parent_x == x+1) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

		
	if( (GetMap( x, y+1 ) < 9) 
		&& !((parent_x == x) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	// diagonal

	if ((GetMap(x-1, y+1) < 9)
		&& !((parent_x == x-1) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode(x-1, y+1);
		NewNode.diag = 1.414f;
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x+1, y+1) < 9)
		&& !((parent_x == x+1) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode(x+1, y+1);
		NewNode.diag = 1.414f;
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x+1, y-1) < 9)
		&& !((parent_x == x+1) && (parent_y == y-1))
		)
	{
		NewNode = MapSearchNode(x+1, y-1);
		NewNode.diag = 1.414f;
		astarsearch->AddSuccessor(NewNode);
	}

	if ((GetMap(x-1, y-1) < 9)
		&& !((parent_x == x-1) && (parent_y == y-1))
		)
	{
		NewNode = MapSearchNode(x-1, y-1);
		NewNode.diag = 1.414f;
		astarsearch->AddSuccessor(NewNode);
	}

	return true;
}

// Get Map Cost
float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return ((float)GetMap(x, y) * diag);
}

// Main

int main( int argc, char *argv[] )
{

	cout << "A* Hannes Hoettinger\n";

	sf::Time time;
	sf::Clock clock;

	const int waySize = 20;
	const int playerSize = 8;

	sf::RenderWindow window(sf::VideoMode(400, 400), "SFML A-star!");

	sf::RectangleShape way1(sf::Vector2f(waySize, waySize));
	way1.setFillColor(sf::Color::Color(102, 255, 102, 255));
	way1.setOutlineThickness(2);
	way1.setOutlineColor(sf::Color::White);
	sf::RectangleShape way2(sf::Vector2f(waySize, waySize));
	way2.setFillColor(sf::Color::Color(255, 178, 102, 255));
	way2.setOutlineThickness(2);
	way2.setOutlineColor(sf::Color::White);
	sf::RectangleShape way3(sf::Vector2f(waySize, waySize));
	way3.setFillColor(sf::Color::Color(0, 102, 0, 255));
	way3.setOutlineThickness(2);
	way3.setOutlineColor(sf::Color::White);
	sf::RectangleShape way4(sf::Vector2f(waySize, waySize));
	way4.setFillColor(sf::Color::Color(0, 80, 20, 255));
	way4.setOutlineThickness(2);
	way4.setOutlineColor(sf::Color::White);
	sf::RectangleShape way9(sf::Vector2f(waySize, waySize));
	way9.setFillColor(sf::Color::Color(51, 153, 255, 255));
	way9.setOutlineThickness(2);
	way9.setOutlineColor(sf::Color::White);

	sf::RectangleShape thread1(sf::Vector2f(waySize, waySize));
	thread1.setFillColor(sf::Color::Color(250, 15, 25, 120));
	thread1.setOutlineThickness(2);
	thread1.setOutlineColor(sf::Color::White);

	sf::RectangleShape way(sf::Vector2f(playerSize, playerSize));
	way.setFillColor(sf::Color::Red);
	way.setOrigin(-playerSize/2, -playerSize/2);

	sf::RectangleShape player(sf::Vector2f(playerSize, playerSize));
	player.setFillColor(sf::Color::Blue);
	player.setOrigin(-playerSize / 2, -playerSize / 2);

	sf::RectangleShape opponent(sf::Vector2f(playerSize, playerSize));
	opponent.setFillColor(sf::Color::Black);
	opponent.setOrigin(-playerSize / 2, -playerSize / 2);
	opponent.setPosition(waySize * 2, waySize * 2);

	bool forward = true;

	std::vector<sf::RectangleShape> wayPoints;

	sf::CircleShape target(playerSize);
	target.setFillColor(sf::Color::Magenta);
	target.setOrigin(waySize, waySize);

	// A-Star first calculation; set parameters
	// Create a start state
	MapSearchNode nodeStart;
	// Define the goal state
	MapSearchNode nodeEnd;

	AStarSearch<MapSearchNode> astarsearch;

	MapSearchNode *node = astarsearch.GetSolutionStart();
	nodeStart.x = 10;
	nodeStart.y = 10;
	
	nodeEnd.x = 5;
	nodeEnd.y = 0;

	bool result = node->AStar(astarsearch, nodeStart, nodeEnd);
	node = astarsearch.GetSolutionStart();

	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed) 
			{
				window.close();
			}
			if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
			{
				// get global mouse position
				sf::Vector2f position = sf::Vector2f(event.mouseButton.x, event.mouseButton.y);
				nodeEnd.x = ceilf(position.x / waySize) - 1;
				nodeEnd.y = ceilf(position.y / waySize) - 1;
			}
			if (event.type == sf::Event::KeyPressed)
			{
				if (event.key.code == sf::Keyboard::Up)
				{
					nodeEnd.y -= 1;
					if (nodeEnd.y < 0)
					{
						nodeEnd.y = MAP_HEIGHT - 1;
					}
				}
				if (event.key.code == sf::Keyboard::Right)
				{
					nodeEnd.x += 1;
					nodeEnd.x = fmodf(nodeEnd.x, MAP_WIDTH);
				}
				if (event.key.code == sf::Keyboard::Down)
				{
					nodeEnd.y += 1;
					nodeEnd.y = fmodf(nodeEnd.y, MAP_HEIGHT);
				}
				if (event.key.code == sf::Keyboard::Left)
				{
					nodeEnd.x -= 1;
					if (nodeEnd.x < 0)
					{
						nodeEnd.x = MAP_WIDTH - 1;
					}
				}
			}
		}

		window.clear();

		time = clock.getElapsedTime();

		// move threadmap
		if (time.asSeconds() > 1.5)
		{
			if (forward == true)
			{
				oppoxpos += 1;
				oppoypos += 1;
				
				if (oppoxpos > 7)
				{
					forward = false;
				}
			}
			else
			{
				oppoxpos -= 1;
				oppoypos -= 1;
				if (oppoxpos == 1)
				{
					forward = true;
				}
			}

			opponent.setPosition(oppoxpos * waySize, oppoypos * waySize);

			// new calculation of a star pathfinding
			SearchCount = 0;
			if (result == true) astarsearch.FreeSolutionNodes();
			node = astarsearch.GetSolutionStart();
			if (nodeEnd.x >= 0 & nodeEnd.x < 40)
				result = node->AStar(astarsearch, nodeStart, nodeEnd);
			else
			{
				nodeEnd.x = 3;
				nodeEnd.y = 10;
				result = node->AStar(astarsearch, nodeStart, nodeEnd);
			}
			node = astarsearch.GetSolutionStart();
			clock.restart();
			
		}

		for (int x = 0; x < MAP_WIDTH; x++)
		{
			for (int y = 0; y < MAP_HEIGHT; y++)
			{
				switch (world_map[(y*MAP_WIDTH) + x])
				{
				case 0:
					break;
				case 1:
					way1.setPosition(waySize * x, waySize *y);
					window.draw(way1);
					break;
				case 2:
					way2.setPosition(waySize * x, waySize *y);
					window.draw(way2);
					break;
				case 3:
					way3.setPosition(waySize * x, waySize *y);
					window.draw(way3);
					break;
				case 4:
					way4.setPosition(waySize * x, waySize *y);
					window.draw(way4);
					break;
				case 9:
					way9.setPosition(waySize * x, waySize *y);
					window.draw(way9);
					break;
				}
			}
		}

		time = clock.getElapsedTime();

		//set waypoints
		while (node && result == true)
		{
			node = astarsearch.GetSolutionNext();
			
			if (!node)
			{
				break;
			}
			way.setPosition(waySize * node->x, waySize * node->y);
			wayPoints.push_back(way);
		}
		node = astarsearch.GetSolutionStart();

		player.setPosition(waySize * nodeStart.x, waySize * nodeStart.y);
		player.setFillColor(sf::Color::Blue);
		window.draw(player);

		for each(sf::RectangleShape rect in wayPoints)
		{
			window.draw(rect);
		}
		wayPoints.clear();

		target.setPosition(waySize * (nodeEnd.x+1), waySize * (nodeEnd.y+1));
		window.draw(target);

		// draw thread map
		for (int i = oppoxpos - 2;i < oppoxpos + 3; i++)
		{
			thread1.setPosition(waySize * i, waySize * oppoypos);
			window.draw(thread1);
		}
		for (int i = oppoypos - 2;i < oppoypos + 3; i++)
		{
			thread1.setPosition(waySize * oppoxpos, waySize * i);
			window.draw(thread1);
		}
		thread1.setPosition(waySize * (oppoxpos - 1), waySize * (oppoypos + 1));
		window.draw(thread1);
		thread1.setPosition(waySize * (oppoxpos + 1), waySize * (oppoypos + 1));
		window.draw(thread1);
		thread1.setPosition(waySize * (oppoxpos + 1), waySize * (oppoypos - 1));
		window.draw(thread1);
		thread1.setPosition(waySize * (oppoxpos - 1), waySize * (oppoypos - 1));
		window.draw(thread1);


		window.draw(opponent);

		window.display();
	}

	return 0;
}

bool MapSearchNode::AStar(AStarSearch<MapSearchNode>& astarsearch, MapSearchNode& nodeStart, MapSearchNode& nodeEnd)
{
	bool result;

	while (SearchCount < NumSearches)
	{
		// Set Start and goal states

		astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();

			SearchSteps++;
		} while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

		if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED)
		{
			cout << "Search found goal state\n";
			result = true;

		}
		else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED)
		{
			cout << "Search terminated. Did not find goal state\n";
			result = false;
			return result;
		}

		cout << "SearchSteps : " << SearchSteps << "\n";

		SearchCount++;
	}

	return result;
}