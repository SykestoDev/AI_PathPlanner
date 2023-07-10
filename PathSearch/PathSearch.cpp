#include "PathSearch.h"

namespace fullsail_ai { namespace algorithms {

	PathSearch::PathSearch()
	{
		
	}

	PathSearch::~PathSearch()
	{
	}

	void PathSearch::initialize(TileMap* _tileMap)
	{
		tileMap = _tileMap;
		
		
		// Iterate through the tileMap //
		for (size_t row = 0; row < tileMap->getRowCount(); row++)
		{
			for (size_t col = 0; col < tileMap->getColumnCount(); col++)
			{
				// take {col, row} and create a new SearchNode   //
				// -- If the tile's weight is 0 don't create a node --  //
				if (tileMap->getTile(row, col)->getWeight() > 0)
				{
					// Create a Node for the curr Tile
					SearchNode* currNode = new SearchNode();
					currNode->tile = tileMap->getTile(row, col);

					// Inserts the new Search node to the node Container //
					nodes.insert(std::make_pair(currNode->tile, currNode));
				}
			}
		}

		SearchNode* CurrNode;
		// Iterate through the tileMap //
		for (size_t row = 0; row < tileMap->getRowCount(); row++)
		{
			for (size_t col = 0; col < tileMap->getColumnCount(); col++)
			{
				if (tileMap->getTile(row, col)->getWeight() != 0)
				{

					CurrNode = nodes[tileMap->getTile(row, col)];

					// Check row to see if it's Even or Odd //
					if (row % 2 == 0)
					{
						GetEvenNeigbors(row, col, CurrNode);
					}
					else
					{
						GetOddNeighbors(row, col, CurrNode);
					}

				}
			}
		}
	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{	
		goalFound = false;
		// Create pointers to the start & goal tiles  //
		SearchNode* startNode  = nodes[tileMap->getTile(startRow, startColumn)];
		goalNode = nodes[tileMap->getTile(goalRow, goalColumn)];

		// Create a new PlannerNode to insert to the visited container  //
		PlannerNode* PlanNode = new PlannerNode();
		PlanNode->searchNode = startNode;
		PlanNode->parent = nullptr;

		// Push new PlanNode Into the Queue //
		openQueue.push(PlanNode);
		// Add it to the Visisted Container //
		visited[startNode] = openQueue.front();

		// Set the planner nodes info
		openQueue.front()->givenCost = 0;
		openQueue.front()->heuristicCost = estimate(startNode, goalNode);
		openQueue.front()->finalCost = openQueue.front()->givenCost + openQueue.front()->heuristicCost * hWeight;
	}

	void PathSearch::update(long timeslice)
	{
		// Loop while Queue isnt empty //
		while (!openQueue.empty())
		{
			// Start at the front of the Queue //
			PlannerNode* currPlanNode = openQueue.front();

			// Check to see if the currNode is the finish Line //
			if (currPlanNode->searchNode == goalNode)
			{
				goalFound = true;
				break;
			}
			else goalFound = false;

			openQueue.remove(currPlanNode);

			SearchNode* successorNode;
			for (size_t i = 0; i < currPlanNode->searchNode->neighbors.size(); i++)
			{
				// make a successor based on the neighbor
				SearchNode* successor = currPlanNode->searchNode->neighbors[i];
				// calculate a given cost from the current node and the successor
				float tempGivenCost = currPlanNode->givenCost + (estimate(currPlanNode->searchNode,currPlanNode->searchNode->neighbors[i]) * successor->tile->getWeight());
				successor->tile->setFill(0xff0000ff);
				
				// if node hasnt been marked visited
				if (visited[successor] != NULL)
				{
					PlannerNode* planner = visited[successor];

					if (tempGivenCost < planner->givenCost)
					{
						openQueue.remove(planner);
						planner->givenCost = tempGivenCost;
						planner->finalCost = planner->givenCost + planner->heuristicCost * hWeight;
						planner->parent = currPlanNode;
						openQueue.push(planner);
					}
				}
				else
				{
					PlannerNode* succNode = new PlannerNode();
					visited[successor] = succNode;
					succNode->searchNode = successor;
					succNode->givenCost = tempGivenCost;
					succNode->heuristicCost = estimate(successor, goalNode);
					succNode->finalCost = succNode->givenCost + succNode->heuristicCost * hWeight;
					succNode->parent = currPlanNode;
					openQueue.push(succNode);
				}
			}
			// if the timeslice is zero, only one itteration is performed
			if (timeslice == 0)
			{
				break;
			}
		}
		
	}

	void PathSearch::exit()
	{
		// Clear the containers for PathSearching //
		openQueue.clear();
		visited.clear();
		goalFound == false;
	}

	void PathSearch::shutdown()
	{
		// Clear the map //
		nodes.clear();
	}

	bool PathSearch::isDone() const
	{
		return goalFound;
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		std::vector<Tile const*> solutionNodes;
		// Point to 
		PlannerNode* planner = openQueue.front();
		//solutionNodes.push_back(planner->searchNode->tile);

		// Iterate till we reach the end //
		while (planner != nullptr)
		{
			// Fill the tile Blue

			// Push into the solution  container //
			solutionNodes.push_back(planner->searchNode->tile);

			if (planner->parent != NULL)
			{
				planner->searchNode->tile->addLineTo(planner->parent->searchNode->tile, 0xffff0000);
			}

			// Move to the next node //
			planner = planner->parent;
		}

		return solutionNodes;
	}

	void PathSearch::GetEvenNeigbors(int row, int col, SearchNode* CurrNode)
	{
		// Check the Neighbors of the currNode
		// { 0, -1 }
		Tile* neighborTile = tileMap->getTile(row - 1, col);

		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			// Point to the Node conatining the Neighbor Tile 
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { -1, -1 }
		neighborTile = tileMap->getTile(row - 1, col - 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			// Point to the Node conatining the Neighbor Tile 
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { -1, 0 }
		neighborTile = tileMap->getTile(row, col - 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			// Point to the Node conatining the Neighbor Tile 
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// {  1, 0 }
		neighborTile = tileMap->getTile(row, col + 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			// Point to the Node conatining the Neighbor Tile 
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { -1, 1 }
		neighborTile = tileMap->getTile(row + 1, col);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			// Point to the Node conatining the Neighbor Tile 
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { 0, 1 }
		neighborTile = tileMap->getTile(row + 1, col - 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			// Point to the Node conatining the Neighbor Tile 
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}
	}

	void PathSearch::GetOddNeighbors(int row, int col, SearchNode* CurrNode)
	{
		// Check the Neighbors of the currNode
		// { 1, -1 }
		Tile* neighborTile = tileMap->getTile(row - 1, col);
		if (neighborTile != 0)
		{
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0 && neighborTile->getWeight() > 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { 1, 0 }
		neighborTile = tileMap->getTile(row, col + 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { 1, 1 }
		neighborTile = tileMap->getTile(row + 1, col);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			//SearchNode *NodeIndex= nodes[neighborTile];
			//if (NodeIndex != nullptr)
			//{
				//Tile* curTile = NodeIndex->tile;
				//float w = curTile->getWeight();
				SearchNode* neighborNode = nodes[neighborTile];
				if (neighborNode != 0)
				{
					CurrNode->neighbors.push_back(neighborNode);
				}
			//}
		}

		// { -1, 1 }
		neighborTile = tileMap->getTile(row + 1, col + 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { -1, 0 }
		neighborTile = tileMap->getTile(row - 1, col + 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}

		// { 0, -1 }
		neighborTile = tileMap->getTile(row, col - 1);
		if (neighborTile != 0 && neighborTile->getWeight() > 0)
		{
			SearchNode* neighborNode = nodes[neighborTile];
			if (neighborNode != 0)
			{
				CurrNode->neighbors.push_back(neighborNode);
			}
		}
	}

	float PathSearch::estimate(SearchNode* start, SearchNode* goal)
	{
		// Get the differnce from the start to the goal //
		float deltaX = abs(start->tile->getXCoordinate() - goal->tile->getXCoordinate());
		float deltaY = abs(start->tile->getYCoordinate() - goal->tile->getYCoordinate());

		// Calcuate the distance //
		float dist = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
		return dist;
	}
}}  // namespace fullsail_ai::algorithms

