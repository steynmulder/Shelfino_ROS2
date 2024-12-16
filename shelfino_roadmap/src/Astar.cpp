#include "Astar.h"
#include <cmath>
#include <set>
#include <algorithm>
#include <map>
#include "Dubins.h"

std::vector<id_t> Astar::findPath(const id_t start_id, const id_t end_id, std::map<id_t, GVertex>& graph)
{
	std::vector<id_t> dest;
    std::multiset<GVertex*, CompareF> openSet;
    std::set<GVertex> closedSet;

	GVertex *Vcurrent, *Vnext;
	openSet.insert(&graph[start_id]);
	do
	{
        Vcurrent = *openSet.begin();
		openSet.erase(openSet.begin());

		if(Vcurrent->getStateID() == end_id) break; //If I find destination vertex
		closedSet.insert(*Vcurrent);

		for (const GEdge& i : (*Vcurrent).getEdgeList())
		{
            Vnext = &graph[i.getDestVID()];
			if (closedSet.find(*Vnext) != closedSet.end()) continue;
			
			// Compute Dubins path cost instead of straight-line edge cost
            // double dubinsCost = dubinsShortestPath(
            //     Vcurrent->getx(), Vcurrent->gety(), Vcurrent->getOrientation(),  // Current vertex
            //     Vnext->getx(), Vnext->gety(), Vnext->getOrientation(),          // Next vertex
            //     1.0).L;                            // Turning radius TODO GET RADIUS

            // Total cost calculation
            auto g = Vcurrent->getg(); // + dubinsCost;
            auto f = g + heuristic_distance_estimator(*Vnext, graph[end_id]);

			if (openSet.count(Vnext)==0){   //If openSet does not contain Vnext
				Vnext->setf(f);
				Vnext->setg(g);
                Vnext->setPredecessor(Vcurrent);    //Take note of the predecessor to traceback
				openSet.insert(Vnext);  //Inserting in order according to the comparator
			} else if(f < Vnext->getf()){
				Vnext->setf(f);
				Vnext->setg(g);
                Vnext->setPredecessor(Vcurrent);    //Update predecessor
			}
		}
	} while (!openSet.empty());

    GVertex* current = &graph[end_id];

    while (current != &graph[start_id]) {
        dest.push_back(current->getStateID());
        current = &graph[current->getPredecessor()];
    }
    dest.push_back(current->getStateID());
    std::reverse(dest.begin(), dest.end());

    return dest;
}

double Astar::heuristic_distance_estimator(const GVertex& vnext, const GVertex& vend)
{
	return sqrt((vend.getx() - vnext.getx()) * (vend.getx() - vnext.getx()) + (vend.gety() - vnext.gety()) * (vend.gety() - vnext.gety()));
}