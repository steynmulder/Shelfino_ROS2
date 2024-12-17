#include "Astar.h"
#include <cmath>
#include <set>
#include <algorithm>
#include <map>
#include "Dubins.h"

using namespace std;

constexpr int MAX_ITERATIONS = 1000;

vector<id_t> Astar::findPath(const id_t start_id, const id_t end_id, map<id_t, GVertex>& graph)
{
	vector<pair<string, vector<id_t>>> result;

	vector<id_t> dest;
	multiset<GVertex*, CompareF> openSet;
	set<GVertex> closedSet;

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
	reverse(dest.begin(), dest.end());
	
    return dest;
}

map<id_t, vector<string>> getConflicts(vector<pair<string, id_t>>& positions) {
	map<id_t, vector<string>> conflicting_names;
	for (pair<string, id_t> position : positions) {
		conflicting_names[position.second].push_back(position.first);
	}

	for (auto it = conflicting_names.begin(); it != conflicting_names.end(); ++it) {
		if (it->second.size() < 2) {
			it = conflicting_names.erase(it);
		}
	}

	return conflicting_names;
}

void Astar::resolveConflict(size_t t, id_t conflict_id, id_t end_id, vector<string>& names, map<string, vector<id_t>>& paths, map<id_t, GVertex>& graph) {
	vector<id_t> chosen_ids;
	chosen_ids.push_back(conflict_id);
	for (size_t i = 1; i < names.size(); ++i) { // leave the first element as is
		id_t previous_id = paths[names[i]][t - 1];
		GVertex previous_vertex = graph[previous_id];
		list<id_t> options = previous_vertex.getDestinationsList();
		for (id_t option : options) {
			if (find(chosen_ids.begin(), chosen_ids.end(), option) == chosen_ids.end() &&
				find(chosen_ids.begin(), chosen_ids.begin() + (t-1), option) != chosen_ids.begin() + (t-1)) {
				vector<id_t> subpath = Astar::findPath(option, end_id, graph);
				paths[names[i]].resize(paths[names[i]].size() - (t - 1));
				paths[names[i]].insert(paths[names[i]].end(), subpath.begin(), subpath.end());
				chosen_ids.push_back(option);
				break;
			}
		}
	}
}

map<string, vector<id_t>> Astar::generatePaths(const vector<id_t> start_ids, const vector<string> names, const id_t end_id, map<id_t, GVertex>& graph) {
	vector<vector<pair<string, id_t>>> positions_time;
	map<string, vector<id_t>> paths;
	for (size_t robot_id = 0; robot_id < names.size(); ++robot_id) {
		vector<id_t> path = Astar::findPath(start_ids[robot_id], end_id, graph);
		vector<pair<string, id_t>> positions;
		paths[names[robot_id]] = path;
		for (size_t t = 0; t < path.size(); ++t) { // for each time step ~= each vertex visited
			positions.push_back(make_pair(names[robot_id], path[t]));
		}
		positions_time.push_back(positions);
	}

	if (names.size() > 1) {
		bool conflict = true;
		int iterations = 0;

		while (conflict && iterations < MAX_ITERATIONS) {
			conflict = false;
			for (size_t t = 1; t < positions_time.size() - 1; ++t) { // ASSUMPTION: the robots do not start in the same node & they can be on the goal together
				// make vector of vectors of all (name, id)s that are conflicting in this time step
				map<id_t, vector<string>> conflicts = getConflicts(positions_time[t]);

				if (!conflicts.empty()) {
					// resolve conflicts
					resolveConflict(t, conflicts.begin()->first, end_id, conflicts.begin()->second, paths, graph);


					conflict = true;
					break; // start from beginning with resolving conflicts
				}
			}
			++iterations;
		}
	}

	

	return paths;
}

double Astar::heuristic_distance_estimator(const GVertex& vnext, const GVertex& vend)
{
	return sqrt((vend.getx() - vnext.getx()) * (vend.getx() - vnext.getx()) + (vend.gety() - vnext.gety()) * (vend.gety() - vnext.gety()));
}