#include "RRTstar.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include "Dubins.h"


class RRTstar {
public:
    RRTstar(Graph* graph, double step_size, double search_radius)
        : graph(graph), step_size(step_size), search_radius(search_radius) {}

    std::vector<id_t> findPath(double start_x, double start_y, double goal_x, double goal_y);

private:
    Graph* graph;
    double step_size;
    double search_radius;

    // MULTI AGENT
    void resolveConflict(size_t t, id_t conflict_id, id_t end_id, vector<string>& names, map<string, vector<id_t>>& paths, map<id_t, GVertex>& graph);
    map<string, vector<id_t>> generatePaths(const vector<id_t> start_ids, const vector<string> names, const id_t end_id, map<id_t, GVertex>& graph) {

};

// Main RRT* pathfinding function
std::vector<id_t> RRTstar::findPath(double start_x, double start_y, double goal_x, double goal_y) {
    std::vector<Node*> tree;
    tree.push_back(new Node(0, start_x, start_y));

    Node* goal_node = nullptr;
    std::random_device rd;
    std::mt19937 gen(rd());
    // TODO define borders 
    std::uniform_real_distribution<> dis_x(0, graph->getWidth()); 
    std::uniform_real_distribution<> dis_y(0, graph->getHeight());

    // TODO Define obstacles for x,y, radius or edge length
    std::uniform_real_distribution<> obs(0, graph->getObstacles());


    while (goal_node == nullptr) {
        double rand_x = dis_x(gen);
        double rand_y = dis_y(gen);

        Node* nearest = nearestNode(tree, rand_x, rand_y);
        double theta = atan2(rand_y - nearest->y, rand_x - nearest->x);
        double new_x = nearest->x + step_size * cos(theta);
        double new_y = nearest->y + step_size * sin(theta);

        if (!isCollisionFree(nearest->x, nearest->y, new_x, new_y))
            continue;

        // TODO: delete new_node?
        Node* new_node = new Node(tree.size(), new_x, new_y, nearest, nearest->cost + step_size);
        tree.push_back(new_node);

        auto near_nodes = nearNodes(tree, new_x, new_y);
        rewire(tree, new_node, near_nodes);

        if (euclideanDistance(new_x, new_y, goal_x, goal_y) <= step_size) {
            goal_node = new Node(tree.size(), goal_x, goal_y, new_node, new_node->cost + step_size);
            tree.push_back(goal_node);
        }
    }

    // Reconstruct path from goal_node
    std::vector<id_t> path;
    Node* current = goal_node;
    while (current) {
        path.push_back(current->id);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
}

// NEAREST NODE 
Node* RRTstar::nearestNode(const std::vector<Node*>& tree, double x, double y) {
    Node* nearest = nullptr;
    double min_distance = std::numeric_limits<double>::max();

    for (Node* node : tree) {
        double distance = euclideanDistance(node->x, node->y, x, y);
        if (distance < min_distance) {
            min_distance = distance;
            nearest = node;
        }
    }
    return nearest;
}

// NEAR NODE
std::vector<Node*> RRTstar::nearNodes(const std::vector<Node*>& tree, double x, double y) {
    std::vector<Node*> near_nodes;
    for (Node* node : tree) {
        if (euclideanDistance(node->x, node->y, x, y) <= search_radius) {
            near_nodes.push_back(node);
        }
    }
    return near_nodes;
}

// COLLISION CHECK between point(x1,y1) and point(x2,y2)
bool RRTstar::isCollisionFree(double x1, double y1, double x2, double y2) {
    const double step_size = 0.1;
    double dx = abs(x1-x2);
    double dy = abs(y1-y2);
    double distance = euclideanDistance(x1,y1,x2,y2);
    int steps = static_cast<int><(distance / step_size);

    // TODO: define the obs
    for (int i = 0; i <= steps; ++i){
        double xi = x1 + i * step_size(dx/distance);
        double yi = y1 + i * step_size(dy/distance);

        // CHECK 1: for cylinder
        // TODO 
        for (const auto& cylinder : obstacles.cylinders){
            if (euclideanDistance(xi, yi, cylinder.x, cylinder.y) <= cylinder.radius +0.2){
                return false;
            }
        }

        // CHECK 2: for boxes
        for(cosnt auto& box : obstacles.boxes){
            double half_length_x = box.length_x / 2.0;
            double half_length_y = box.length_y / 2.0;

            if (xi >= (box.x - half_length_x-0.2) && xi <= (box.x + half_length_x+0.2) &&
                yi >= (box.y - half_length_y-0.2) && yi <= (box.y + half_length_y+0.2)
            ){
                return false;
            }
        }
    }

    return true;
}

double RRTstar::euclideanDistance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// REWIRE
void RRTstar::rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& near_nodes) {
    for (Node* near_node : near_nodes) {
        double new_cost = new_node->cost + euclideanDistance(new_node->x, new_node->y, near_node->x, near_node->y);
        if (new_cost < near_node->cost) {
            near_node->parent = new_node;
            near_node->cost = new_cost;
        }
    }
}




// MULTI AGENT PART
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


void RRTstar::resolveConflict(size_t t, id_t conflict_id, id_t end_id, vector<string>& names, map<string, vector<id_t>>& paths, map<id_t, GVertex>& graph) {
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

map<string, vector<id_t>> RRTstar::generatePaths(const vector<id_t> start_ids, const vector<string> names, const id_t end_id, map<id_t, GVertex>& graph) {
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