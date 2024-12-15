#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "path_interface/srv/generate_graph.hpp"

#include "Astar.h"

class AStarPlanner : public rclcpp::Node {
	public:
		AStarPlanner() : Node("astar_planner") {
			client_ = this->create_client<path_interface::srv::GenerateGraph>("generate_graph");
		}

        // for graph construction
		std::map<id_t, GVertex> vertices;


        void getGraph(float x, float y) {
			if (!client_->wait_for_service(std::chrono::seconds(5))) {
				RCLCPP_ERROR(this->get_logger(), "Cannot call generate_graph service after waiting 5 seconds");
				return;
			}


			auto request = std::make_shared<path_interface::srv::GenerateGraph::Request>();
			request->x = x;
			request->y = y;

			const auto future = client_->async_send_request(request, std::bind(&AStarPlanner::generate_graph_response_callback, this, std::placeholders::_1));
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);


			try {
				auto graph_vertices = this->graph_response_->vertices;
				std::map<id_t, std::vector<id_t>> graph_edges;
				for (auto vertex : graph_vertices) {
					GVertex v = {(id_t)vertex.id, "", vertex.x, vertex.y, 0.0};
					for (auto edge : vertex.edges) {
						graph_edges[vertex.id].push_back(edge);
					}
					vertices[vertex.id] = v;
				}


				for (auto it = vertices.begin(); it != vertices.end(); ++it) {
					for (id_t edge_id : graph_edges[it->first]) {

						float dist = sqrt(pow(it->second.getx() - vertices[edge_id].getx(), 2) + pow(it->second.gety() - vertices[edge_id].gety(), 2));
						it->second.addEdge(edge_id, dist, "");
					}
					
				}

				RCLCPP_INFO(this->get_logger(), "#vertices: %zu", vertices.size());

				Astar astar;

				std::vector<id_t> path = astar.findPath(this->graph_response_->start_id, this->graph_response_->gate_id, vertices);

				for (auto id : path) {
				RCLCPP_INFO(this->get_logger(), "path id: %d", id);

				}

				RCLCPP_INFO(this->get_logger(), "path size: %zu", path.size());

				
			} catch (const std::exception &e) {
            	RCLCPP_ERROR(this->get_logger(), "Service call for generate_graph failed: %s", e.what());
        	}
		}

        void generate_graph_response_callback(rclcpp::Client<path_interface::srv::GenerateGraph>::SharedFuture future) {
            this->graph_response_ = future.get();
        }

        

    private:
        rclcpp::Client<path_interface::srv::GenerateGraph>::SharedPtr client_;
        path_interface::srv::GenerateGraph::Response::SharedPtr graph_response_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AStarPlanner>();

    node->getGraph(1, 2);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}