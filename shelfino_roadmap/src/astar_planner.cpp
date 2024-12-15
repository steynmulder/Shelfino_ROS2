#include <map>

#include "rclcpp/rclcpp.hpp"
#include "path_interface/srv/generate_graph.hpp"

#include "Astar.h"

class AStarPlanner : public rclcpp::Node {
	public:
		AStarPlanner() : Node("astar_planner") {
			client_ = this->create_client<path_interface::srv::GenerateGraph>("generate_graph");
		}

        // for graph construction
		map<id_t, Vertex> vertices;


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
				auto graph_edges = this->graph_response_->graph.edges;
				auto graph_vertices = this->graph_response_->graph.vertices;
				id_t id = 0;
				for (auto vertex : graph_vertices) {
					Vertex v = {id, "", vertex.x, vertex.y, 0.0};
					id++;
				}
                // RCLCPP_INFO(this->get_logger(), "%zu", this->graph_response_->graph.edges.size());
				// for (auto edge : this->graph_response_->graph.edges) {
				// 	RCLCPP_INFO(this->get_logger(), "(%f, %f) - (%f, %f)", edge.start.x, edge.start.y, edge.end.x, edge.end.y);
				// }
				
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