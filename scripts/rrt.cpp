#include <iostream>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <random>
#include <math.h>
#include <tuple>

namespace py = pybind11;

class Node{
    public:
        std::vector<std::vector<double>> configuration;
        Node* parent;
        int id;
        
        Node(std::vector<std::vector<double>> configuration, Node* parent, int id){
            // copy the vector configuration
            for (int i = 0; i < configuration.size(); i++){
                this->configuration.push_back(std::vector<double>{configuration[i][0], configuration[i][1]});
            }
            this->parent = parent;
            this->id = id;
        }

        std::vector<std::vector<double>>* get_configuration(){
            return &this->configuration;
        }

        Node* get_parent(){
            return this->parent;
        }

        void set_parent(Node* parent){
            this->parent = parent;
        }

        int get_id(){
            return this->id;
        }

        int get_parent_id(){
            if (this->parent == nullptr)
                return -1;
            return this->parent->get_id();
        }
};

std::vector<std::vector<double>> get_random_configuration(
    int num_robots,
    int env_min_x,
    int env_min_y,
    int env_width,
    int env_height)
    {
        std::vector<std::vector<double>> random_configuration;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(env_min_x, env_min_x + env_width);
        std::uniform_real_distribution<> dis_y(env_min_y, env_min_y + env_height);
        for (int i = 0; i < num_robots; i++){
            random_configuration.push_back(std::vector<double>{dis_x(gen), dis_y(gen)});
        }
        return random_configuration;
    }
    
Node* get_nearest_node(
    std::vector<std::vector<double>> random_configuration,
    std::vector<Node> &nodes)
    // WARNING: NOT SURE IF THIS IS CORRECT
    {
        Node* nearest_node = &nodes[0];
        double nearest_node_distance = 100000000;
        for (int i = 0; i < nodes.size(); i++){
            double distance = 0;
            for (int j = 0; j < random_configuration.size(); j++){
                auto configuration = nodes[i].get_configuration();
                distance += std::sqrt(
                    std::pow(random_configuration[j][0] - (*configuration)[j][0], 2) +
                    std::pow(random_configuration[j][1] - (*configuration)[j][1], 2)); 
            }
            if (distance < nearest_node_distance){
                nearest_node = &nodes[i];
                nearest_node_distance = distance;
            }
        }
        return nearest_node;
    }

bool check_for_collision(
    std::vector<std::vector<double>> &new_configuration,
    std::vector<std::vector<std::vector<int>>> &img)
    {
        for (int i = 0; i < new_configuration.size(); i++){
            int x = new_configuration[i][0];
            int y = new_configuration[i][1];
            //std::cout << "x: " << x << " y: " << y << std::endl;
            //std::cout << "Boundaries are: " << img.size() << " " << img[0].size() << std::endl;
            if(x > 0 && y > 0 && x < (img.size() - 1) && y < (img[0].size() - 1)){
                //std::cout << "In the boundaries" << std::endl;
                if(img[x][y][0] != 0 || img[x][y][1] != 0 || img[x][y][2] != 0){
                    //std::cout << "Collision in " << x << " " << y << std::endl;
                    return true;
                }
                if(img[x-1][y-1][0] != 0 || img[x-1][y-1][1] != 0 || img[x-1][y-1][2] != 0){
                    //std::cout << "Collision in " << x-1 << " " << y-1 << std::endl;
                    return true;
                }
                if(img[x-1][y][0] != 0 || img[x-1][y][1] != 0 || img[x-1][y][2] != 0){
                    //std::cout << "Collision in " << x-1 << " " << y << std::endl;
                    return true;
                }
                if(img[x-1][y+1][0] != 0 || img[x-1][y+1][1] != 0 || img[x-1][y+1][2] != 0){
                    //std::cout << "Collision in " << x-1 << " " << y+1 << std::endl;
                    return true;
                }
                if(img[x+1][y-1][0] != 0 || img[x+1][y-1][1] != 0 || img[x+1][y-1][2] != 0){
                    //std::cout << "Collision in " << x+1 << " " << y-1 << std::endl;
                    return true;
                }
                if(img[x+1][y][0] != 0 || img[x+1][y][1] != 0 || img[x+1][y][2] != 0){
                    //std::cout << "Collision in " << x+1 << " " << y << std::endl;
                    return true;
                }
                if(img[x+1][y+1][0] != 0 || img[x+1][y+1][1] != 0 || img[x+1][y+1][2] != 0){
                    //std::cout << "Collision in " << x+1 << " " << y+1 << std::endl;
                    return true;
                }
                if(img[x][y-1][0] != 0 || img[x][y-1][1] != 0 || img[x][y-1][2] != 0){
                    //std::cout << "Collision in " << x << " " << y-1 << std::endl;
                    return true;
                }
                if(img[x][y+1][0] != 0 || img[x][y+1][1] != 0 || img[x][y+1][2] != 0){
                    //std::cout << "Collision in " << x << " " << y+1 << std::endl;
                    return true;
                }
            }
            
            // now check for self collisions
        }
        return false;
    }

bool check_if_arrived(
    std::vector<std::vector<double>> &new_configuration,
    std::vector<double> &robots_goal_x,
    std::vector<double> &robots_goal_y,
    int step_size)
    {
        // calculate the distance between the new configuration and the goal for each agent
        for (int i = 0; i < new_configuration.size(); i++){
            double distance = sqrt(pow(new_configuration[i][0] - robots_goal_x[i], 2) + pow(new_configuration[i][1] - robots_goal_y[i], 2));
            if (i == 1){
                std::cout << "Distance is: " << distance << std::endl;
            }
            if (distance > step_size){
                return false;
            }
        }
        return true;
}

std::vector<std::tuple<std::vector<std::vector<double>>, int>> rrt(
    int num_robots,
    std::vector<double> robots_start_x,
    std::vector<double> robots_start_y,
    std::vector<double> robots_goal_x,
    std::vector<double> robots_goal_y,
    double rrt_step_size,
    double rrt_rewire_distance,
    int rrt_max_iter,
    int env_min_x,
    int env_min_y,
    int env_width,
    int env_height,
    std::vector<std::vector<double>> speed_for_freq,
    std::vector<std::vector<std::vector<int>>> &img)
    {
        std::vector<std::tuple<std::vector<std::vector<double>>, int>> return_list;
        // merge robots start x and y into a vector of tuples
        std::vector<std::vector<double>> robots_start;
        for (int i = 0; i < num_robots; i++){
            robots_start.push_back(std::vector<double>{robots_start_x[i], robots_start_y[i]});
        }
        // merge robots goal x and y into a vector of tuples
        std::vector<std::vector<double>> robots_goal;
        for (int i = 0; i < num_robots; i++){
            robots_goal.push_back(std::vector<double>{robots_goal_x[i], robots_goal_y[i]});
        }

        std::vector<Node> nodes;
        int node_id = 0;
        nodes.push_back(Node(robots_start, NULL, node_id++));
        return_list.push_back(std::make_tuple(robots_start, -1));

        // loop with rrt_max_iter
        for(int i = 0; i < rrt_max_iter; i++){

            // get a random configuration
            std::vector<std::vector<double>> random_configuration = get_random_configuration(num_robots, env_min_x, env_min_y, env_width, env_height);

            // find the nearest node
            Node* nearest_node = get_nearest_node(random_configuration, nodes);

            auto nearest_node_configuration = nearest_node->get_configuration();

            // calculate the angle between the nearest node and the random configuration
            double angle = atan2(random_configuration[0][1] - (*nearest_node_configuration)[0][1], random_configuration[0][0] - (*nearest_node_configuration)[0][0]);

            // calculate the new configuration by using speed for freq and random angle
            std::vector<std::vector<double>> new_configuration;
            for (int j = 0; j < num_robots; j++){
                double new_x = (*nearest_node_configuration)[j][0] + speed_for_freq[j][0] * cos(angle);
                double new_y = (*nearest_node_configuration)[j][1] + speed_for_freq[j][0] * sin(angle);
                new_configuration.push_back(std::vector<double>{new_x, new_y});
            }

            /*
            // choose a random node by using the index
            int random_index = rand() % nodes.size();
            Node* random_node = &nodes[random_index];

            // choose a random angle between 0 - 2pi
            double random_angle = (double)rand() / (double)RAND_MAX * 2 * M_PI;

            // calculate the new configuration by using speed for freq and random angle
            std::vector<std::vector<double>> new_configuration;
            for (int j = 0; j < num_robots; j++){
                //std::cout << "193" << std::endl;
                double new_x = (*random_node->get_configuration())[j][0] + speed_for_freq[j][0] * cos(random_angle);
                //std::cout << "195" << std::endl;
                double new_y = (*random_node->get_configuration())[j][1] + speed_for_freq[j][0] * sin(random_angle);
                //std::cout << "197" << std::endl;
                new_configuration.push_back(std::vector<double>{new_x, new_y});
            }

            std::vector<std::vector<double>> new_configuration = get_new_configuration(
                random_configuration,
                nearest_node->get_configuration(),
                rrt_step_size);
            */
            /*
            std::cout << "New configuration: ";
            for (int i = 0; i < num_robots; i++){
                std::cout << "(" << new_configuration[i][0] << ", " << new_configuration[i][1] << ") ";
            }
            std::cout << std::endl;
            */
            //std::cout << "Checking collision" << std::endl;
            // check if new configuration is valid
            bool is_there_collision = check_for_collision(new_configuration, img);
            if (!is_there_collision){
                //std::cout << "Valid configuration" << std::endl;
                nodes.push_back(Node(new_configuration, nearest_node, node_id++));
                return_list.push_back(std::make_tuple(new_configuration, nearest_node->get_id()));
                bool is_arrived = check_if_arrived(new_configuration, robots_goal_x, robots_goal_y, rrt_step_size);
                if (is_arrived){
                    std::cout << "Arrived in " << i << " steps" << std::endl;
                    break;
                }
            }
            else{
                //std::cout << "Invalid configuration" << std::endl;
            }

        }
        
        return return_list;
    }

PYBIND11_MODULE(rrt_fast, m){
    m.def("rrt", &rrt, "A function which implements RRT");
}

int main() {
    std::vector<double> robots_start_x = {100, 300};
    std::vector<double> robots_start_y = {100, 200};
    std::vector<double> robots_goal_x = {200, 200};
    std::vector<double> robots_goal_y = {100, 200};
    double rrt_step_size = 10.0;
    double rrt_rewire_distance = 10.0;
    int rrt_max_iter = 50000;
    int env_min_x = -100;
    int env_min_y = -100;
    int env_width = 600;
    int env_height = 600;
    std::vector<std::vector<std::vector<int>>> img;
    std::vector<std::vector<double>> speed_for_freq = {{1, 1}, {1, 1}};
    rrt(2, robots_start_x, robots_start_y, robots_goal_x, robots_goal_y, rrt_step_size, rrt_rewire_distance, rrt_max_iter, env_min_x, env_min_y, env_width, env_height, speed_for_freq, img);
    return 0;
}

