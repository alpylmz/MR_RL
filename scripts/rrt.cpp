#include <iostream>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <random>
#include <math.h>
#include <tuple>
#include <chrono>

namespace py = pybind11;

std::vector<std::vector<double>> get_random_configuration(
    int num_robots,
    int env_min_x,
    int env_min_y,
    int env_width,
    int env_height,
    std::uniform_real_distribution<double> &dis_x,
    std::uniform_real_distribution<double> &dis_y,
    std::mt19937 &gen)
    {
        // initialize random_configuration with reserved space num_robots
        std::vector<std::vector<double>> random_configuration;
        random_configuration.reserve(num_robots);
        for (int i = 0; i < num_robots; i++){
            random_configuration.push_back(std::vector<double>{});
            random_configuration[i].push_back(dis_x(gen));
            random_configuration[i].push_back(dis_y(gen));
        }
        return random_configuration;
    }
    
int get_nearest_node_index(
    std::vector<std::vector<double>> random_configuration,
    std::vector<std::tuple<std::vector<std::vector<double>>, int>> &nodes)
    // WARNING: NOT SURE IF THIS IS CORRECT
    {
        int nearest_node_index = 0;
        double nearest_node_distance = 100000000;
        for (int i = 0; i < nodes.size(); i++){
            double distance = 0;
            for (int j = 0; j < random_configuration.size(); j++){
                auto configuration = std::get<0>(nodes[i]);
                distance +=
                    std::pow(random_configuration[j][0] - configuration[j][0], 2) +
                    std::pow(random_configuration[j][1] - configuration[j][1], 2); 
            }
            if (distance < nearest_node_distance){
                nearest_node_index = i;
                nearest_node_distance = distance;
            }
        }
        return nearest_node_index;
    }

std::vector<std::vector<int>> process_img(
    std::vector<std::vector<std::vector<int>>> &img,
    double rrt_step_size
    )
    {
        std::vector<std::vector<int>> processed_img;
        for (int i = 0; i < img.size(); i++){
            processed_img.push_back(std::vector<int>{});
            for (int j = 0; j < img[0].size(); j++){
                processed_img[i].push_back(0);
                
            }
        }

        // Now, start iterating over the image. 
        // If the pixel is not black, we will make the values 
        // around the pixel 1, so that the robot can't go there
        // the boundaries are defined with rrt_step_size

        for (int i = 0; i < img.size(); i++){
            for (int j = 0; j < img[0].size(); j++){
                if(img[i][j][0] != 0 || img[i][j][1] != 0 || img[i][j][2] != 0){
                    //std::cout << "Pixel is not black" << std::endl;
                    int x_min = i - rrt_step_size;
                    int x_max = i + rrt_step_size;
                    int y_min = j - rrt_step_size;
                    int y_max = j + rrt_step_size;
                    for (int x = x_min; x < x_max; x++){
                        for (int y = y_min; y < y_max; y++){
                            if(x > 0 && y > 0 && x < (img.size() - 1) && y < (img[0].size() - 1)){
                                processed_img[x][y] = 1;
                            }
                        }
                    }
                }
            }
        }

        return processed_img;


    }

bool check_for_collision(
    std::vector<std::vector<double>> &new_configuration,
    std::vector<std::vector<int>> &img)
    {
        for (int i = 0; i < new_configuration.size(); i++){
            int x = new_configuration[i][0];
            int y = new_configuration[i][1];
            if (x < 0 || y < 0 || x >= img.size() || y >= img[0].size()){
                return true;
            }
            if (img[x][y] == 1){
                return true;
            }
        }
        return false;
    }

bool check_if_arrived(
    std::vector<std::vector<double>> &new_configuration,
    std::vector<double> &robots_goal_x,
    std::vector<double> &robots_goal_y,
    int step_size)
    {
        double step_size_squared = std::pow(step_size, 2);
        // calculate the distance between the new configuration and the goal for each agent
        for (int i = 0; i < new_configuration.size(); i++){
            double distance = pow(new_configuration[i][0] - robots_goal_x[i], 2) + pow(new_configuration[i][1] - robots_goal_y[i], 2);
            if (i == 1){
                std::cout << "Distance squared is: " << distance << std::endl;
            }
            if (distance > step_size_squared){
                return false;
            }
        }
        return true;
}

std::vector<std::tuple<double, double>> calculate_speeds(
    std::vector<std::vector<double>> &inversed_frequencies,
    std::vector<std::vector<double>> &current_configuration,
    std::vector<std::vector<double>> &goal_configuration,
    int num_robots){

    // create a matrix of size (num_robots, 2) by current_configuration - goal_configuration
    std::vector<std::vector<double>> displacements;
    for (int i = 0; i < num_robots; i++){
        displacements.push_back(std::vector<double>{});
        for (int j = 0; j < 2; j++){
            displacements[i].push_back(current_configuration[i][j] - goal_configuration[i][j]);
        }
    }

    // multiply the matrix 
    // inversed_frequencies * displacements
    // and store the result in time_for_each_freq matrix
    std::vector<std::vector<double>> time_for_each_freq;
    for (int i = 0; i < num_robots; i++){
        time_for_each_freq.push_back(std::vector<double>{});
        for (int j = 0; j < num_robots; j++){
            double sum = 0;
            for (int k = 0; k < num_robots; k++){
                sum += inversed_frequencies[i][k] * displacements[k][j];
            }
            time_for_each_freq[i].push_back(sum);
        }
    }

    std::vector<std::tuple<double, double>> angle_and_time;
    // angle is found by atan2(y, x) in each vector
    // time is found by sqrt(x^2 + y^2) in each vector
    for (int i = 0; i < time_for_each_freq.size(); i++){
        double angle = atan2(time_for_each_freq[i][1], time_for_each_freq[i][0]);
        double time = sqrt(pow(time_for_each_freq[i][0], 2) + pow(time_for_each_freq[i][1], 2));
        angle_and_time.push_back(std::make_tuple(angle, time));
    }

    return angle_and_time;
}

bool extend_tree(
    std::vector<std::tuple<std::vector<std::vector<double>>, int>> &tree,
    std::vector<std::vector<double>> &speed_for_freq,
    std::vector<std::tuple<double, double>> &angle_and_time,
    std::vector<std::vector<int>> &img,
    std::vector<std::vector<double>> &random_configuration,
    int& node_id,
    int nearest_node_index,
    double rrt_step_size,
    int num_robots){
        //normalize angle_and_time for all robots
        // the max value for time should be 1,
        // angle should not be touched
        double max_time = 0;
        for (int i = 0; i < angle_and_time.size(); i++){
            if (std::get<1>(angle_and_time[i]) > max_time){
                max_time = std::get<1>(angle_and_time[i]);
            }
        }
        for (int i = 0; i < angle_and_time.size(); i++){
            std::get<1>(angle_and_time[i]) /= max_time;
        }
        // firstly we need to decide for the step distances and angles that we are taking
        // steps[i][j][k], i is the robot, j is the frequency, k is 0 or 1, x or y
        std::vector<std::vector<std::tuple<double, double>>> steps;
        for (int j = 0; j < num_robots; j++){
            steps.push_back(std::vector<std::tuple<double, double>>{});
            for (int k = 0; k < num_robots; k++){
                // use the transpose of speed_for_freq
                double add_x = -1 * speed_for_freq[k][j] * cos(std::get<0>(angle_and_time[j])) * std::get<1>(angle_and_time[j]);
                double add_y = -1 * speed_for_freq[k][j] * sin(std::get<0>(angle_and_time[j])) * std::get<1>(angle_and_time[j]);
                steps[j].push_back(std::make_tuple(add_x, add_y));
            }
        }
        bool is_there_collision = false;
        while(true){
            for(int j = 0; j < num_robots; j++){
                std::vector<std::vector<double>> intermediate_configuration;
                for(int k = 0; k < num_robots; k++){
                    double new_x = std::get<0>(tree[nearest_node_index])[k][0] + std::get<0>(steps[j][k]);
                    double new_y = std::get<0>(tree[nearest_node_index])[k][1] + std::get<1>(steps[j][k]);

                    intermediate_configuration.push_back(std::vector<double>{new_x, new_y});
                }
                is_there_collision = check_for_collision(intermediate_configuration, img);
                if (is_there_collision){
                    break;
                }

                tree.push_back(std::make_tuple(intermediate_configuration, nearest_node_index));
                nearest_node_index = node_id;
                node_id++;
            
                // are we close enough to the random configuration?
                double distance = 0;
                for (int j = 0; j < num_robots; j++){
                    distance += sqrt(
                        pow(
                            std::get<0>(tree[nearest_node_index])[j][0] 
                            - 
                            random_configuration[j][0], 2
                        ) 
                        + 
                        pow(
                            std::get<0>(tree[nearest_node_index])[j][1]
                            -
                            random_configuration[j][1], 2
                        )
                    );
                }
                if (distance < rrt_step_size){
                    return true;
                }
            
            }
            if (is_there_collision){
                break;
            }

        }
        return false;
    }

std::tuple<std::vector<std::tuple<std::vector<std::vector<double>>, int>>, std::vector<std::tuple<std::vector<std::vector<double>>, int>>> rrt(
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
    std::vector<std::vector<std::vector<int>>> &img,
    std::vector<std::vector<double>> &inversed_frequencies)
    {
        // get the start time
        auto start = std::chrono::high_resolution_clock::now();
        // random process for optimization
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(env_min_x, env_min_x + env_width);
        std::uniform_real_distribution<> dis_y(env_min_y, env_min_y + env_height);
        std::uniform_real_distribution<> dis_angle(0, 2 * M_PI);

        //std::cout << "Processing image" << std::endl;
        auto processed_img = process_img(img, rrt_step_size);
        //std::cout << "Image processed" << std::endl;

        // merge robots start x and y into a vector of tuples
        std::vector<std::vector<double>> robots_start;
        // reserve the capacity of the vector to num_robots
        robots_start.reserve(num_robots);
        for (int i = 0; i < num_robots; i++){
            robots_start.push_back(std::vector<double>{robots_start_x[i], robots_start_y[i]});
        }
        // merge robots goal x and y into a vector of tuples
        std::vector<std::vector<double>> robots_goal;
        // reserve the capacity of the vector to num_robots
        robots_goal.reserve(num_robots);
        for (int i = 0; i < num_robots; i++){
            robots_goal.push_back(std::vector<double>{robots_goal_x[i], robots_goal_y[i]});
        }

        // create the variables for the first tree
        // a vector of tuples
        // tuples' first element is a vector of vectors of doubles
        // tuples' second element is an int
        // tuples' first element consist of the state of the system
        // each element is the coordinate of a robot
        // tuples' second element is the index of the parent node
        std::vector<std::tuple<std::vector<std::vector<double>>, int>> return_list;
        // reserve space for the return list
        return_list.reserve(num_robots);

        int node_id = 1;
        return_list.push_back(std::make_tuple(robots_start, -1));

        // create the variables for the second tree
        std::vector<std::tuple<std::vector<std::vector<double>>, int>> goal_return_list;
        // reserve space for the return list
        goal_return_list.reserve(num_robots);

        int goal_node_id = 1;
        goal_return_list.push_back(std::make_tuple(robots_goal, -1));
        
        // loop with rrt_max_iter
        for(int i = 0; i < rrt_max_iter; i++){
            
            std::vector<std::vector<double>> random_configuration = 
                get_random_configuration(
                    num_robots, 
                    env_min_x, 
                    env_min_y, 
                    env_width, 
                    env_height, 
                    dis_x, 
                    dis_y,
                    gen
                );

            int nearest_node_index = get_nearest_node_index(random_configuration, return_list);
            int goal_nearest_node_index = get_nearest_node_index(random_configuration, goal_return_list);

            std::vector<std::vector<double>> current_configuration;
            for (int j = 0; j < num_robots; j++){
                current_configuration.push_back(
                    std::vector<double>{
                        std::get<0>(return_list[nearest_node_index])[j][0], 
                        std::get<0>(return_list[nearest_node_index])[j][1]
                    }
                );
            }
            std::vector<std::vector<double>> goal_current_configuration;
            for (int j = 0; j < num_robots; j++){
                goal_current_configuration.push_back(
                    std::vector<double>{
                        std::get<0>(goal_return_list[goal_nearest_node_index])[j][0], 
                        std::get<0>(goal_return_list[goal_nearest_node_index])[j][1]
                    }
                );
            }

            auto angle_and_time = calculate_speeds(
                inversed_frequencies, 
                current_configuration, 
                random_configuration, 
                num_robots
            );
            
            auto goal_angle_and_time = calculate_speeds(
                inversed_frequencies, 
                goal_current_configuration, 
                random_configuration, 
                num_robots
            );
            

            //std::cout << "current_configuration: " << current_configuration[0][0] << ", " << current_configuration[0][1] << std::endl;
            //std::cout << "current_configuration: " << current_configuration[1][0] << ", " << current_configuration[1][1] << std::endl;
            //std::cout << "random_configuration: " << random_configuration[0][0] << ", " << random_configuration[0][1] << std::endl;
            //std::cout << "random_configuration: " << random_configuration[1][0] << ", " << random_configuration[1][1] << std::endl;

            bool found_way_1 = extend_tree(
                return_list,
                speed_for_freq,
                angle_and_time,
                processed_img,
                random_configuration,
                node_id,
                nearest_node_index,
                rrt_step_size,
                num_robots
            );

            bool found_way_2 = extend_tree(
                goal_return_list,
                speed_for_freq,
                goal_angle_and_time,
                processed_img,
                random_configuration,
                goal_node_id,
                goal_nearest_node_index,
                rrt_step_size,
                num_robots
            );

            if (found_way_1 && found_way_2){
                std::cout << "Found a way" << std::endl;
                break;
            }
           

        }
        
        // end the timer and print the time
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Elapsed time: " << elapsed.count() << " s" << std::endl;
        return std::make_tuple(return_list, goal_return_list);
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
    /*
    rrt(
        2, 
        robots_start_x, 
        robots_start_y, 
        robots_goal_x, 
        robots_goal_y, 
        rrt_step_size, 
        rrt_rewire_distance, 
        rrt_max_iter, 
        env_min_x, 
        env_min_y, 
        env_width, 
        env_height, 
        speed_for_freq, 
        img
    );
    */
    return 0;
}

