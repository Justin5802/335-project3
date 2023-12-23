/*
CSCI335 Fall 2023
Assignment 3 - Traveling Salesman
Name: Justin Chu
Date: 12/22/23
NearestNeighbor.hpp
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>   
#include <limits>   
#include <cmath>    

// Node class represents a point in multidimensional space with an identifier and coordinates.
class Node {
public:
    int id;                         // Identifier for the node
    std::vector<double> position;   // Coordinates of the node in multidimensional space

    // Constructor for Node class, initializing id and coordinates.
    Node(int id, std::vector<double> coords) : id(id), position(coords) {}

    // Calculate the Euclidean distance between two nodes in multidimensional space.
    double calcDistance(Node& other);
};

// Nearest Neighbor Algorithm to solve the Traveling Salesman Problem.
void nearestNeighbor(const std::string filename);

// Implementation of Node class method to calculate the Euclidean distance.
double Node::calcDistance(Node& other) {
    double squaredDist = 0.0;
    for (size_t i = 0; i < position.size(); ++i) {
        double diff = position[i] - other.position[i];
        squaredDist += diff * diff;
    }
    return round(std::sqrt(squaredDist));
}

// Main function implementing the Nearest Neighbor Algorithm for solving the TSP.
void nearestNeighbor(const std::string filename) {
    // Open the input file for reading node information.
    std::ifstream file(filename);
    if (!file.is_open()) {
        // Display an error message if the file cannot be opened.
        std::cerr << "Error: Unable to open file.\n";
        return;
    }

    // Vector to store Node objects representing points in space.
    std::vector<Node> nodes;
    std::string line;
    int id;
    std::vector<double> coords;

    // Parse the input file to extract node information.
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (!(iss >> id)) {
            continue;
        }
        coords.clear();
        double coord;
        while (iss >> coord) {
            coords.push_back(coord);
        }
        nodes.emplace_back(id, coords);
    }

    if (nodes.empty()) {
        // Display a message if there are no nodes to process.
        std::cout << "No nodes to process.\n";
        return;
    }

    // Vector to keep track of visited nodes during the algorithm.
    std::vector<bool> visited(nodes.size(), false);
    // Vector to store the final path of node identifiers.
    std::vector<int> path;
    // Variable to track the total distance of the TSP path.
    double totalDistance = 0.0;

    // Record the start time for measuring execution time.
    auto startTime = std::chrono::steady_clock::now();

    // Initialize the algorithm with the first node as the starting point.
    int current = 0;
    path.push_back(nodes[current].id);
    visited[current] = true;

    // Apply the Nearest Neighbor algorithm to construct a path through all nodes.
    while (path.size() < nodes.size()) {
        double nearestDistance = std::numeric_limits<double>::max();
        int nearestNode = -1;

        // Find the nearest unvisited node to the current node.
        for (size_t j = 0; j < nodes.size(); ++j) {
            if (!visited[j]) {
                double distance = nodes[current].calcDistance(nodes[j]);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestNode = j;
                }
            }
        }

        if (nearestNode == -1) {
            // Display an error message if the nearest node cannot be found.
            std::cerr << "Error: Unable to find nearest node.\n";
            return;
        }

        // Mark the nearest node as visited and update the path and total distance.
        visited[nearestNode] = true;
        path.push_back(nodes[nearestNode].id);
        totalDistance += nearestDistance;
        current = nearestNode;
    }

    // Complete the TSP path by returning to the starting node.
    totalDistance += nodes[current].calcDistance(nodes[0]);
    path.push_back(nodes[0].id);

    // Record the end time for measuring execution time.
    auto endTime = std::chrono::steady_clock::now();

    // Display the TSP path, total distance, and execution time.
    for (const auto& nodeId : path) {
        std::cout << nodeId << " ";
    }
    std::cout << "\nTotal Distance: " << totalDistance << "\n";
    std::cout << "Execution Time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count()
              << " milliseconds\n";
}
