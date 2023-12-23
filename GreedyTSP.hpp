/*
CSCI335 Fall 2023
Assignment 3 - Traveling Salesman
Name: Justin Chu
Date: 12/22/23
GreedyTSP.hpp
*/

#ifndef GREEDY_TSP_HPP
#define GREEDY_TSP_HPP
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <list>
#include <cmath>
#include <iterator>
#include <chrono>

class Edge 
{
public:
    int from;
    int to;
    double weight;

    Edge(int f, int t, double w) : from(f), to(t), weight(w) {}

    // Comparison operator for sorting edges
    bool operator<(const Edge& other) const 
    {
        return weight < other.weight;
    }
};

class NODE 
{
public:
    int id;
    double latitude;
    double longitude;
    NODE* edge1;  // First edge
    NODE* edge2;  // Second edge

    NODE() : id(0), latitude(0.0), longitude(0.0), edge1(nullptr), edge2(nullptr) {}
    
    NODE(int i, double lat, double lon) : id(i), latitude(lat), longitude(lon), edge1(nullptr), edge2(nullptr) {}
    
    double distance(const NODE& other) const 
    {
        double latDiff = latitude - other.latitude;
        double lonDiff = longitude - other.longitude;
        return std::sqrt(latDiff * latDiff + lonDiff * lonDiff);
    }
};

void greedyTSP (std::string filename)
{
    std::ifstream file(filename);

    int numNodes;
    
    // Skip irrelevant lines
    for (int i = 0; ; i++) 
    {
        std::string line;
        std::getline(file, line);
        if (line.find("DIMENSION") == 0)
        {
            int colon = line.find(":");
            std::string tempNum = line.substr(colon + 1);
            numNodes = std::stoi(tempNum);
        }
        if (line.find("NODE_COORD_SECTION") == 0)
        {
            break;
        }
    }

    std::vector<NODE> nodes(numNodes + 1);  // Use direct addressing
    int maxEdges = numNodes;
    std::vector<int> edgesPerNode(numNodes + 1);

    // Read node information
    for (int i = 1; i <= numNodes; ++i) 
    {
        file >> nodes[i].id >> nodes[i].latitude >> nodes[i].longitude;
    }

    file.close();
    
    auto start_time = std::chrono::high_resolution_clock::now();

    std::list<Edge> edges;

    // Calculate and store all edges
    for (int i = 1; i <= numNodes; ++i) 
    {
        for (int j = i + 1; j <= numNodes; ++j) 
        {
            double weight = nodes[i].distance(nodes[j]);
            edges.emplace_back(i, j, weight);
        }
    }

    // Sort edges by weight
    edges.sort();

    // Initialize tour with the first edge
    nodes[edges.front().from].edge1 = &nodes[edges.front().to];
    nodes[edges.front().to].edge2 = &nodes[edges.front().from];

    double totalDistance = edges.front().weight;

    // Print the first edge
    std::cout << "Edge from " << edges.front().from << " to " << edges.front().to << " of weight " << edges.front().weight << std::endl;
    edgesPerNode[edges.front().from]++;
    edgesPerNode[edges.front().to]++;

    // Add the remaining edges
    for (auto it = std::next(edges.begin()); it != edges.end(); ++it)
    {
        int from = it->from;
        int to = it->to;
        bool goodEdge = true;

        // Check if adding the edge would create a cycle
        if (edgesPerNode[from] >= 2 || edgesPerNode[to] >= 2) 
        {
            continue;
        }
        else
        {
            // Check for cycles
            int current = to;
            while (nodes[current].edge1) 
            {
                int next = nodes[current].edge1->id;

                if (next == from)
                {
                    // Adding this edge would create a cycle, so skip it
                    break;
                }

                current = next;
            }
        }

        // Update the tour with the new edge
        maxEdges--;
        if (maxEdges > 0 && goodEdge)
        {
            if (!nodes[from].edge1) 
            {
                nodes[from].edge1 = &nodes[to];
                nodes[to].edge2 = &nodes[from];
            } 
            else 
            {
                nodes[from].edge2 = &nodes[to];
                nodes[to].edge1 = &nodes[from];
            }

            totalDistance += it->weight;

            // Print the added edge
            edgesPerNode[from]++;
            edgesPerNode[to]++;
            std::cout << "Edge from " << from << " to " << to << " of weight " << it->weight << std::endl;
        }
    }
    
    // Stop the clock
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Print the results
    std::cout << "Total Distance: " << round(totalDistance) << std::endl;
    
    // Print the order of visited nodes
    int current = 1;
    std::cout << current << " ";
    current = nodes[current].edge1->id;
    while (nodes[current].id != 1) 
    {
        std::cout << current << " ";
        current = nodes[current].edge1->id;
    }
    std::cout << current << std::endl;
    
    // Print the time taken
    std::cout << "Time in ms: " << duration.count() << std::endl;
}

#endif
