# IoT-Sensor-Network_Dijkstra-Network
# Minimum Energy Consumption Path in IoT Sensor Network Using Dijkstra’s Algorithm With Priority Queue

## Overview
This project aims to find the minimum-energy data offloading path in an IoT sensor network using Dijkstra’s algorithm with a priority queue. The network is modeled as a graph where nodes represent sensor nodes, and edges represent the communication links between them. The objective is to ensure efficient data transfer with minimum energy consumption.

## Network Model
- The IoT sensor network is represented as a graph `G(V,E)`, where `V = {1, 2, ..., N}` is the set of `N` nodes, and `E` is the set of edges.
- Sensor nodes are randomly generated within an `x` by `y` area.
- Each node has the same transmission range `Tr`. Nodes within this range can communicate directly and are connected by an edge.
- `p` nodes are randomly selected as storage-depleted data generating nodes (data nodes, DNs), denoted as `DN = {DN1, DN2, ..., DNp}`.
- Each DN has `si` number of data packets, each 400 bytes (3200 bits).
- Other nodes are storage nodes (SNs), with each `SNi` having a storage capacity of `mi` data packets.

## Energy Model
- Sensor nodes are battery-powered with limited energy.
- Communication between nodes costs battery power.
- Transmission energy spent by node `i` sending `k`-bit data to node `j` over distance `l`:
  - `ET (k, l) = Eelec * k + Eamp * k * l^2`
- Receiving energy spent by node `j` receiving `k`-bit data:
  - `ER (k) = Eelec * k`
- Constants:
  - `Eamp = 100 pJ/bit/m^2`
  - `Eelec = 100 nJ/bit`
- Edge weight (cost) for communication between nodes `i` and `j`:
  - `Cost(i, j) = 2 * Eelec * k + Eamp * k * l^2`

## Objectives
- **Connectivity Check**: Verify if the sensor network graph is connected.
  - If not, print "the network is not connected" and prompt user input again.
- **Feasibility Check**: Ensure that the network has enough storage capacity for all data packets.
  - Condition: `p * q <= (N - p) * m`
  - If not, print "there is not enough storage in the network" and prompt user input again.
- **Data Offloading**: Once connectivity and feasibility are satisfied:
  - List the IDs of DNs and SNs.
  - Prompt the user to input a DN node and a storage node (SN).
  - Output the minimum-energy data offloading path, the energy cost for one data packet, and the total energy cost for all data packets.

## Usage
### Prerequisites
- Python 3.x
- Priority queue/heap library (e.g., `heapq`)

I have implemented a Minimum-Energy Data Offloading Path using Dijkstra’s algorithm with a priority queue and Prim’s algorithm .
### Test Case 1:
Small Network : a small network with few nodes and data to offload
 
![image](https://github.com/OletiKavya/IoT-Sensor-Network_Dijkstra-Network/assets/121835613/c37c4431-a8dd-46d5-9269-cc18d32ed37f)

### Test Case 2: 
Insufficient storage: a network where there is not enough storage to offload all the data

![image](https://github.com/OletiKavya/IoT-Sensor-Network_Dijkstra-Network/assets/121835613/c41e5769-3ca4-44b7-8d25-26c5ee8a3591)

### Test Case 3:
 Large network: a larger network with many nodes and data to offload

![image](https://github.com/OletiKavya/IoT-Sensor-Network_Dijkstra-Network/assets/121835613/8bd8155a-a392-4535-a842-81cfb957b391)

### Test Case 4:
Maximum storage: a network where there is just enough storage to offload all the data.

![image](https://github.com/OletiKavya/IoT-Sensor-Network_Dijkstra-Network/assets/121835613/6b25df52-700f-4f6b-aca9-eec88014fe97)

 
