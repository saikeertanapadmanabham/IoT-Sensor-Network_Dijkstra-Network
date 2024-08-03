import heapq
import math

# define the energy model constants
Eelec = 100 * 10 ** -9  # energy consumption of one bit on transmitter circuit and receiver circuit
Eamp = 100 * 10 ** -12  # energy consumption of one bit on transmit amplifier

# define the graph class
class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.adj = [[] for i in range(vertices)]
        self.nodes = []
        self.storageNodes = []
    
    def add_edge(self, u, v, dist):
        if v not in self.adj:
            self.add_vertex(v)  # add v as a new vertex if not already in the graph
        if u not in self.adj:
            self.add_vertex(u)  # add u as a new vertex if not already in the graph
        self.adj[u].append((v, dist))
        self.adj[v].append((u, dist))

    def add_vertex(self, v):
        self.adj.append([])
        self.V += 1

    # Dijkstra's algorithm with priority queue
    def dijkstra(self, src, dst, q):
        pq = [(0, src)]  # priority queue for storing vertices with their distances from source
        dist = [math.inf] * self.V
        dist[src] = 0

        while pq:
            (d, u) = heapq.heappop(pq)
            if u == dst:
                break
            if d > dist[u]:
                continue
            for v, w in self.adj[u]:
                alt = d + self.energy_cost(w, q)
                if alt < dist[v]:
                    dist[v] = alt
                    heapq.heappush(pq, (alt, v))
        
        return dist[dst]
        # Prim's algorithm for minimum spanning tree
    def prims_algorithm(self, src):
        mst = [False] * self.V  # list to keep track of vertices in MST
        key = [math.inf] * self.V  # list to keep track of key values
        parent = [None] * self.V  # list to keep track of parent nodes in MST

        # set key value of source node to 0
        key[src] = 0

        # heap to store key-value pairs
        pq = [(0, src)]

        while pq:
            u = heapq.heappop(pq)[1]

            # add u to MST
            mst[u] = True

            # update key values and heap
            for v, w in self.adj[u]:
                if not mst[v] and w < key[v]:
                    key[v] = w
                    parent[v] = u
                    heapq.heappush(pq, (w, v))

        # construct MST
        mst_weight = 0
        for i in range(self.V):
            if parent[i] is not None:
                mst_weight += key[i]
                self.adj[i].append((parent[i], key[i]))
                self.adj[parent[i]].append((i, key[i]))

        return mst_weight
    # energy cost of offloading one data packet from a DN to a SN along an edge
    def energy_cost(self, w, q):
        return 2 * Eelec * q + Eamp * q * w ** 2

# read input from user
while True:
    try:
        x, y = map(int, input("Enter width and length of the sensor network (in meters): ").split())
        N = int(input("Enter number of sensor nodes: "))
        Tr = int(input("Enter transmission range (in meters): "))
        p = int(input("Enter number of data nodes: "))
        q = int(input("Enter number of data packets each data node has: "))
        m = int(input("Enter storage capacity of each storage node: "))
        choice = int(input("Enter 1 for shortest path or 2 for minimum spanning tree: "))
    except ValueError:
        print("Invalid input, please try again.")
        continue

    # check connectivity of the sensor network graph

    V = N
    g = Graph(V)
    nodes = [(i, j) for i in range(x) for j in range(y)]
    edges = []
    j=0
    for i in range(N):
        u = i
        while True:
            v = u
            while v == u:
                v = nodes.index((int(x * i / N), int(y * j / N)))
                j = (j + 1) % N
            dist = math.dist(nodes[u], nodes[v])
            if dist <= Tr:
                edges.append((u, v, dist))
                try:
                    g.add_edge(u, v, dist)
                except IndexError:
                    continue
            else:
                break
            u = v
    
    
    

    # check feasibility of data offloading
    
    if len(edges) < N - 1:
        print("The network is not connected, please try again.")
        continue
        
    if p * q <= (N - p) * m:
        print("There is not enough storage in the network, please try again.")
        continue

    # print DN and SN IDs
    DN = list(range(1, p + 1))
    print("DN IDs: ", end="")
    for i in range(p):
        print("DN" + str(i+1) + "(" + str(DN[i]) + ")", end=" ")
    print("")
    print("SN IDs: ", end="")
    for i in range(N):
        if i+1 not in DN:
            print("SN" + str(i+1) + "(" + str(i+1) + ")", end=" ")
    print("")
    if choice == 1:
        # shortest path
        src, dst = map(int, input("Enter source and destination nodes: ").split())
        dist = g.dijkstra(src, dst, q)
        print("The shortest distance between nodes", src, "and", dst, "is", dist, "meters.")
    elif choice == 2:
        # minimum spanning tree
        src = 0
        mst_weight = g.prims_algorithm(src)
        print("The total weight of the minimum spanning tree is", mst_weight, "meters.")
    else:
        print("Invalid choice, please try again.")
        continue
    # prompt user to input DN and SN IDs
    while True:
        try:
            DN_ID, SN_ID = input("Please input the IDs of a DN and a SN: ").split()
            DN_ID = int(DN_ID)
            SN_ID = int(SN_ID)
            if DN_ID not in DN or SN_ID in DN:
                raise ValueError
            break
        except ValueError:
            print("Invalid input. Please input two IDs (one DN and one SN) separated by a space.")

    # find minimum-energy data offloading path using Dijkstra's algorithm
    dist = [float("inf")] * N
    dist[DN_ID-1] = 0
    pq = [(0, DN_ID)]
    while pq:
        curr_dist, curr_node = heapq.heappop(pq)
        if curr_dist > dist[curr_node-1]:
            continue
        for neighbor, weight in g.adj[curr_node]:
            alt = curr_dist + weight
            if neighbor<=N:
                if alt < dist[neighbor-1]:
                    dist[neighbor-1] = alt
                    heapq.heappush(pq, (alt, neighbor))

    # calculate energy cost of offloading one data packet and total energy cost
    data_size=3200
    energy_per_packet = 2 * Eelec * data_size + Eamp * data_size * dist[SN_ID-1]**2
    total_energy_cost = energy_per_packet * q

    # output results
    print("Minimum-energy data offloading path: ", end="")
    path = [SN_ID]
    curr_node = SN_ID
    while curr_node != DN_ID:
        for neighbor, weight in g.adj[curr_node]:
            if dist[curr_node-1] == dist[neighbor-1] + weight:
                path.append(neighbor)
                curr_node = neighbor
                break
    print(" -> ".join("SN"+str(node) for node in path[::-1]))
    print("Energy cost of offloading one data packet: ", energy_per_packet, " Joule")
    print("Total energy cost of offloading all data packets: ", total_energy_cost, " Joule")

