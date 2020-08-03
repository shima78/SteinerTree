def getWeight(edge):
  return edge[2]


def getUsedEdges(nodes, g):
  Usededges = []
  for i in range(len(nodes) - 1):
    for adj in g.adj[nodes[i]]:
      if adj[0] == nodes[i + 1]:
        Usededges.append((nodes[i], nodes[i + 1], adj[1]))
        Usededges.append((nodes[i + 1], nodes[i], adj[1]))
  return Usededges


class Graph:
  def __init__(self, nodes, edges, terminals):
    self.edges = edges  # edges as a tuple (src,destination,wight)
    self.nodes = nodes  # number of nodes
    self.terminals = terminals  # nodes who are terminal
    self.MSTCost = 0
    self.steinerCost = 0
    self.adj = []
    for i in range(self.nodes + 1):
      self.adj.append([])

  def addEdge(self, u, v, w):
    self.edges.append((u, v, w))
    if v not in self.adj[u]:
      self.adj[u].append((v, w))
    if u not in self.adj[v]:
      self.adj[v].append((u, w))

  def addTerminal(self, node):
    self.terminals.append(node)

  def find(self, parent, node):
    if parent[node] == node:
      return parent[node]
    return self.find(parent, parent[node])

  def unionBySize(self, parent, size, parent1, parent2):
    root1 = self.find(parent, parent1)
    root2 = self.find(parent, parent2)
    if root1 == root2:
      return
    if size[root1] < size[root2]:
      root1, root2 = root2, root1  # swap the roots
    parent[root2] = root1
    size[root1] += size[root2]

  def unionByRank(self, parent, rank, parent1, parent2):
    root1 = self.find(parent, parent1)
    root2 = self.find(parent, parent2)
    # Attach smaller rank tree under root of
    # high rank tree (Union by Rank)
    if rank[root1] < rank[root2]:
      parent[root1] = root2
    elif rank[root1] > rank[root2]:
      parent[root2] = root1
      # If ranks are same, then make one as root
    # and increment its rank by one
    else:
      parent[root2] = root1
      rank[root1] += 1

  def findMST(self):  # find minimum spanning tree using kruskal's algorithm
    mst = []
    parent = []
    rank = []
    size = []
    presentedEdgesNum = 0
    i = 0
    numberOfNodes = 0
    # sort edges by their weight decreasing order using sort method in python lists
    self.edges.sort(key=getWeight)

    for node in range(self.nodes + 1):
      # making disjoint set for each node parent is the node itself and size = 1 and rank = 0
      parent.append(node)  # parent[node] = node
      rank.append(0)  # rank[node] = 0
      size.append(1)  # size[node] = 1

    while presentedEdgesNum < (self.nodes - 1):
      edge = self.edges[i]
      parent1 = self.find(parent, int(edge[0]))
      parent2 = self.find(parent, int(edge[1]))
      if parent1 != parent2:
        mst.append(edge)
        numberOfNodes += 1
        self.MSTCost += edge[2]
        self.unionBySize(parent, size, parent1, parent2)
        presentedEdgesNum += 1
      i += 1

    # create mst graph
    mstGraph = Graph(self.nodes, [], [])
    for u, v, weight in mst:
      mstGraph.addEdge(u, v, weight)

    for t in self.terminals:
      mstGraph.addTerminal(t)
    print("mst cost : ", self.MSTCost)

    return mstGraph

  def BFS_findAllPath(self, start, graph):
    visited = [False] * (graph.nodes + 1)
    shortestPath = []
    queue = [[start]]
    numSpanned = 0
    numTerminals = len(self.terminals)
    spanned = [False] * (graph.nodes + 1)
    checkTerminal = [False] * (graph.nodes + 1)
    for t in self.terminals:
      checkTerminal[t] = True
    # Loop to traverse the graph
    # with the help of the queue
    while queue and numSpanned != numTerminals:
      path = queue.pop(0)
      node = path[-1]
      # Condition to check
      if not visited[node]:
        neighbours = graph.adj[node]
        # Loop to iterate over the
        # neighbours of the node
        for neighbour in neighbours:
          new_path = list(path)
          new_path.append(neighbour[0])
          queue.append(new_path)

          # Condition to check
          if checkTerminal[neighbour[0]] and neighbour[0] != start and not spanned[
            neighbour[0]] and numSpanned != numTerminals:
            shortestPath.append((neighbour[0], new_path))
            spanned[neighbour[0]] = True
            numSpanned += 1

        visited[node] = True
    return shortestPath

  def findSteinerTRee(self):
    mst = self.findMST()
    useLessEdges = []

    # assign all edges to useLess
    for edge in mst.edges:
      useLessEdges.append(edge)

    # start point for BFS
    t = self.terminals[0]

    # get used nodes to reach terminals
    usedNodes = self.BFS_findAllPath(t, mst)

    for path in usedNodes:
      usedEdges = getUsedEdges(path[1], mst)
      # find useLess edges
      for edg in usedEdges:
        try:
          useLessEdges.remove(edg)
        except:
          pass
          # print(" ")
    edges = mst.edges

    # remove UseLess edges
    for i in range(len(useLessEdges)):
      edges.remove(useLessEdges[i])
    # create Steiner tree
    steinerTree = Graph(self.nodes, [], [])
    self.steinerCost = 0
    for i in range(len(edges)):
      steinerTree.addEdge(edges[i][0], edges[i][1], edges[i][2])
      self.steinerCost += edges[i][2]

    return steinerTree


def createGraph(name):
  f = open(name + ".stp", "r")

  out = open(name + ".out", "w")
  for i in range(8):
    f.readline()

  nodes = int(f.readline().split(" ")[1])
  edgesNum = int(f.readline().split(" ")[1])

  g = Graph(nodes, [], [])
  for i in range(edgesNum):
    edge = f.readline().split(" ")
    first = int(edge[1])
    second = int(edge[2])
    weight = int(edge[3])
    g.addEdge(first, second, weight)

  f.readline()
  f.readline()
  f.readline()

  numTerminal = int(f.readline().split(" ")[1])
  for i in range(numTerminal):
    term = f.readline().split(" ")
    node = int(term[1])
    g.addTerminal(node)

  steinrerTree = g.findSteinerTRee()
  out.write("Cost ")
  out.write(str(g.steinerCost) + "\n")
  out.write("Edges " + str(len(steinrerTree.edges)) + "\n")
  for e in steinrerTree.edges:
    out.write("E " + str(e[0]) + " " + str(e[1]) + "\n")
  f.close()
  out.close()
  print("steiner cost : ", g.steinerCost)


createGraph("cc3-5p")
