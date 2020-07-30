def getWeight(edge):
  return edge[2]


class Graph:
  def __init__(self, nodes, edges, terminals):
    self.edges = edges  # edges as a tuple (src,destination,wight)
    self.nodes = nodes  # number of nodes
    self.terminals = terminals  # nodes who are terminal
    self.MSTCost = 0
    self.steinerCost = 0

  def addEdge(self, u, v, w):
    self.edges.append((u, v, w))

  def find(self, parent, node):
    if parent[node] == node:
      return parent[node]
    return self.find(parent, parent[node])

  def unionBySize(self, paret1, parent2, parent, size):
    root1 = self.find(parent, paret1)
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

    # sort edges by their weight decreasing order using sort method in python lists
    self.edges.sort(key=getWeight)
    print(self.edges)
    for node in range(self.nodes):
      # making disjoint set for each node parent is the node inteslf and size = 1 and rank = 0

      parent.append(node)  # parent[node] = node
      rank.append(0)  # rank[node] = 0
      size.append(1)  # size[node] = 1
    print(parent,size,rank)

    while presentedEdgesNum < (self.nodes - 1):
      edge = self.edges[i]
      parent1 = self.find(parent, edge[0])
      parent2 = self.find(parent, edge[1])
      if parent1 != parent2:
        mst.append(edge)
        self.MSTCost += edge[2]
        self.unionByRank(parent,rank,parent1,parent2)
        presentedEdgesNum += 1
      i += 1

    print("Following are the edges in the constructed MST")
    for u, v, weight in mst:
      # print str(u) + " -- " + str(v) + " == " + str(weight)
      print("%d -- %d == %d" % (u, v, weight))
    print(self.MSTCost)


g = Graph(4, [], 1)
g.addEdge(0, 1, 10)
g.addEdge(0, 2, 6)
g.addEdge(0, 3, 5)
g.addEdge(1, 3, 15)
g.addEdge(2, 3, 4)

g.findMST()
