import networkx as nx

# 1. Read the problem file written by C++
g = nx.Graph()
with open('matching_problem.txt', 'r') as f:
    for line in f:
        u, v, weight = line.split()
        # Add a weighted edge to the matching graph
        g.add_edge(int(u), int(v), weight=float(weight))

# 2. Solve the matching (the magic line)
# This finds the set of pairs with the minimum total weight
matching = nx.min_weight_matching(g, weight='weight')

# 3. Write the solution back for C++
with open('matching_solution.txt', 'w') as f:
    for u, v in matching:
        f.write(f"{u} {v}\n")

# print("Python: Matching complete.") # Optional: for debugging