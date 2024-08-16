import sys
import heapq

# Matriz de adjacência para o novo grafo
graph = [
    [0, 2, 7, sys.maxsize, sys.maxsize],  # Distâncias a partir do vértice 0
    [sys.maxsize, 0, 2, 8, 5],              # Distâncias a partir do vértice 1
    [sys.maxsize, 3, 0, 1, sys.maxsize],    # Distâncias a partir do vértice 2
    # Distâncias a partir do vértice 3
    [sys.maxsize, sys.maxsize, sys.maxsize, 0, 4],
    # Distâncias a partir do vértice 4
    [sys.maxsize, sys.maxsize, sys.maxsize, 4, 0]
]

# Algoritmo de Dijkstra


def dijkstra(graph, start):
    num_vertices = len(graph)
    dist = [sys.maxsize] * num_vertices
    dist[start] = 0
    priority_queue = [(0, start)]

    while priority_queue:
        current_dist, u = heapq.heappop(priority_queue)

        if current_dist > dist[u]:
            continue

        for v, weight in enumerate(graph[u]):
            if weight < sys.maxsize:
                distance = current_dist + weight
                if distance < dist[v]:
                    dist[v] = distance
                    heapq.heappush(priority_queue, (distance, v))

    return dist

# Algoritmo de Bellman-Ford


def bellman_ford(graph, start):
    num_vertices = len(graph)
    dist = [sys.maxsize] * num_vertices
    dist[start] = 0

    for _ in range(num_vertices - 1):
        for u in range(num_vertices):
            for v, weight in enumerate(graph[u]):
                if weight < sys.maxsize and dist[u] + weight < dist[v]:
                    dist[v] = dist[u] + weight

    # Verificação de ciclo negativo
    for u in range(num_vertices):
        for v, weight in enumerate(graph[u]):
            if weight < sys.maxsize and dist[u] + weight < dist[v]:
                raise ValueError("O grafo contém um ciclo negativo")

    return dist

# Algoritmo de Floyd-Warshall


def floyd_warshall(graph):
    num_vertices = len(graph)
    dist = [row[:] for row in graph]

    for k in range(num_vertices):
        for i in range(num_vertices):
            for j in range(num_vertices):
                if dist[i][k] < sys.maxsize and dist[k][j] < sys.maxsize:
                    dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])

    return dist

# Função para exibir a matriz de distâncias


def print_matrix(matrix):
    for row in matrix:
        print(
            ' '.join(f"{dist if dist < sys.maxsize else 'Inf':>4}" for dist in row))


# Exemplo de uso
start_vertex = 0

print("Dijkstra:")
print(dijkstra(graph, start_vertex))

print("\nBellman-Ford:")
try:
    print(bellman_ford(graph, start_vertex))
except ValueError as e:
    print(e)

print("\nFloyd-Warshall:")
print_matrix(floyd_warshall(graph))
