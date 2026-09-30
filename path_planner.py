import numpy as np
import heapq
import matplotlib.pyplot as plt

class UAVPathPlanner:
    def __init__(self, grid_size=(50, 50), radar_threats=None):
        self.grid_size = grid_size
        self.cost_map = np.ones(grid_size)  # Temel hareket maliyeti (Düz uçuş = 1.0)
        
        # Tehdit bölgelerini (Radar kapsama alanları) haritaya Gauss maliyet fonksiyonuyla ekle
        if radar_threats:
            for rx, ry, radius, intensity in radar_threats:
                self._add_radar_threat(rx, ry, radius, intensity)

    def _add_radar_threat(self, rx, ry, radius, intensity):
        """Radardan uzaklaştıkça azalan tehdit/risk fonksiyonu"""
        for x in range(self.grid_size[0]):
            for y in range(self.grid_size[1]):
                dist = np.hypot(x - rx, y - ry)
                if dist <= radius:
                    # Merkezde en yüksek, sınıra doğru azalan ceza maliyeti
                    self.cost_map[x, y] += intensity * np.exp(- (dist**2) / (2 * (radius / 2)**2))

    def heuristic(self, a, b):
        """Öklid uzaklığı sezgiseli (Euclidean Heuristic)"""
        return np.hypot(a[0] - b[0], a[1] - b[1])

    def plan_path(self, start, goal):
        """A* Algoritması ile minimum maliyetli rota hesabı"""
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 8 Yönlü hareket (çapraz ve düz uçuşlar)
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Rotayı geriye doğru oluştur
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Sınır kontrolü
                if 0 <= neighbor[0] < self.grid_size[0] and 0 <= neighbor[1] < self.grid_size[1]:
                    # Çapraz hareket maliyeti sqrt(2), düz hareket 1.0
                    move_cost = np.hypot(dx, dy) * self.cost_map[neighbor]
                    tentative_g_score = g_score[current] + move_cost

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

if __name__ == "__main__":
    # Radarlar: (x, y, yarıçap, tehdit şiddeti)
    radars = [
        (25, 25, 12, 100.0),
        (15, 35, 8, 80.0),
        (35, 15, 10, 90.0)
    ]
    
    planner = UAVPathPlanner(grid_size=(50, 50), radar_threats=radars)
    start_pos = (5, 5)
    goal_pos = (45, 45)
    
    path = planner.plan_path(start_pos, goal_pos)
    
    if path:
        print(f"Optimal Rota Bulundu! Rota uzunluğu: {len(path)} adım")
    else:
        print("Geçerli bir rota bulunamadı!")
