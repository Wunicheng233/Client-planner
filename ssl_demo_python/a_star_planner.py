import math
import heapq

class Planner:
    def __init__(self, grid_size=100, robot_radius=250):
        """
        初始化规划器
        :param grid_size: 栅格大小 (mm)
        :param robot_radius: 障碍物膨胀半径 (mm)
        """
        self.GRID_SIZE = grid_size
        self.ROBOT_RADIUS = robot_radius

    def world_to_grid(self, x, y):
        return (int(x // self.GRID_SIZE), int(y // self.GRID_SIZE))

    def grid_to_world(self, gx, gy):
        return (gx * self.GRID_SIZE + self.GRID_SIZE // 2, gy * self.GRID_SIZE + self.GRID_SIZE // 2)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_obstacles(self, vision):
        """从视觉数据中提取障碍物坐标"""
        obstacles = []
        for robot in vision.yellow_robot:
            if robot.visible:
                obstacles.append((robot.x, robot.y))
        for i, robot in enumerate(vision.blue_robot):
            if i != 0 and robot.visible:
                obstacles.append((robot.x, robot.y))
        return obstacles

    def is_collision(self, gx, gy, obstacles):
        wx, wy = self.grid_to_world(gx, gy)
        for ox, oy in obstacles:
            if math.hypot(wx - ox, wy - oy) < self.ROBOT_RADIUS:
                return True
        return False

    def a_star_search(self, start_w, goal_w, obstacles):
        """A* 搜索主函数"""
        start_g = self.world_to_grid(*start_w)
        goal_g = self.world_to_grid(*goal_w)
        
        frontier = []
        heapq.heappush(frontier, (0, start_g))
        came_from = {}
        cost_so_far = {}
        came_from[start_g] = None
        cost_so_far[start_g] = 0
        
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        while frontier:
            _, current = heapq.heappop(frontier)
            
            if current == goal_g:
                break
                
            for dx, dy in neighbors:
                next_g = (current[0] + dx, current[1] + dy)
                # 场地边界限制
                if abs(next_g[0] * self.GRID_SIZE) > 5000 or abs(next_g[1] * self.GRID_SIZE) > 4000:
                    continue
                    
                if self.is_collision(next_g[0], next_g[1], obstacles):
                    continue
                    
                move_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                new_cost = cost_so_far[current] + move_cost
                
                if next_g not in cost_so_far or new_cost < cost_so_far[next_g]:
                    cost_so_far[next_g] = new_cost
                    priority = new_cost + self.heuristic(next_g, goal_g)
                    heapq.heappush(frontier, (priority, next_g))
                    came_from[next_g] = current
                    
        if goal_g not in came_from:
            return [] 
            
        path_g = []
        curr = goal_g
        while curr != start_g:
            path_g.append(curr)
            curr = came_from[curr]
        path_g.reverse()
        
        return [self.grid_to_world(x, y) for x, y in path_g]