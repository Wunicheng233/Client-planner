import time
import math
from zss_debug_pb2 import Debug_Msgs, Debug_Msg

def smooth_path(path, iterations=2):
    if len(path) <= 2:
        return path
    for _ in range(iterations):
        new_path = [path[0]]
        for i in range(len(path) - 1):
            p0, p1 = path[i], path[i+1]
            q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
            r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
            new_path.extend([q, r])
        new_path.append(path[-1])
        path = new_path
    return path

class RobotController:
    def __init__(self, vision, action, debugger, planner):
        self.vision = vision
        self.action = action
        self.debugger = debugger
        self.planner = planner

        self.START_POS = (2500, 1800)
        self.GOAL_POS = (-2400, -1500)

        self.mission_state = "INIT"  
        self.current_target = self.START_POS
        self.trip_count = 0
        self.MAX_TRIPS = 5  
        self.start_time = 0
        
        # 评测记录
        self.collision_count = 0
        self.was_colliding = False
        self.last_collision_time = 0.0
        self.COLLISION_THRESHOLD = 200
        
        self.global_path = []
        self.target_index = 0
        self.prev_angle_error = 0.0

    def check_collisions(self, my_robot, obstacles):
        if self.mission_state == "RUNNING":
            is_colliding_now = False
            for ox, oy in obstacles:
                if math.hypot(my_robot.x - ox, my_robot.y - oy) < self.COLLISION_THRESHOLD:
                    is_colliding_now = True
                    break
            
            if is_colliding_now:
                current_time = time.time()
                if not self.was_colliding and (current_time - self.last_collision_time > 1.0):
                    self.collision_count += 1
                    self.last_collision_time = current_time
                    print(f"[警告] 发生碰撞！当前累计: {self.collision_count} 次")
                
                self.was_colliding = True
            else:
                self.was_colliding = False
    
    def get_dynamic_obstacles(self):
        """【新增】不仅获取当前坐标，还提取黄车的速度向量生成未来残影"""
        obstacles = []
        # 提取动态黄车及残影
        for robot in self.vision.yellow_robot:
            if robot.visible:
                obstacles.append((robot.x, robot.y))
                # 如果速度较快，额外增加 0.35秒 后的残影点
                if abs(robot.vel_x) > 50 or abs(robot.vel_y) > 50:
                    ghost_x = robot.x + robot.vel_x * 0.35
                    ghost_y = robot.y + robot.vel_y * 0.35
                    obstacles.append((ghost_x, ghost_y))
                    
        # 提取静态蓝车 (剔除自己)
        for i, robot in enumerate(self.vision.blue_robot):
            if i != 0 and robot.visible:
                obstacles.append((robot.x, robot.y))
        return obstacles
    
    def check_path_blocked(self, obstacles):
        # 极其保守的防撞重规划
        if self.global_path and self.target_index < len(self.global_path):
            path_blocked = False
            check_steps = min(10, len(self.global_path) - self.target_index)
            for i in range(self.target_index, self.target_index + check_steps):
                pt = self.global_path[i]
                for ox, oy in obstacles:
                    if math.hypot(pt[0] - ox, pt[1] - oy) < 500:
                        path_blocked = True
                        break
                if path_blocked:
                    break
            
            if path_blocked:
                self.global_path = []

    def plan_path(self, my_robot, obstacles):
        # A* 全局路径规划
        if not self.global_path:
            raw_path = self.planner.a_star_search((my_robot.x, my_robot.y), self.current_target, obstacles)
            if raw_path:
                raw_path[-1] = self.current_target 
                
                # 滑动均值滤波去毛刺
                window = 3  # 滤波窗口，数值越大折线越直
                filtered_path = []
                
                for i in range(len(raw_path)):
                    # 起点和终点绝对不能动，锁死目标
                    if i == 0 or i == len(raw_path) - 1:
                        filtered_path.append(raw_path[i])
                        continue
                    
                    # 截取当前点前后几个点，取平均坐标
                    start = max(0, i - window)
                    end = min(len(raw_path), i + window + 1)
                    avg_x = sum(p[0] for p in raw_path[start:end]) / (end - start)
                    avg_y = sum(p[1] for p in raw_path[start:end]) / (end - start)
                    filtered_path.append((avg_x, avg_y))

                # 把去完毛刺的干净路线，交给原有的函数做最后圆弧润色
                self.global_path = smooth_path(filtered_path, iterations=2)
                self.target_index = 0
            else:
                self.global_path = []
                self.action.sendCommand(vx=0, vy=0, vw=0)
                return False
        return True

    def calculate_pd_control(self, my_robot):
        # 局部运动控制 (PD 控制器)
        vx, vw = 0.0, 0.0
        if self.global_path:
            lookahead_distance = 250.0
            while self.target_index < len(self.global_path) - 1:
                pt = self.global_path[self.target_index]
                if math.hypot(pt[0] - my_robot.x, pt[1] - my_robot.y) < lookahead_distance:
                    self.target_index += 1
                else:
                    break
                    
            target_pt = self.global_path[self.target_index]
            dist_to_pt = math.hypot(target_pt[0] - my_robot.x, target_pt[1] - my_robot.y)
            
            target_angle = math.atan2(target_pt[1] - my_robot.y, target_pt[0] - my_robot.x)
            angle_error = target_angle - my_robot.orientation
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            
            Kp_w = 6.0
            Kd_w = 2.5 
            error_diff = angle_error - self.prev_angle_error
            vw = max(-10.0, min(10.0, Kp_w * angle_error + Kd_w * error_diff))
            self.prev_angle_error = angle_error 
            
            speed_factor = max(0.15, 1.0 - abs(angle_error) / (math.pi / 2.5) ** 2)
            base_speed = 5.0 * dist_to_pt
            vx = min(2500.0, base_speed * speed_factor)
            if vx < 400: 
                vx = 400 
        return vx, vw

    def update_state_machine(self, my_robot):
        # 状态机切换
        dist_to_final = math.hypot(self.current_target[0] - my_robot.x, self.current_target[1] - my_robot.y)
        if dist_to_final < 180: 
            if self.mission_state == "INIT":
                print("\n>>> 已就位，正式开始计分挑战！ <<<")
                self.mission_state = "RUNNING"
                self.current_target = self.GOAL_POS
                self.start_time = time.time()  
                self.global_path = []  
            elif self.mission_state == "RUNNING":
                if self.current_target == self.GOAL_POS:
                    self.current_target = self.START_POS
                    self.global_path = []
                elif self.current_target == self.START_POS:
                    self.trip_count += 1
                    current_time = time.time() - self.start_time
                    print(f" -> 完成第 {self.trip_count} 个来回，当前累计耗时: {current_time:.2f} 秒")
                    
                    if self.trip_count >= self.MAX_TRIPS:
                        print("\n" + "="*40)
                        print("🏁 验收挑战结束 🏁")
                        print(f"🕒 最终总耗时 (5个来回): {current_time:.2f} 秒")
                        print(f"💥 总碰撞次数: {self.collision_count} 次")
                        self.mission_state = "DONE"
                    else:
                        self.current_target = self.GOAL_POS
                        self.global_path = []

    def send_command_and_draw(self, vx, vw):
        # 发送指令与稳定绘图
        self.action.sendCommand(vx=vx, vy=0, vw=vw)
        
        package = Debug_Msgs()
        if self.global_path:
            x1, y1, x2, y2 = [], [], [], []
            for i in range(len(self.global_path) - 1):
                x1.append(self.global_path[i][0])
                y1.append(self.global_path[i][1])
                x2.append(self.global_path[i+1][0])
                y2.append(self.global_path[i+1][1])
            if x1:
                self.debugger.draw_lines(package, x1=x1, y1=y1, x2=x2, y2=y2)
                
        self.debugger.draw_circle(package, x=self.current_target[0], y=self.current_target[1])
        self.debugger.send(package) 

    def run(self):
        time.sleep(0.1)
        print(">>> A* + PD 追踪器就绪 <<<")

        while True:
            my_robot = self.vision.my_robot
            if my_robot.x == -999999:
                time.sleep(0.01)
                continue
                
            if self.mission_state == "DONE":
                self.action.sendCommand(vx=0, vy=0, vw=0)
                time.sleep(0.1)
                continue

            # 1. 获取障碍物
            obstacles = self.get_dynamic_obstacles()

            # 2. 碰撞检测
            self.check_collisions(my_robot, obstacles)

            # 3. 撕毁被阻挡的路线
            self.check_path_blocked(obstacles)

            # 4. A* 路径规划 (如果没算出来则直接进下一帧)
            if not self.plan_path(my_robot, obstacles):
                continue

            # 5. 计算 PD 局部控制速度
            vx, vw = self.calculate_pd_control(my_robot)

            # 6. 维护任务与状态机
            self.update_state_machine(my_robot)

            # 7. 下发指令与绘图
            self.send_command_and_draw(vx, vw)

            time.sleep(0.01)