from vision import Vision
from action import Action
from debug import Debugger
from a_star_planner import Planner
from utils import RobotController

if __name__ == '__main__':
    # 1. 实例化各个组件模块
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = Planner()

    # 2. 注入组件
    controller = RobotController(vision, action, debugger, planner)
    controller.run()