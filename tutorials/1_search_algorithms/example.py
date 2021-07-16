import os
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt

try:
    mpl.use('Qt5Agg')
except ImportError:
    mpl.use('TkAgg')

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object

# add current directory to python path for local imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from motion_planner.motion_planner import MotionPlanner
from motion_planner.plot_config import StudentScriptPlotConfig


def main():
    # configurations
    path_scenario = '../../scenarios/tutorial/ZAM_Tutorial_Urban-3_2.xml'
    id_type_vehicle = 2
    file_motion_primitives = 'V_9.0_9.0_Vstep_0_SA_-0.2_0.2_SAstep_0.4_T_0.5_Model_BMW320i.xml'
    config_plot = StudentScriptPlotConfig(DO_PLOT=True)

    # load scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
    # retrieve the first planning problem
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # plot scenario
    plt.figure(figsize=(8, 8))
    draw_object(scenario)
    draw_object(planning_problem_set)
    plt.gca().set_aspect('equal')
    plt.margins(0, 0)
    plt.show()
    # close the figure to continue!

    # create maneuver automaton and planning problem
    automaton = ManeuverAutomaton.generate_automaton(file_motion_primitives)

    # comment out the planners which you don't want to execute
    dict_motion_planners = {
        0: (MotionPlanner.BreadthFirstSearch, "Breadth First Search"),
        # 1: (MotionPlanner.DepthFirstSearch, "Depth First Search"),
        # 2: (MotionPlanner.DepthLimitedSearch, "Depth Limited Search"),
        # 3: (MotionPlanner.UniformCostSearch, "Uniform Cost Search"),
        # 4: (MotionPlanner.GreedyBestFirstSearch, "Greedy Best First Search"),
        5: (MotionPlanner.AStarSearch, "A* Search")

        # 6: (MotionPlanner.StudentMotionPlanner, "Student Planner"),
        # 7: (MotionPlanner.StudentMotionPlannerExample, "Student Planner Example")
    }

    for (class_planner, name_planner) in dict_motion_planners.values():
        planner = class_planner(scenario=scenario, planning_problem=planning_problem,
                                automaton=automaton, plot_config=config_plot)

        # start search
        print(name_planner + " started..")
        planner.execute_search()
        plt.title(name_planner)
        # close the figure to continue
        plt.show()


print('Done')

if __name__ == '__main__':
    main()
