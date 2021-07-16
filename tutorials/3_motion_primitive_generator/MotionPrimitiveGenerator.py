import os
import copy
import random
import warnings

warnings.filterwarnings("ignore")

from tqdm.notebook import tqdm
import itertools
import numpy as np
import matplotlib.pyplot as plt

import yaml
import xml.etree.ElementTree as et
from xml.dom import minidom

from commonroad.scenario.trajectory import State, Trajectory
from commonroad.common.solution import VehicleType
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics
import commonroad_dc.feasibility.feasibility_checker as feasibility_checker

from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3


class MotionPrimitiveGenerator:
    """
    Class for motion primitive generator.
    """

    class Parameter:
        """
        Class to hold all necessary parameters
        """

        def __init__(self, configuration):
            # output related
            self.dir_output = configuration['outputs']['output_directory']

            # vehicle related
            self.id_type_vehicle = configuration['vehicles']['vehicle_type_id']

            assert self.id_type_vehicle in [1, 2, 3], "Wrong vehicle type id!"

            if self.id_type_vehicle == 1:
                self.type_vehicle = VehicleType.FORD_ESCORT
                self.param_vehicle = parameters_vehicle1()
                self.name_vehicle = "FORD_ESCORT"
                self.vehicle_dynamics = VehicleDynamics.KS(VehicleType.FORD_ESCORT)

            elif self.id_type_vehicle == 2:
                self.type_vehicle = VehicleType.BMW_320i
                self.param_vehicle = parameters_vehicle2()
                self.name_vehicle = "BMW_320i"
                self.vehicle_dynamics = VehicleDynamics.KS(VehicleType.BMW_320i)

            elif self.id_type_vehicle == 3:
                self.type_vehicle = VehicleType.VW_VANAGON
                self.param_vehicle = parameters_vehicle3()
                self.name_vehicle = "VW_VANAGON"
                self.vehicle_dynamics = VehicleDynamics.KS(VehicleType.VW_VANAGON)

            # time related
            self.duration = configuration['primitives']['duration']
            self.dt = configuration['primitives']['dt']
            self.dt_sim = configuration['primitives']['dt_simulation']
            # times 100 for two-significant-digits accuracy, turns into centi-seconds
            self.time_stamps = ((np.arange(0, self.duration, self.dt_sim) + self.dt_sim) * 100).astype(int)

            # velocity related
            self.velocity_sample_min = configuration['primitives']['velocity_sample_min']
            self.velocity_sample_max = configuration['primitives']['velocity_sample_max']
            self.num_sample_velocity = configuration['primitives']['num_sample_velocity']

            # steering angle related
            self.steering_angle_sample_min = configuration['primitives']['steering_angle_sample_min']
            self.steering_angle_sample_max = configuration['primitives']['steering_angle_sample_max']
            self.num_sample_steering_angle = configuration['primitives']['num_sample_steering_angle']

            if np.isclose(self.steering_angle_sample_max, 0):
                self.steering_angle_sample_max = self.param_vehicle.steering.max

            # sample trajectories
            self.num_segments_trajectory = configuration['sample_trajectories']['num_segments_trajectory']
            self.num_simulations = configuration['sample_trajectories']['num_simulations']

    configuration = None
    parameter: Parameter = None

    @classmethod
    def load_configuration(cls, path_file_config):
        """
        loads input configuration file
        """
        with open(path_file_config, 'r') as stream:
            try:
                configuration = yaml.safe_load(stream)
                cls.configuration = configuration
                cls.parameter = cls.Parameter(configuration)

            except yaml.YAMLError as exc:
                print(exc)

    @classmethod
    def generate_motion_primitives(cls):
        """
        main function to generate motion primitives.
        """
        # create lists of possible samples for velocity and steering angle
        print(cls.parameter)
        list_samples_v, list_samples_sa = cls.generate_list_of_samples()

        # for some statistics
        # calculate number of possible states
        num_possible_states = len(list_samples_v) * len(list_samples_sa)
        # calculate number of possible combinations of initial states and final states
        num_possible_combinations = num_possible_states ** 2
        print("Total possible combinations of states: ", num_possible_combinations)

        count_accepted = 0

        # for saving results
        list_motion_primitives = []
        # number of time steps for simulation
        num_step_sim = int(int(cls.parameter.duration * 10) / int(cls.parameter.dt * 10))

        # v = velocity, sa = steering_angle
        # iterate through possible instances of the Cartesian product of list_samples_v and list_samples_sa
        # create all possible combinations of (v_start, sa_start) and (v_end, sa_end)
        print("Generating motion primitives...")
        bar_progress = tqdm(total=num_possible_combinations)
        for (v_start, sa_start) in itertools.product(list_samples_v, list_samples_sa):
            for (v_end, sa_end) in itertools.product(list_samples_v, list_samples_sa):

                # check input constraints
                is_valid_combination, input_a, input_sa_rate = cls.check_input_constraints(v_start, v_end, sa_start,
                                                                                           sa_end, cls.parameter)
                if not is_valid_combination:
                    continue

                # add initial state to the list of forward simulated states
                list_states_simulated = [State(position=np.array([0.0, 0.0]), velocity=v_start, steering_angle=sa_start,
                                               orientation=0.0, time_step=0)]

                input_state = State(acceleration=input_a, steering_angle_speed=input_sa_rate, time_step=0)

                for _ in range(num_step_sim):
                    state_simulated_next = cls.parameter.vehicle_dynamics.simulate_next_state(list_states_simulated[-1],
                                                                                              input_state,
                                                                                              cls.parameter.dt,
                                                                                              throw=False)
                    # if the simulation result is not valid
                    if not state_simulated_next:
                        break
                    else:
                        list_states_simulated.append(state_simulated_next)

                trajectory_simulated = Trajectory(initial_time_step=0, state_list=list_states_simulated)

                with warnings.catch_warnings():
                    warnings.simplefilter('ignore')

                    try:
                        is_feasible_trajectory, _ = feasibility_checker.trajectory_feasibility(trajectory_simulated,
                                                                                               cls.parameter.vehicle_dynamics,
                                                                                               cls.parameter.dt)
                    except:
                        break

                if is_feasible_trajectory:
                    count_accepted += 1
                    list_motion_primitives.append(trajectory_simulated)

            bar_progress.update(num_possible_states)

        bar_progress.close()

        print(f"Number of feasible motion primitives: {count_accepted}")

        return list_motion_primitives

    @classmethod
    def generate_list_of_samples(cls):
        """
        generates lists of samples for velocity and steering angle based on the input parameters
        """
        list_samples_v = np.linspace(cls.parameter.velocity_sample_min,
                                     cls.parameter.velocity_sample_max,
                                     cls.parameter.num_sample_velocity)

        list_samples_steering_angle = np.linspace(cls.parameter.steering_angle_sample_min,
                                                  cls.parameter.steering_angle_sample_max,
                                                  cls.parameter.num_sample_steering_angle)

        return list_samples_v, list_samples_steering_angle

    @classmethod
    def check_input_constraints(cls, v_start, v_end, sa_start, sa_end, parameter):
        """
        computes the required acceleration and steering rate, and checks whether they are of permissible values
        """
        # compute required inputs
        input_a = (v_end - v_start) / parameter.duration
        input_sa_rate = (sa_end - sa_start) / parameter.duration

        # check if the constraints are respected
        if abs(input_a) > abs(parameter.param_vehicle.longitudinal.a_max) or \
                abs(input_sa_rate) > abs(parameter.param_vehicle.steering.v_max):
            return False, None, None
        else:
            return True, input_a, input_sa_rate

    @classmethod
    def create_mirrored_primitives(cls, list_motion_primitives):
        """
        Mirrors the motion primitives
        """
        # make sure to make a deep copy
        list_motion_primitives_mirrored = copy.deepcopy(list_motion_primitives)

        for primitive in list_motion_primitives:
            list_states_mirrored = []

            for state in primitive.state_list:
                # add mirrored state into list
                state_mirrored = State(position=np.array([state.position[0], -state.position[1]]),
                                       velocity=state.velocity,
                                       steering_angle=-state.steering_angle,
                                       orientation=-state.orientation,
                                       time_step=state.time_step)

                list_states_mirrored.append(state_mirrored)

            primitive_mirrored = Trajectory(initial_time_step=0, state_list=list_states_mirrored)

            if not cls.is_identical(primitive, primitive_mirrored):
                list_motion_primitives_mirrored.append(primitive_mirrored)

        return list_motion_primitives_mirrored

    @classmethod
    def is_identical(cls, primitive, primitive_mirrored):
        """
        Checks whether the two input primitives are identical
        """
        list_results = []
        for state1, state2 in zip(primitive.state_list, primitive_mirrored.state_list):
            if np.isclose(state1.position[1], state2.position[1]) and \
                    np.isclose(state1.steering_angle, state2.steering_angle) and \
                    np.isclose(state1.orientation, state2.orientation):
                list_results.append(True)
            else:
                list_results.append(False)

        return all(list_results)

    @classmethod
    def compute_branching_factor(cls, list_motion_primitives):
        """
        computes the average branching factor (number of successors) of primitives
        """
        list_count_successors = []

        with tqdm(total=len(list_motion_primitives)) as bar_progress:
            for primitive_predecessor in list_motion_primitives:
                count_successors = 0

                for primitive_successor in list_motion_primitives:

                    state_final_pp = primitive_predecessor.state_list[-1]
                    state_initial_ps = primitive_successor.state_list[0]

                    if abs(state_final_pp.velocity - state_initial_ps.velocity) < 0.02 and \
                            abs(state_final_pp.steering_angle - state_initial_ps.steering_angle) < 0.02:
                        count_successors += 1

                list_count_successors.append(count_successors)
                bar_progress.update(1)

        return np.mean(list_count_successors)

    @classmethod
    def generate_sample_trajectories(cls, list_motion_primitives):
        """
        generates sample trajectories and check if they pass the solution checker
        """
        print(cls.parameter)
        random.seed()
        for count_run in range(cls.parameter.num_simulations):
            count_segments_primitives = 0
            num_primitives = len(list_motion_primitives)

            # get a random starting primitive
            idx_primitive = random.randrange(num_primitives)
            list_primitive = []

            while count_segments_primitives < cls.parameter.num_segments_trajectory:
                # retrieve primitive
                primitive_predecessor = copy.deepcopy(list_motion_primitives[idx_primitive])
                list_primitive.append(primitive_predecessor)
                list_successors_primitive_predecessor = []
                count_segments_primitives += 1

                # obtain its successors
                for j in range(num_primitives):
                    # 2bc = to be connected
                    primitive_successor = list_motion_primitives[j]

                    state_final_pp = primitive_predecessor.state_list[-1]
                    state_initial_ps = primitive_successor.state_list[0]

                    if abs(state_final_pp.velocity - state_initial_ps.velocity) < 0.02 and \
                            abs(state_final_pp.steering_angle - state_initial_ps.steering_angle) < 0.02:
                        list_successors_primitive_predecessor.append(j)

                # start over if a primitive does not have a valid successor
                num_successors_primitive = len(list_successors_primitive_predecessor)
                if num_successors_primitive == 0:
                    count_segments_primitives = 0
                    idx_primitive = random.randrange(num_primitives)
                    list_primitive = []
                else:
                    # retrieve a random successor id
                    idx_successor = random.randrange(num_successors_primitive)
                    idx_primitive = list_successors_primitive_predecessor[idx_successor]

            plt.figure(figsize=(7, 7))
            # plot first trajectory
            list_states_final = copy.deepcopy(list_primitive[0].state_list)
            list_x = [state.position[0] for state in list_states_final]
            list_y = [state.position[1] for state in list_states_final]
            plt.scatter(list_x, list_y)

            # plot remaining trajectories
            for i in range(1, len(list_primitive)):
                traj_pre = list_primitive[i - 1]
                traj_cur = list_primitive[i]

                # retrieve states
                state_final_traj_pre = traj_pre.state_list[-1]

                # rotate + translate with regard to the last state of previous trajectory
                traj_cur.translate_rotate(np.zeros(2), state_final_traj_pre.orientation)
                traj_cur.translate_rotate(state_final_traj_pre.position, 0)

                list_x = [state.position[0] for state in traj_cur.state_list]
                list_y = [state.position[1] for state in traj_cur.state_list]
                plt.scatter(list_x, list_y)

                # discard the first state of second trajectory onward to prevent duplication
                traj_cur.state_list.pop(0)
                list_states_final += traj_cur.state_list

            list_x = [state.position[0] for state in list_states_final]
            list_y = [state.position[1] for state in list_states_final]
            plt.xlim([min(list_x) - 2, max(list_x) + 2])
            plt.ylim([min(list_y) - 2, max(list_y) + 2])
            plt.axis('equal')
            plt.show()

            trajectory_simulated = Trajectory(initial_time_step=0, state_list=list_states_final)

            with warnings.catch_warnings():
                warnings.simplefilter('ignore')

                try:
                    is_feasible_trajectory, _ = feasibility_checker.trajectory_feasibility(trajectory_simulated,
                                                                                           cls.parameter.vehicle_dynamics,
                                                                                           cls.parameter.dt)
                except:
                    break

                if not is_feasible_trajectory:
                    print("This is not good :(")

    @classmethod
    def save_motion_primitives(cls, list_motion_primitives):
        """
        Saves the generated motion primitives to XML file.
        """
        # create XML content
        node_xml = cls.create_xml_node(list_motion_primitives)
        node_xml_prettified = minidom.parseString(et.tostring(node_xml)).toprettyxml(indent="   ")
        dir_output = cls.parameter.dir_output
        # compute rounded step sizes for velocity and steering angle
        if cls.parameter.num_sample_velocity == 1:
            step_v = 0
        else:
            step_v = round((cls.parameter.velocity_sample_max - cls.parameter.velocity_sample_min) /
                           (cls.parameter.num_sample_velocity - 1), 2)

        if cls.parameter.num_sample_steering_angle == 1:
            step_sa = 0
        else:
            step_sa = round(cls.parameter.steering_angle_sample_max * 2 /
                            (cls.parameter.num_sample_steering_angle - 1) / 2, 2)

        file_name = "V_{}_{}_Vstep_{}_SA_{}_{}_SAstep_{}_T_{}_Model_{}.xml".format(cls.parameter.velocity_sample_min,
                                                                                   cls.parameter.velocity_sample_max,
                                                                                   step_v,
                                                                                   -cls.parameter.steering_angle_sample_max,
                                                                                   cls.parameter.steering_angle_sample_max,
                                                                                   step_sa,
                                                                                   round(cls.parameter.duration, 1),
                                                                                   cls.parameter.name_vehicle)

        with open(os.path.join(dir_output, file_name), "w") as f:
            f.write(node_xml_prettified)
            print("File saved: {}".format(file_name))

    @classmethod
    def create_xml_node(cls, list_motion_primitives):
        # create root tag
        node_root = et.Element('MotionPrimitiveFile')

        # create vehicle type tag
        node_vehicle_type = et.SubElement(node_root, 'VehicleType')
        node_vehicle_type.text = cls.parameter.name_vehicle

        # create Trajectories tag
        node_trajectories = et.SubElement(node_root, 'Trajectories')

        for motion_primitive in list_motion_primitives:

            # create a tag for individual motion_primitive
            node_trajectory = et.SubElement(node_trajectories, 'Trajectory')

            # add time duration tag
            node_duration = et.SubElement(node_trajectory, 'Duration')
            node_duration.set('unit', 'deci-second')
            node_duration.text = str(int(cls.parameter.duration * 10))

            list_states_primitive = motion_primitive.state_list

            # add start state
            node_start = et.SubElement(node_trajectory, 'Initial')
            node_x = et.SubElement(node_start, 'x')
            node_y = et.SubElement(node_start, 'y')
            node_sa = et.SubElement(node_start, 'steering_angle')
            node_v = et.SubElement(node_start, 'velocity')
            node_o = et.SubElement(node_start, 'orientation')
            node_t = et.SubElement(node_start, 'time_step')

            state_start = list_states_primitive[0]

            node_x.text = str(state_start.position[0])
            node_y.text = str(state_start.position[1])
            node_sa.text = str(state_start.steering_angle)
            node_v.text = str(state_start.velocity)
            node_o.text = str(state_start.orientation)
            node_t.text = str(state_start.time_step)

            # add final state
            node_final = et.SubElement(node_trajectory, 'Final')
            node_x = et.SubElement(node_final, 'x')
            node_y = et.SubElement(node_final, 'y')
            node_sa = et.SubElement(node_final, 'steering_angle')
            node_v = et.SubElement(node_final, 'velocity')
            node_o = et.SubElement(node_final, 'orientation')
            node_t = et.SubElement(node_final, 'time_step')

            state_final = list_states_primitive[-1]

            node_x.text = str(state_final.position[0])
            node_y.text = str(state_final.position[1])
            node_sa.text = str(state_final.steering_angle)
            node_v.text = str(state_final.velocity)
            node_o.text = str(state_final.orientation)
            node_t.text = str(state_final.time_step)

            # add states in between
            list_states_in_between = list_states_primitive[1:-1]

            node_path = et.SubElement(node_trajectory, 'Path')

            for state in list_states_in_between:
                node_state = et.SubElement(node_path, 'State')

                node_x = et.SubElement(node_state, 'x')
                node_y = et.SubElement(node_state, 'y')
                node_sa = et.SubElement(node_state, 'steering_angle')
                node_v = et.SubElement(node_state, 'velocity')
                node_o = et.SubElement(node_state, 'orientation')
                node_t = et.SubElement(node_state, 'time_step')

                node_x.text = str(state.position[0])
                node_y.text = str(state.position[1])
                node_sa.text = str(state.steering_angle)
                node_v.text = str(state.velocity)
                node_o.text = str(state.orientation)
                node_t.text = str(state.time_step)

        return node_root
