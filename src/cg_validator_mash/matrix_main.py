import logging
from math import cos, sin
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import toml
from model_matrix import Model_MD, Model_LD250

# configure logger for debug messages
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s : %(levelname)s : %(name)s : %(message)s')
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)
file_handler = logging.FileHandler(filename="matrix_log.log")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

def main():
    # load cfg file with robot parameters
    cfg_file_path = Path(__file__).parent / 'data/cg_parameters_mash.toml'
    with open(cfg_file_path) as cfg_file:
        cfg_data = toml.load(cfg_file)
        logger.debug(f'toml config loaded')

    # init robot object from model libraries
    payload_mass = cfg_data['payload']['mass']
    # ld_robot = Model_LD250.LD250(payload_mass)
    # ld_height = cfg_data['LD']['dimensions']['height'] / 100  # height in m
    ld_robot = Model_MD.MD(payload_mass)
    # ld_height = cfg_data['LD']['dimensions']['height'] / 100  # height in m
    ld_height = Model_MD.PLATFORM_Z
    pass

    # test all possible payload cg locations
    stable_cgs_0 = []
    stable_cgs_1 = []
    stable_cgs_2 = []
    stable_cgs_3 = []
    stable_cgs_4 = []
    stable_cgs_5 = []
    stable_cgs_6 = []
    stable_cgs_7 = []
    highest_z_at_xy_0 = {}
    highest_z_at_xy_1 = {}
    highest_z_at_xy_2 = {}
    highest_z_at_xy_3 = {}
    highest_z_at_xy_4 = {}
    highest_z_at_xy_5 = {}
    highest_z_at_xy_6 = {}
    highest_z_at_xy_7 = {}
    cg_range_step = cfg_data['misc']['cg_range_step']
    cg_boundary_x = cfg_data['misc']['cg_range_x']
    cg_range_x = np.arange(-cg_boundary_x, cg_boundary_x, cg_range_step)
    cg_boundary_y = cfg_data['misc']['cg_range_y']
    cg_range_y = np.arange(-cg_boundary_y, cg_boundary_y, cg_range_step)
    cg_boundary_z = cfg_data['misc']['cg_range_z']
    cg_range_z = np.arange(ld_height, ld_height + cg_boundary_z, cg_range_step)
    
    velocity_acceleration_combos = ((2.2, 0.5), (-2.2, -0.5), (2.2, -1.3), (-2.2, 1.3))

    for xi, payload_cg_x in enumerate(cg_range_x):
        for yi, payload_cg_y in enumerate(cg_range_y):
            # temp_x_y = [payload_cg_x, payload_cg_y, 0]
            temp_highest_z_0 = 0
            temp_highest_z_1 = 0
            temp_highest_z_2 = 0
            temp_highest_z_3 = 0
            temp_highest_z_4 = 0
            temp_highest_z_5 = 0
            temp_highest_z_6 = 0
            temp_highest_z_7 = 0
            for zi, payload_cg_z in enumerate(cg_range_z):
                ld_robot.payload_cg = (payload_cg_x, payload_cg_y, payload_cg_z)
                
                # vel/acc_1
                ld_robot.velocity, ld_robot.acceleration = velocity_acceleration_combos[0]
                wheel_forces = ld_robot.modelNoBrake(*ld_robot._combined_cg)
                # wheel_forces = ld_robot.modelBrake(*ld_robot._combined_cg)
                valid = ld_robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_highest_z_0:
                    temp_highest_z_0 = payload_cg_z
                # if not valid:
                #     logger.debug(f'acc1 not valid')
                #     continue
                # logger.debug(f'acc1 valid')

                # vel/acc_2
                ld_robot.velocity, ld_robot.acceleration = velocity_acceleration_combos[1]
                wheel_forces = ld_robot.modelNoBrake(*ld_robot._combined_cg)
                # wheel_forces = ld_robot.modelBrake(*ld_robot._combined_cg)
                valid = ld_robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_highest_z_1:
                    temp_highest_z_1 = payload_cg_z
                # if not valid:
                #     logger.debug(f'acc2 not valid')
                #     continue
                # logger.debug(f'acc2 valid')

                # vel/acc_3
                ld_robot.velocity, ld_robot.acceleration = velocity_acceleration_combos[2]
                wheel_forces = ld_robot.modelNoBrake(*ld_robot._combined_cg)
                # wheel_forces = ld_robot.modelBrake(*ld_robot._combined_cg)
                valid = ld_robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_highest_z_2:
                    temp_highest_z_2 = payload_cg_z
                # if not valid:
                #     logger.debug(f'acc3 not valid')
                #     continue
                # logger.debug(f'acc3 valid')

                # vel/acc_4
                ld_robot.velocity, ld_robot.acceleration = velocity_acceleration_combos[3]
                wheel_forces = ld_robot.modelNoBrake(*ld_robot._combined_cg)
                # wheel_forces = ld_robot.modelBrake(*ld_robot._combined_cg)
                valid = ld_robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_highest_z_3:
                    temp_highest_z_3 = payload_cg_z
                # if not valid:
                #     logger.debug(f'acc4 not valid')
                #     continue
                # logger.debug(f'acc4 valid')

                # logger.debug(f'payload cg is: {ld_robot.payload_cg}. combined cg is: {ld_robot._combined_cg}. valid is {valid}')
                stable_cgs.append((payload_cg_x, payload_cg_y, payload_cg_z))
                if payload_cg_z > temp_highest_z:
                    temp_highest_z = payload_cg_z
            highest_z_at_xy[(payload_cg_x, payload_cg_y)] = (xi, yi, temp_highest_z)
    pass

    X, Y = np.meshgrid(cg_range_x, cg_range_y)
    Z = np.zeros(X.shape)
    for coords, z_info in highest_z_at_xy.items():
        xi = z_info[0]
        yi = z_info[1]
        temp_highest_z = z_info[2]
        Z[yi][xi] = temp_highest_z
    pass

    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    fig, ax = plt.subplot(2, 4)
    # ax.scatter(cg_xs, cg_ys, cg_zs)
    ax[0, 0].plot_surface(X, Y, Z, alpha=0.75, shade=True, color='orange')
    ax[0, 0].scatter([0], [0], [1], color='r')
    plt.show()

    with open('stable_cg_locations', 'w') as outfile:
        for stable_cg in stable_cgs:
            outfile.write(f'{str(stable_cg)}\n')
    pass

if __name__ == "__main__":
    main()