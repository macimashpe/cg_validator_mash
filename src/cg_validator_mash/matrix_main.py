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
    md_robot = Model_MD.MD(payload_mass)
    pass

    # test all possible payload cg locations
    stable_cgs = []
    highest_z_at_xy = {}
    cg_range_step = cfg_data['misc']['cg_range_step']
    cg_boundary_x = cfg_data['misc']['cg_range_x']
    cg_range_x = np.arange(-cg_boundary_x, cg_boundary_x, cg_range_step)
    cg_boundary_y = cfg_data['misc']['cg_range_y']
    cg_range_y = np.arange(-cg_boundary_y, cg_boundary_y, cg_range_step)
    cg_boundary_z = cfg_data['misc']['cg_range_z']
    cg_range_z = np.arange(ld_height, ld_height + cg_boundary_z, cg_range_step)

    for xi, payload_cg_x in enumerate(cg_range_x):
        for yi, payload_cg_y in enumerate(cg_range_y):
            # temp_x_y = [payload_cg_x, payload_cg_y, 0]
            temp_highest_z = 0
            for zi, payload_cg_z in enumerate(cg_range_z):
                ld_robot.payload_cg = (payload_cg_x, payload_cg_y, payload_cg_z)
                wheel_forces = ld_robot.modelNoBrake(*ld_robot._combined_cg)
                # wheel_forces = ld_robot.modelBrake(*ld_robot._combined_cg)
                valid = ld_robot.normalDriveCriterion(wheel_forces)
                logger.debug(f'payload cg is: {ld_robot.payload_cg}. combined cg is: {ld_robot._combined_cg}. valid is {valid}')
                if valid:
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

    # for cg in highest_z_at_xy:
    #     cg_xs.append(cg[0])
    #     cg_ys.append(cg[1])
    #     cg_zs.append(cg[2])
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # ax.scatter(cg_xs, cg_ys, cg_zs)
    ax.plot_surface(X, Y, Z, alpha=0.75)
    ax.scatter([0], [0], [1], color='r')
    plt.show()

    with open('stable_cg_locations', 'w') as outfile:
        for stable_cg in stable_cgs:
            outfile.write(f'{str(stable_cg)}\n')
    pass

if __name__ == "__main__":
    main()