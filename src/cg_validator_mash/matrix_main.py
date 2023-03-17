import logging
from math import cos, sin
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt, patches
from mpl_toolkits import mplot3d
import toml
from model_matrix import Model_MD, Model_LD250
from safety_zone_generator import zone_generator

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

def analyze_cg():
    # load cfg file with robot parameters
    cfg_file_path = Path(__file__).parent / 'data/cg_parameters_mash.toml'
    with open(cfg_file_path) as cfg_file:
        cfg_data = toml.load(cfg_file)
        logger.debug(f'cg toml config loaded')

    # init robot object from model libraries
    payload_mass = cfg_data['payload']['mass']
    # robot = Model_LD250.LD250(payload_mass)
    robot = Model_MD.MD(payload_mass)
    robot.payload_mass = payload_mass
    robot.centripetal_acceleration = 0
    robot_height = Model_MD.PLATFORM_Z
    '''robot.payload_cg = (0.05, 0, 0.048)
    # robot.velocity, robot.acceleration = velocity_acceleration_combos[2]
    robot.velocity, robot.acceleration = (2.2, 0.5)
    # wheel_forces = robot.model_no_brake_2(*robot.payload_cg)
    wheel_forces = robot.modelNoBrake(*robot._combined_cg)
    valid = robot.normalDriveCriterion(wheel_forces)
    if valid:
        logger.debug(f'single point test | valid combined cg: x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
    else:
        logger.debug(f'single point test | INVALID combined cg: x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
    # logger.debug(f'valid: {valid}, wheel forces: {wheel_forces}')
    robot.velocity, robot.acceleration = (-2.2, -0.5)
    # wheel_forces = robot.model_no_brake_2(*robot.payload_cg)
    wheel_forces = robot.modelNoBrake(*robot._combined_cg)
    valid = robot.normalDriveCriterion(wheel_forces)
    if valid:
        logger.debug(f'single point test | valid combined cg: x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
    else:
        logger.debug(f'single point test | INVALID combined cg: x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
    # logger.debug(f'valid: {valid}, wheel forces: {wheel_forces}')
    # for attr in dir(robot):
    #     print(f'{attr}: {getattr(robot, attr)}')
    '''

    # test all possible payload cg locations
    logger.debug(f'calculating cg validity')

    # test single point for weird saddle shape del when done
    robot.velocity, robot.acceleration = (-2.2, -0.5)
    # for temp_z in (0.001, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1):
    for temp_z in (0.05, 0.06):
        # temp_x, temp_y = (0.0861, -0.0784)
        # temp_x, temp_y = (0.1, 0.05)
        temp_x, temp_y = (0.025, 0.025)
        robot.payload_cg = (temp_x, temp_y, temp_z)  # 0.086
        wheel_forces_dict = robot.model_brake_2(*robot._combined_cg)
        valid = robot.normal_drive_criterion_2(wheel_forces_dict)
        logger.debug(f'temp z = {temp_z} | valid = {valid}')
        logger.debug(f'{wheel_forces_dict}')

    cg_range_step = cfg_data['misc']['cg_range_step']
    cg_boundary_x = cfg_data['misc']['cg_range_x']
    cg_range_x = np.arange(-cg_boundary_x, cg_boundary_x, cg_range_step)
    cg_boundary_y = cfg_data['misc']['cg_range_y']
    cg_range_y = np.arange(-cg_boundary_y, cg_boundary_y, cg_range_step)
    cg_boundary_z = cfg_data['misc']['cg_range_z']
    cg_range_z = np.arange(robot_height, robot_height + cg_boundary_z, cg_range_step)
    
    # velocity_acceleration_combos = ((2.2, 0.5), (-2.2, -0.5), (2.2, -1.3), (-2.2, 1.3))
    velocity_acceleration_combos = ((2.2, 0.5), (-2.2, -0.5), (2.2, -0.5), (-2.2, 0.5))

    X, Y = np.meshgrid(cg_range_x, cg_range_y)
    nrows, ncols = X.shape
    Z_0 = np.zeros(X.shape)
    Z_1 = np.zeros(X.shape)
    Z_2 = np.zeros(X.shape)
    Z_3 = np.zeros(X.shape)
    Z_4 = np.zeros(X.shape)
    Z_5 = np.zeros(X.shape)
    Z_6 = np.zeros(X.shape)
    Z_7 = np.zeros(X.shape)
    Z_AND_0_3 = np.zeros(X.shape)
    Z_AND_0_7 = np.zeros(X.shape)

    '''# used when debugging a certain double loop for sanity
    logger.debug(f'starting double loop search')
    Z_temp_0 = np.zeros(X.shape)
    Z_temp_1 = np.zeros(X.shape)
    cg_list_0 = []
    cg_list_1 = []
    for xi in range(nrows):
        for yi in range(ncols):
            temp_high_z = 0
            for zi, payload_cg_z in enumerate(cg_range_z):
                robot.payload_cg = (X[xi,yi], Y[xi,yi], payload_cg_z)
                # robot.payload_cg = (0.05, 0.0, 0.048)
                # robot.velocity, robot.acceleration = velocity_acceleration_combos[2]
                robot.velocity, robot.acceleration = (2.2, 0.5)
                # wheel_forces = robot.model_no_brake_2(*robot.payload_cg)
                wheel_forces = robot.modelNoBrake(*robot._combined_cg)
                valid = robot.normalDriveCriterion(wheel_forces)
                # logger.debug(f'valid: {valid}, wheel forces: {wheel_forces}')
                if valid:
                    cg_list_0.append(robot.payload_cg)
                    logger.debug(f'VALID | payload cg x {robot.payload_cg[0]} y {robot.payload_cg[1]} z {robot.payload_cg[2]}, combined cg x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
                else:
                    logger.debug(f'INVALID | payload cg x {robot.payload_cg[0]} y {robot.payload_cg[1]} z {robot.payload_cg[2]}, combined cg x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
                if valid and payload_cg_z > temp_high_z:
                    temp_high_z = payload_cg_z
            Z_temp_0[xi,yi] = temp_high_z

    for xi in range(nrows):
        for yi in range(ncols):
            temp_high_z = 0
            for zi, payload_cg_z in enumerate(cg_range_z):
                robot.payload_cg = (X[xi,yi], Y[xi,yi], payload_cg_z)
                # robot.payload_cg = (0.05, 0.0, 0.048)
                # robot.velocity, robot.acceleration = velocity_acceleration_combos[2]
                robot.velocity, robot.acceleration = (-2.2, -0.5)
                wheel_forces = robot.modelNoBrake(*robot._combined_cg)
                valid = robot.normalDriveCriterion(wheel_forces)
                # logger.debug(f'valid: {valid}, wheel forces: {wheel_forces}')
                if valid:
                    cg_list_1.append(robot.payload_cg)
                    logger.debug(f'VALID | payload cg x {robot.payload_cg[0]} y {robot.payload_cg[1]} z {robot.payload_cg[2]}, combined cg x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
                else:
                    logger.debug(f'INVALID | payload cg x {robot.payload_cg[0]} y {robot.payload_cg[1]} z {robot.payload_cg[2]}, combined cg x {robot._combined_cg[0]} y {robot._combined_cg[1]} z {robot._combined_cg[2]}')
                if valid and payload_cg_z > temp_high_z:
                    temp_high_z = payload_cg_z
            Z_temp_1[xi,yi] = temp_high_z

    logger.debug(f'plotting')
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    cg_x_0 = [x_ for x_, y_, z_ in cg_list_0]
    cg_y_0 = [y_ for x_, y_, z_ in cg_list_0]
    cg_z_0 = [z_ for x_, y_, z_ in cg_list_0]
    cg_x_1 = [x_ for x_, y_, z_ in cg_list_1]
    cg_y_1 = [y_ for x_, y_, z_ in cg_list_1]
    cg_z_1 = [z_ for x_, y_, z_ in cg_list_1]
    fig, ax = plt.subplots(2, 2, subplot_kw=dict(projection='3d'))
    ax[0,0].title.set_text(f'vel/acc Z_temp_0 2.2, 5\nmodelnobrake')
    ax[0,0].plot_surface(X, Y, Z_temp_0)
    ax[0,1].title.set_text(f'vel/acc Z_temp_0 -2.2, -5\nmodelnobrake')
    ax[0,1].plot_surface(X, Y, Z_temp_1)
    ax[1, 0].scatter(cg_x_0, cg_y_0, cg_z_0)
    ax[1, 1].scatter(cg_x_1, cg_y_1, cg_z_1)
    plt.show()'''

    # for xi, payload_cg_x in enumerate(cg_range_x):
    for xi in range(nrows):
        # for yi, payload_cg_y in enumerate(cg_range_y):
        for yi in range(ncols):
            temp_high_z_0 = 0
            temp_high_z_1 = 0
            temp_high_z_2 = 0
            temp_high_z_3 = 0
            temp_high_z_4 = 0
            temp_high_z_5 = 0
            temp_high_z_6 = 0
            temp_high_z_7 = 0
            for zi, payload_cg_z in enumerate(cg_range_z):
                robot.payload_cg = (X[xi,yi], Y[xi,yi], payload_cg_z)
                
                # vel/acc_0
                robot.velocity, robot.acceleration = velocity_acceleration_combos[0]
                wheel_forces = robot.modelNoBrake(*robot._combined_cg)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_0:
                    temp_high_z_0 = payload_cg_z

                # vel/acc_1
                robot.velocity, robot.acceleration = velocity_acceleration_combos[1]
                wheel_forces = robot.modelNoBrake(*robot._combined_cg)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_1:
                    temp_high_z_1 = payload_cg_z

                # vel/acc_2
                robot.velocity, robot.acceleration = velocity_acceleration_combos[2]
                wheel_forces = robot.modelNoBrake(*robot._combined_cg)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_2:
                    temp_high_z_2 = payload_cg_z

                # vel/acc_3
                robot.velocity, robot.acceleration = velocity_acceleration_combos[3]
                wheel_forces = robot.modelNoBrake(*robot._combined_cg)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_3:
                    temp_high_z_3 = payload_cg_z

                # vel/acc_4
                robot.velocity, robot.acceleration = velocity_acceleration_combos[0]
                wheel_forces = robot.modelBrake(*robot._combined_cg)
                # valid = robot.normalDriveCriterion_print(wheel_forces)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_4:
                    temp_high_z_4 = payload_cg_z

                # vel/acc_5
                robot.velocity, robot.acceleration = velocity_acceleration_combos[1]
                wheel_forces = robot.modelBrake(*robot._combined_cg)
                # valid = robot.normalDriveCriterion_print(wheel_forces)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_5:
                    temp_high_z_5 = payload_cg_z

                # vel/acc_6
                robot.velocity, robot.acceleration = velocity_acceleration_combos[2]
                wheel_forces = robot.modelBrake(*robot._combined_cg)
                # valid = robot.normalDriveCriterion_print(wheel_forces)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_6:
                    temp_high_z_6 = payload_cg_z

                # vel/acc_7
                robot.velocity, robot.acceleration = velocity_acceleration_combos[3]
                wheel_forces = robot.modelBrake(*robot._combined_cg)
                # valid = robot.normalDriveCriterion_print(wheel_forces)
                valid = robot.normalDriveCriterion(wheel_forces)
                if valid and payload_cg_z > temp_high_z_7:
                    temp_high_z_7 = payload_cg_z

            Z_0[xi,yi] = temp_high_z_0
            Z_1[xi,yi] = temp_high_z_1
            Z_2[xi,yi] = temp_high_z_2
            Z_3[xi,yi] = temp_high_z_3
            Z_4[xi,yi] = temp_high_z_4
            Z_5[xi,yi] = temp_high_z_5
            Z_6[xi,yi] = temp_high_z_6
            Z_7[xi,yi] = temp_high_z_7
            Z_AND_0_3[xi,yi] = min([temp_high_z_0, temp_high_z_1, temp_high_z_2, temp_high_z_3])
            Z_AND_0_7[xi,yi] = min([temp_high_z_0, temp_high_z_1, temp_high_z_2, temp_high_z_3, temp_high_z_4, temp_high_z_5, temp_high_z_6, temp_high_z_7])

    # logger.debug(f'{(X[9][12], Y[9][12], Z_5[9][12])}')
    logger.debug(f'plotting')
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    fig, ax = plt.subplots(2, 4, num='Individual velocity/acceleration cases', subplot_kw=dict(projection='3d'))
    ax[0,0].title.set_text(f'vel/acc 0 {str(velocity_acceleration_combos[0])}, modelnobrake')
    ax[0, 0].plot_surface(X, Y, Z_0, alpha=0.90, shade=True)
    ax[0, 0].set_xlabel('x')
    ax[0, 0].set_ylabel('y')
    ax[0, 0].set_zlabel('z')
    ax[0,1].title.set_text(f'vel/acc 1 {str(velocity_acceleration_combos[1])}, modelnobrake')
    ax[0, 1].plot_surface(X, Y, Z_1, alpha=0.90, shade=True)
    ax[0, 1].set_xlabel('x')
    ax[0, 1].set_ylabel('y')
    ax[0, 1].set_zlabel('z')
    ax[0, 2].title.set_text(f'vel/acc 2 {str(velocity_acceleration_combos[2])}, modelnobrake')
    ax[0, 2].plot_surface(X, Y, Z_2, alpha=0.90, shade=True)
    ax[0, 2].set_xlabel('x')
    ax[0, 2].set_ylabel('y')
    ax[0, 2].set_zlabel('z')
    ax[0, 3].title.set_text(f'vel/acc 3 {str(velocity_acceleration_combos[3])}, modelnobrake')
    ax[0, 3].plot_surface(X, Y, Z_3, alpha=0.90, shade=True)
    ax[0, 3].set_xlabel('x')
    ax[0, 3].set_ylabel('y')
    ax[0, 3].set_zlabel('z')
    ax[1, 0].title.set_text(f'vel/acc 4 {str(velocity_acceleration_combos[0])}, modelbrake')
    ax[1, 0].plot_surface(X, Y, Z_4, alpha=0.90, shade=True)
    ax[1, 0].set_xlabel('x')
    ax[1, 0].set_ylabel('y')
    ax[1, 0].set_zlabel('z')
    ax[1, 1].title.set_text(f'vel/acc 5 {str(velocity_acceleration_combos[1])}, modelbrake')
    ax[1, 1].plot_surface(X, Y, Z_5, alpha=0.90, shade=True)
    ax[1, 1].set_xlabel('x')
    ax[1, 1].set_ylabel('y')
    ax[1, 1].set_zlabel('z')
    ax[1, 2].title.set_text(f'vel/acc 6 {str(velocity_acceleration_combos[2])}, modelbrake')
    ax[1, 2].plot_surface(X, Y, Z_6, alpha=0.90, shade=True)
    ax[1, 2].set_xlabel('x')
    ax[1, 2].set_ylabel('y')
    ax[1, 2].set_zlabel('z')
    ax[1, 3].title.set_text(f'vel/acc 7 {str(velocity_acceleration_combos[3])}, modelbrake')
    ax[1, 3].plot_surface(X, Y, Z_7, alpha=0.90, shade=True)
    ax[1, 3].set_xlabel('x')
    ax[1, 3].set_ylabel('y')
    ax[1, 3].set_zlabel('z')
    ax[0, 0].scatter([0], [0], [1], color='r')
    fig2, ax2 = plt.subplots(1, 2, num='Overall ANDed valid CGs', subplot_kw=dict(projection='3d'))
    ax2[0].title.set_text(f'vel/acc AND, [0..3], modelnobrake')
    ax2[0].plot_surface(X, Y, Z_AND_0_3, alpha=0.90, shade=True)
    ax2[0].set_xlabel('x')
    ax2[0].set_ylabel('y')
    ax2[0].set_zlabel('z')
    ax2[1].title.set_text(f'vel/acc AND, [0..7], modelbrake')
    ax2[1].plot_surface(X, Y, Z_AND_0_7, alpha=0.90, shade=True)
    ax2[1].set_xlabel('x')
    ax2[1].set_ylabel('y')
    ax2[1].set_zlabel('z')

    # add safety zones to plot. currently a temp way to do it.
    z_line = np.ones(2) * 0
    y_line = (-cfg_data['misc']['cg_range_y'], cfg_data['misc']['cg_range_y'])
    logger.debug(f'Calculating LD safety zones...')
    # calculate LD straight zone lengths using v0, vf, a to find displacement
    ld_safety_zones = zone_generator.zone_creation_ld(
        # cfg_data['payload']['max_translational_deceleration'],
        Model_LD250.MAX_ACCELERATION_axa*1000,
        cfg_data['payload']['max_velocity'],
        cfg_data['payload']['cnt_subdivisions'],
        cfg_data['misc']['safety_factor'],
        cfg_data['payload']['extension'],
        robot.platform_dimensions)
    for zone_x in ld_safety_zones:
        ax2[1].plot((zone_x[1]/1000, zone_x[1]/1000), y_line, z_line)
    plt.show()

    # with open('stable_cg_locations', 'w') as outfile:
    #     for stable_cg in stable_cgs:
    #         outfile.write(f'{str(stable_cg)}\n')

def analyze_safety_zones():
    # load cfg file with robot parameters
    cfg_file_path = Path(__file__).parent / 'data/cg_parameters_mash.toml'
    with open(cfg_file_path) as cfg_file:
        cfg_data = toml.load(cfg_file)
        logger.debug(f'safety zones toml cfg loaded: {str(cfg_file_path)}')

    if cfg_data['platform']['type'] == "LD":
        logger.debug(f'Calculating LD safety zones...')
        # calculate LD straight zone lengths using v0, vf, a to find displacement
        ld_safety_zones = zone_generator.zone_creation_ld(
            # cfg_data['payload']['max_translational_deceleration'],
            Model_LD250.MAX_ACCELERATION_axa,
            cfg_data['payload']['max_velocity'],
            cfg_data['payload']['cnt_subdivisions'],
            cfg_data['payload']['straight_zone_safety_factor'],
            cfg_data['payload']['extension'],
            cfg_data['LD']['dimensions'])

        zone_generator.plot_ld_safety_zones(ld_safety_zones)
        pass

def main():
    valid_cgs = analyze_cg()
    valid_szs = analyze_safety_zones()


if __name__ == "__main__":
    main()