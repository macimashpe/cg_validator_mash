"""Analyze the valid center-of-gravity and safety-zone characteristics of LD/MD/HD.

Dev: Maci Miri, maci@mashpe.net

Info: Leverages a previous version of the cg analaysis library provided by 
Shen from Omron. 
The safety-zone analysis was implemented using 1-D kinematics 
and needs real-world verification/tweaking.
"""

import logging
from math import cos, sin
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt, patches
from mpl_toolkits import mplot3d
import toml
from model_matrix import BaseModel, Model_MD, Model_LD250
from safety_zone_generator import zone_generator
import sys

CFG_FILE_PATH = Path(__file__).parent / "data/cg_parameters_mash.toml"

# configure logger for debug messages
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s : %(levelname)s : %(name)s : %(message)s")
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)
file_handler = logging.FileHandler(filename="matrix_log.log")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)


def analyze_cg(robot: BaseModel, cfg_data: dict):
    """Analyzes a volume above the robot platform and returns highest valid payload CGs.

    Parameters:
    robot (BaseModel): model with matrix representation
    cfg_data (dict): settings loaded from config file

    Returns:
    tuple: array of highest valid CGs at every XY point above platform
    """

    # init robot object from model libraries
    payload_mass = cfg_data["payload"]["mass"]
    robot.payload_mass = payload_mass
    robot.centripetal_acceleration = 0
    # TODO: fix m vs mm in cg vs sz libraries
    robot_height = robot.platform_dimensions["height"] / 1000

    # test all possible payload cg locations
    logger.debug(f"calculating cg validity")

    cg_range_step = cfg_data["misc"]["cg_range_step"]
    cg_boundary_x = cfg_data["misc"]["cg_range_x"]
    cg_range_x = np.arange(-cg_boundary_x, cg_boundary_x, cg_range_step)
    cg_boundary_y = cfg_data["misc"]["cg_range_y"]
    cg_range_y = np.arange(-cg_boundary_y, cg_boundary_y, cg_range_step)
    cg_boundary_z = cfg_data["misc"]["cg_range_z"]
    cg_range_z = np.arange(robot_height, robot_height + cg_boundary_z, cg_range_step)
    velocity_acceleration_combos = cfg_data["misc"]["velocity_acceleration_combos"]

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

    # iterate through all possible payload cg locations and check for stability
    for xi in range(nrows):
        for yi in range(ncols):
            temp_high_z_0 = 0
            temp_high_z_1 = 0
            temp_high_z_2 = 0
            temp_high_z_3 = 0
            temp_high_z_4 = 0
            temp_high_z_5 = 0
            temp_high_z_6 = 0
            temp_high_z_7 = 0
            for payload_cg_z in cg_range_z:
                robot.payload_cg = (X[xi, yi], Y[xi, yi], payload_cg_z)

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

            Z_0[xi, yi] = temp_high_z_0
            Z_1[xi, yi] = temp_high_z_1
            Z_2[xi, yi] = temp_high_z_2
            Z_3[xi, yi] = temp_high_z_3
            Z_4[xi, yi] = temp_high_z_4
            Z_5[xi, yi] = temp_high_z_5
            Z_6[xi, yi] = temp_high_z_6
            Z_7[xi, yi] = temp_high_z_7
            Z_AND_0_3[xi, yi] = min(
                [temp_high_z_0, temp_high_z_1, temp_high_z_2, temp_high_z_3]
            )
            Z_AND_0_7[xi, yi] = min(
                [
                    temp_high_z_0,
                    temp_high_z_1,
                    temp_high_z_2,
                    temp_high_z_3,
                    temp_high_z_4,
                    temp_high_z_5,
                    temp_high_z_6,
                    temp_high_z_7,
                ]
            )

    return (X, Y, Z_0, Z_1, Z_2, Z_3, Z_4, Z_5, Z_6, Z_7, Z_AND_0_3, Z_AND_0_7)


def analyze_ld_safety_zones(robot, cfg_data):
    """Analyze LD safety zones based on top speed and acceleration given by center of gravity analysis.

    Parameters:
    robot (BaseModel): model with matrix representation
    cfg_data (dict): settings loaded from config file

    Returns:
    tuple: array of width and length of safety zones
    """

    logger.debug(f"Calculating LD safety zones...")
    # calculate LD straight zone lengths using v0, vf, a to find displacement
    safety_zones = zone_generator.zone_creation_ld(
        # cfg_data['payload']['max_translational_deceleration'],
        Model_LD250.MAX_ACCELERATION_axa * 1000,
        cfg_data["payload"]["max_velocity"],
        cfg_data["payload"]["cnt_subdivisions"],
        cfg_data["misc"]["safety_factor"],
        cfg_data["payload"]["extension"],
        robot.platform_dimensions,
    )

    with open("safety_zones.csv", "w") as outfile:
        outfile.write(f"zone,width [mm],length [mm]\n")
        for zone, dimensions in enumerate(safety_zones):
            outfile.write(f"{zone},{str(dimensions)[1:-1]}\n")

    return safety_zones


def plot_cg_sz(cg_z_values, safety_zone_values, cfg_data, plot_safety_zones=True):
    """Plot 3D values provided by previous analysis.

    Parameters:
    cg_z_values (tuple): array of highest valid CGs at every XY point above platform
    safety_zone_values (tuple): array of width and length of safety zones
    cfg_data (dict): settings loaded from config file
    plot_safety_zones (bool): if true, add the safety zone lines to the 3D plots.
    """

    logger.debug(f"plotting cgs")
    velocity_acceleration_combos = cfg_data["misc"]["velocity_acceleration_combos"]
    plt.rcParams["figure.figsize"] = [20, 8]

    # plot surfaces for each of the velocity/acceleration cases, as well as the combined ANDED plots
    (X, Y, Z_0, Z_1, Z_2, Z_3, Z_4, Z_5, Z_6, Z_7, Z_AND_0_3, Z_AND_0_7) = cg_z_values
    fig, ax = plt.subplots(
        2,
        4,
        num="Individual velocity/acceleration cases",
        subplot_kw=dict(projection="3d"),
    )
    ax[0, 0].title.set_text(
        f"vel/acc 0 {str(velocity_acceleration_combos[0])}, modelnobrake"
    )
    ax[0, 0].plot_surface(X, Y, Z_0, alpha=0.90, shade=True)
    ax[0, 0].set_xlabel("x")
    ax[0, 0].set_ylabel("y")
    ax[0, 0].set_zlabel("z")
    ax[0, 1].title.set_text(
        f"vel/acc 1 {str(velocity_acceleration_combos[1])}, modelnobrake"
    )
    ax[0, 1].plot_surface(X, Y, Z_1, alpha=0.90, shade=True)
    ax[0, 1].set_xlabel("x")
    ax[0, 1].set_ylabel("y")
    ax[0, 1].set_zlabel("z")
    ax[0, 2].title.set_text(
        f"vel/acc 2 {str(velocity_acceleration_combos[2])}, modelnobrake"
    )
    ax[0, 2].plot_surface(X, Y, Z_2, alpha=0.90, shade=True)
    ax[0, 2].set_xlabel("x")
    ax[0, 2].set_ylabel("y")
    ax[0, 2].set_zlabel("z")
    ax[0, 3].title.set_text(
        f"vel/acc 3 {str(velocity_acceleration_combos[3])}, modelnobrake"
    )
    ax[0, 3].plot_surface(X, Y, Z_3, alpha=0.90, shade=True)
    ax[0, 3].set_xlabel("x")
    ax[0, 3].set_ylabel("y")
    ax[0, 3].set_zlabel("z")
    ax[1, 0].title.set_text(
        f"vel/acc 4 {str(velocity_acceleration_combos[0])}, modelbrake"
    )
    ax[1, 0].plot_surface(X, Y, Z_4, alpha=0.90, shade=True)
    ax[1, 0].set_xlabel("x")
    ax[1, 0].set_ylabel("y")
    ax[1, 0].set_zlabel("z")
    ax[1, 1].title.set_text(
        f"vel/acc 5 {str(velocity_acceleration_combos[1])}, modelbrake"
    )
    ax[1, 1].plot_surface(X, Y, Z_5, alpha=0.90, shade=True)
    ax[1, 1].set_xlabel("x")
    ax[1, 1].set_ylabel("y")
    ax[1, 1].set_zlabel("z")
    ax[1, 2].title.set_text(
        f"vel/acc 6 {str(velocity_acceleration_combos[2])}, modelbrake"
    )
    ax[1, 2].plot_surface(X, Y, Z_6, alpha=0.90, shade=True)
    ax[1, 2].set_xlabel("x")
    ax[1, 2].set_ylabel("y")
    ax[1, 2].set_zlabel("z")
    ax[1, 3].title.set_text(
        f"vel/acc 7 {str(velocity_acceleration_combos[3])}, modelbrake"
    )
    ax[1, 3].plot_surface(X, Y, Z_7, alpha=0.90, shade=True)
    ax[1, 3].set_xlabel("x")
    ax[1, 3].set_ylabel("y")
    ax[1, 3].set_zlabel("z")
    ax[0, 0].scatter([0], [0], [1], color="r")
    plt.savefig("vel_acc_combos.png")

    # create ANDED plots
    fig2, ax2 = plt.subplots(
        1, 2, num="Overall ANDed valid CGs", subplot_kw=dict(projection="3d")
    )
    ax2[0].title.set_text(f"vel/acc AND, [0..3], modelnobrake")
    ax2[0].plot_surface(X, Y, Z_AND_0_3, alpha=0.90, shade=True)
    ax2[0].set_xlabel("x")
    ax2[0].set_ylabel("y")
    ax2[0].set_zlabel("z")
    ax2[1].title.set_text(f"vel/acc AND, [0..7], modelbrake")
    ax2[1].plot_surface(X, Y, Z_AND_0_7, alpha=0.90, shade=True)
    ax2[1].set_xlabel("x")
    ax2[1].set_ylabel("y")
    ax2[1].set_zlabel("z")

    # add safety zone lines to plot
    if plot_safety_zones:
        z_line = np.ones(2) * 0
        y_line = (-cfg_data["misc"]["cg_range_y"], cfg_data["misc"]["cg_range_y"])
        for zone_x in safety_zone_values:
            ax2[0].plot((zone_x[1] / 1000, zone_x[1] / 1000), y_line, z_line)
            ax2[1].plot((zone_x[1] / 1000, zone_x[1] / 1000), y_line, z_line)

    plt.savefig("ANDED_valid_cgs.png")
    plt.show()


def main():
    """Take in data from cfg and perform center-of-gravity and safety-zone analysis.
    """

    # load config file
    with open(CFG_FILE_PATH) as cfg_file:
        cfg_data = toml.load(cfg_file)
        logger.debug(f"safety zones toml cfg loaded: {str(CFG_FILE_PATH)}")

    # instantiate robot based on platform provided by cfg_data
    platform_type = cfg_data["platform"]["type"]
    match platform_type:
        case "LD":
            robot = Model_LD250.LD250()
        case "MD":
            robot = Model_MD.MD()
        case _:
            logger.debug(
                f"Have not yet implemented {platform_type} platform type specified in cfg_file"
            )
            sys.exit(
                f"Have not yet implemented {platform_type} platform type specified in cfg_file"
            )

    # analyze center-of-gravity
    valid_cg_z_vals = analyze_cg(robot, cfg_data)

    # analyze safety-zone (only if LD currently)
    if platform_type == "LD":
        valid_szs = analyze_ld_safety_zones(robot, cfg_data)
    else:
        logger.debug(
            f"Warning: not calculating safety zones for {platform_type} - simple model only used for LD."
        )

    # plot cg and sz data in 3D plots
    plot_safety_zones = cfg_data["misc"]["plot_safety_zones"]
    plot_cg_sz(valid_cg_z_vals, valid_szs, cfg_data, plot_safety_zones)


if __name__ == "__main__":
    main()
