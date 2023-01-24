import logging
from math import cos, sin
from pathlib import Path

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
    ld_robot = Model_LD250.LD250(payload_mass)

    # test all possible payload cg locations
    for payload_cg_x in range(10):
        for payload_cg_y in range(10):
            for payload_cg_z in range(10):
                ld_robot.payload_cg = (payload_cg_x, payload_cg_y, payload_cg_z)
                wheel_forces = ld_robot.modelNoBrake(*ld_robot._combined_cg)
                valid = ld_robot.normalDriveCriterion(wheel_forces)
                print(f'payload cg is: {ld_robot.payload_cg}. combined cg is: {ld_robot._combined_cg}. valid is {valid}')
    pass

if __name__ == "__main__":
    main()