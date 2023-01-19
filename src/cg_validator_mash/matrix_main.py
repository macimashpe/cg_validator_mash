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

    test = ld_robot.modelNoBrake(0,0,0)
    test2 = ld_robot.normalDriveCriterion(test)
    pass

if __name__ == "__main__":
    main()