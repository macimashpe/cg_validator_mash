import logging
from pathlib import Path
import toml
from math import sin, cos

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
    
    ld250_normal_drive_Mk = cfg_data['matrix_models']['ld250_normal_drive']
    logger.debug(ld250_normal_drive_Mk)

    eval_global_vars = {'__builtins__':{},'sin': sin, 'cos': cos}
    eval_local_vars = {'Br1': 1,
                        'Br2': 1,
                        'Dc': 1,
                        'r': 1,
                        'x': 1,
                        'y': 1,
                        'Bf1': 1,
                        'Bf2': 1,
                        'Dd': 1,
                        'w': 1,
                        'h': 1,
                        'dir': 1,
                        'u': 1,
                        'M': 1,
                        'g': 1,
                        'Jz': 1,
                        'theta': 1,
                        'L': 1,
                        'R': 1,
                        'ax': 1
                        }
    ld250_brake_drive_Mk_eval = [[eval(item, eval_global_vars, eval_local_vars) for item in row] for row in ld250_normal_drive_Mk]
    logger.debug(ld250_brake_drive_Mk_eval)

if __name__ == "__main__":
    main()