import toml
import pathlib
import math

def main():
    cfg_file_path = pathlib.Path(__file__).parent / 'data/cg_parameters_mash.toml'
    with open(cfg_file_path) as cfg_file:
        cfg_data = toml.load(cfg_file)
    
    ld250_brake_drive_Mk = cfg_data['matrix_models']['ld250_brake_drive_Mk']
    print(ld250_brake_drive_Mk)

    eval_global_vars = {'__builtins__':{},'sin': math.sin, 'cos': math.cos}
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
                        'R': 1
                        }
    ld250_brake_drive_Mk_eval = [[eval(item, eval_global_vars, eval_local_vars) for item in row] for row in ld250_brake_drive_Mk]
    print(ld250_brake_drive_Mk_eval)

if __name__ == "__main__":
    main()