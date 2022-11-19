import math
import pathlib
import toml
import logging

# configure logger for debug messages
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s : %(levelname)s : %(name)s : %(message)s')
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)
file_handler = logging.FileHandler(filename="log.log")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

if __name__ == "__main__":
    # load cfg file with robot parameters
    cfg_file_path = pathlib.Path(__file__).parent / 'data/cg_parameters_mash.toml'
    with open(cfg_file_path) as cfg_file:
        cfg_data = toml.load(cfg_file)
        logger.debug(f'loaded toml cfg file successfully : {str(cfg_file_path)}')

    # Get/calculate all necessary variables

    # need m_fsp2, m_w2, ld250_m, load_m, phi, and CG Z height to calc max_accel
    platform = cfg_data['platform']['type']
    caster_swivel = cfg_data[platform]['dimensions']['caster_swivel']
    platform_cg_x = cfg_data[platform]['dimensions']['platform_cg_x']
    platform_cg_y = cfg_data[platform]['dimensions']['platform_cg_y']
    platform_cg_z = cfg_data[platform]['dimensions']['platform_cg_z']
    platform_mass = cfg_data[platform]['dimensions']['mass']
    front_caster_x = cfg_data[platform]['dimensions']['front_caster_x']
    front_caster_y = cfg_data[platform]['dimensions']['front_caster_y']
    front_caster_y_minus = front_caster_y-caster_swivel		#(mm) wheel behind the caster mounting point (moving forward)
    front_caster_y_plus = front_caster_y+caster_swivel		#(mm) wheel in front of caster mounting point(moving backward)
    rear_caster_x = cfg_data[platform]['dimensions']['rear_caster_x']
    rear_caster_y = cfg_data[platform]['dimensions']['rear_caster_y']
    rear_caster_y_minus = rear_caster_y-caster_swivel		#(mm) wheel behind the caster mounting point (moving forward)
    rear_caster_y_plus= rear_caster_y+caster_swivel		#(mm) wheel in front of caster mounting point(moving backward)
    drive_wheel_x = cfg_data[platform]['dimensions']['drive_wheel_x']
    drive_wheel_y = cfg_data[platform]['dimensions']['drive_wheel_y']

    payload_mass = cfg_data['payload']['mass']
    payload_cg_x = cfg_data['payload']['cg']['x']
    payload_cg_y = cfg_data['payload']['cg']['y']
    payload_cg_z = cfg_data['payload']['cg']['z']

    # Incline angle (rad), representing the ramp on the floor. If using FL of 15, the angle would be about 0.5 degrees
    # psi = 0.5*math.pi/180
    psi = cfg_data['misc']['psi']
    # ramp angle, not to be confused with psi which represenets minor out of levelness of the floor
    # phi is used to determine transverse tipping when ld250 is sideways on the ramp
    # phi = 3*math.pi/180 	# this is lower than the 5degree used for wheel chair ramps
    phi = cfg_data['misc']['phi']
    # angle of of the FO axis relative to the center line of ld250 parallel to y-axis(rad)
    th = math.atan(abs((rear_caster_x-front_caster_x)/(front_caster_y_minus-rear_caster_y_minus))) # th=0 for Ld250, becasue casters are inline




    # calc combined cg (platform and payload)
    combined_cg_x = ((platform_mass*platform_cg_x)+(payload_mass*payload_cg_x))/(platform_mass+payload_mass)
    combined_cg_y = ((platform_mass*platform_cg_y)+(payload_mass*payload_cg_y))/(platform_mass+payload_mass)
    combined_cg_z = ((platform_mass*platform_cg_z)+(payload_mass*payload_cg_z))/(platform_mass+payload_mass)

    # calc accel_max
    # calc m_fsp2 (spring_force_moment_2)
    spring_force = (87+4.364*(150.7-120.7-2.7))*280/160	# new spring N
    spring_force_moment_2_right = -spring_force*(rear_caster_y_plus-drive_wheel_y) #  [N-mm], moment about front casters due to combines weight of cart and payload that is normal to the inclined plane
    spring_force_moment_2 = 2 * spring_force_moment_2_right #  [N-mm]
    # calc m_w2
    weight_force = (platform_mass+payload_mass)*9.81 # total weight force
    weight_moment_2 = weight_force*(rear_caster_y_plus-combined_cg_y)*math.cos(phi) # N-mm for accel going up an incline (caster wheels in front of mounting pt)
    # calc accel_max [mm/s^2]
    accel_max = -(spring_force_moment_2+weight_moment_2)/((platform_mass+payload_mass)*(combined_cg_z))-(9810*math.sin(phi))

    # calc max_decel [mm/s^2]. moment about front casters due to the drive wheel spring for decel going down an incline:
    # calc m_fsp1 (spring force moment 2)
    spring_force_moment_1_right = -spring_force*(front_caster_y_minus-drive_wheel_y) # N-mm
    spring_force_moment_1 = 2*spring_force_moment_1_right # N-mm
    # calc m_w1 (weight moment 1)
    weight_moment_1 = weight_force*(front_caster_y_minus-combined_cg_y)*math.cos(phi) # N-mm for accel going up an incline (caster wheels in front of mounting pt)
    # calc decel_max
    decel_max =  (spring_force_moment_1+weight_moment_1)/((platform_mass+payload_mass)*(combined_cg_z))-(9810*math.sin(phi))

    # calc velocity_max

    logger.debug(f'accel_max: {accel_max}')
    logger.debug(f'decel_max: {decel_max}')
    pass


def iterate_all_payload_positions():
    # NOT CURRENTLY USED
    # go through all x, y, z locations, calculate viability
    x_vals = range(-480, 490, 10)  # width (transverse) of LD, in 10 mm increments
    y_vals = range(0, 328, 10)  # length (longitudinal) of LD, in 10mm increments
    z_vals = range(0, 101, 50)  # not sure if this is needed yet, just experimenting with looping
    # for i, (x_val, y_val) in enumerate(product(x_vals, y_vals)):
        # print(f'{i} | x:{x_val} | y:{y_val}')
    for i, x_val in enumerate(x_vals):
        for j, y_val in enumerate(y_vals):
            # now we have all x, y, z values available at once.

            # look at transverse tipping on ramp, no rotation

            # look at radial acceleration added

            # look at drive wheels added
            pass
