import logging
from matplotlib import pyplot as plt, patches
from math import ceil

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s : %(levelname)s : %(name)s : %(message)s')
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)

def zone_creation_ld(
    trans_decel: int, 
    max_vel: int,
    vel_subdiv_cnt: int,
    trans_safety_factor: float,
    payload_extensions: dict,
    platform_dimensions: dict) -> list:
    """
    Use kinematic equation to find displacement, given vf, v0, a.

    Parameters:
        trans_decel (int): Robot's stopping capability. Expressed in [mm/s^2]. Number will be negated to express negative acceleration.
        velocities list(int): Initial velocity for each safety zone. Expressed in [mm/s]
        trans_safety_factor (float): How much safety factor to include in calculation
        payload_extension (dict): Contains how much the payload extends outside the platform on each side.

    Returns:
        list(float): list of distances denoting each safety zone

    """
    # calculate list of velocities from max speed and number of safety zone subdivisions
    velocities = calculate_safety_zone_velocities(max_vel, vel_subdiv_cnt)

    # Use kinematic derivation to solve for displacement
    # equation explained in Onenote sheet: "Understanding 1D kinematics"
    safety_zone_lengths = [(-(v0 ** 2)/(2 * -trans_decel)) * trans_safety_factor for v0 in velocities]
    logger.debug(f'safety zone lengths: {safety_zone_lengths}')

    # adjust safety zone for payload extension, front and sides
    # Method will need to be modified to include MD and HD
    # create a list of rectangle safety zones, in [width, length] format
    safety_zones_adjusted_front_sides = [
        [payload_extensions["left"] + payload_extensions["right"] + platform_dimensions["width"],
        x + payload_extensions['front']]
        for x in safety_zone_lengths]
    logger.debug(f'safety zone dimensions, adjusted: {safety_zones_adjusted_front_sides}')

    return safety_zones_adjusted_front_sides

def calculate_safety_zone_velocities(max_velocity: int, vel_subdiv_cnt: int) -> list:
    """
    Use the max velocity to subdivide for each safety zone's velocity setting.
    
    Parameters:
        max_velocity (int): Max velocity of the robot in [mm/s], taking into account mass and CG from prior CG phase.
        
    Returns:
        list(float): 6 velocites in [mm/s] in a list for each zone.
    """
    logger.debug(f'calc\'ing safety zone velocities with max speed [{max_velocity}] and subdiv cnt [{vel_subdiv_cnt}]')
    delta = max_velocity / vel_subdiv_cnt
    safety_zone_velocities = [delta * (x+1) for x in range(vel_subdiv_cnt)]
    logger.debug(f'safety zones vels: {safety_zone_velocities}')
    return safety_zone_velocities

def plot_ld_safety_zones(ld_safety_zones: list) -> None:
    fig, ax = plt.subplots()

    # show spines at origin, not surrounding the plot
    ax.spines['left'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['bottom'].set_position('zero')
    ax.spines['top'].set_color('none')

    plt.style.use('seaborn-v0_8')

    ax.set_title('LD Safety Zones [mm]')
    # ax.set_ylabel('kenjs')
    # ax.set_xlabel('borks')
    xlim = ceil(ld_safety_zones[0][0] / 1000) * 1000
    ax.set_xlim([-xlim, xlim])
    ylim_low = -100  # arbitrarily chosen for looks
    ylim_high = ceil(ld_safety_zones[-1][1] / 1000) * 1000
    ax.set_ylim([ylim_low, ylim_high])
    # ax.legend()

    rects = [patches.Rectangle(xy=(-zone[0]/2,0), width=zone[0], height=zone[1], color='r', fill=False, linewidth=2) for zone in ld_safety_zones]
    for x in range(len(rects)):
        ax.add_patch(rects[x])
        ax.text(x=ld_safety_zones[x][0]/2 + 50, y=ld_safety_zones[x][1], s=f'Safety zone {x}: {ld_safety_zones[x][1]}', color='r')

    plt.tight_layout()
    plt.show()