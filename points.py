from points_utils import Points, STEPS_PER_CM, MotorParams
from points_sim import simulate_points
from tqdm import tqdm
import numpy as np
from typing import List
import math


positions1 = [0, 0, 10 * STEPS_PER_CM]
positions2 = [40 * STEPS_PER_CM, 60 * STEPS_PER_CM, 50 * STEPS_PER_CM]
positions3 = [40 * STEPS_PER_CM, 40 * STEPS_PER_CM, 40 * STEPS_PER_CM]
# speeds3 = [8, 8, None]
# positions4 = [curX, curY, curX + 1]
# positions4 = [6 * STEPS_PER_CM, 6 * STEPS_PER_CM, 6 * STEPS_PER_CM] # THis should be changed with speeds
# positions5 = [6 * STEPS_PER_CM, 6 * STEPS_PER_CM, 7 * STEPS_PER_CM] # THis should then be deleted
# positions6 = [10 * STEPS_PER_CM, 1 * STEPS_PER_CM, 11 * STEPS_PER_CM]
# positions7 = [10 * STEPS_PER_CM, 1 * STEPS_PER_CM, 10.5 * STEPS_PER_CM]
# positions8 = [10 * STEPS_PER_CM, 1 * STEPS_PER_CM, 11 * STEPS_PER_CM]
# positions9 = [0 * STEPS_PER_CM, 0 * STEPS_PER_CM, 1 * STEPS_PER_CM]

all_positions = [positions1, positions2]

x_poss = []
y_poss = []
z_poss = []

x_pos = 0
y_pos = 0
z_pos = 0

x_vols = {}
y_vols = {}
z_vols = {}

x_vol = 0
y_vol = 0
z_vol = 0

time = 0 # in milliseconds

MAX_SPEED = 200.0 # steps/s
MAX_ACCELERATION = 20.0 # steps/s^2
MIN_SPEED = 3.0 # steps/s
DIMS = 3
CMS_PER_STEPS = 1 / STEPS_PER_CM
EPSILON = 0.0001

def steps_by_time(speed, end_speed, time):
    distance = 0.5 * (speed + end_speed) * time
    return int(distance)

def time_to_reach_speed(speed, start_speed, acceleration):
    return (speed - start_speed) / acceleration

def steps_to_reach_speed(speed, start_speed, acceleration):
    time = (speed - start_speed) / acceleration
    distance = 0.5 * (speed + start_speed) * time
    return int(distance)

def get_step_kinds(total_steps, max_speed, min_speed, acceleration, current_speed, init_steps_to_hold):
    result = []
    if current_speed > max_speed:
        steps_to_brake_init = steps_to_reach_speed(max_speed, current_speed, -acceleration)
        result.extend([-1] * steps_to_brake_init)
        steps_to_brake = steps_to_reach_speed(min_speed, max_speed, -acceleration)
        result.extend([0] * (total_steps - steps_to_brake_init - steps_to_brake))
        result.extend([-1] * steps_to_brake)
    else:
        result.extend([0] * init_steps_to_hold)
        total_steps -= init_steps_to_hold
        steps_to_max = steps_to_reach_speed(max_speed, current_speed, acceleration)
        steps_to_brake = steps_to_reach_speed(min_speed, max_speed, -acceleration)
        # print(steps_to_max, steps_to_brake, current_speed, min_speed)
        if total_steps < steps_to_max + steps_to_brake:
            ratio_to_accelerate = steps_to_max / (steps_to_max + steps_to_brake)
            # print(total_steps, math.ceil(total_steps * ratio_to_accelerate), math.ceil(total_steps * (1 - ratio_to_accelerate)))
            result.extend([1] * math.ceil(total_steps * ratio_to_accelerate))
            if total_steps % 2 == 1:
                result.append(0)
            result.extend([-1] * math.ceil(total_steps * (1 - ratio_to_accelerate)))
        else:
            result.extend([1] * steps_to_max)
            result.extend([0] * (total_steps - steps_to_max - steps_to_brake))
            result.extend([-1] * steps_to_brake)
    # print(result)
    return result

def calculate_motor_params(steps, current_speeds) -> List[MotorParams]:
    # max_steps = max(steps)
    if not any(current_speeds[i] <= MIN_SPEED for i in range(DIMS)):
        raise NotImplementedError("This should not happen")
    steps_to_hold = [0] * DIMS
    if all(current_speeds[i] == 0 for i in range(DIMS)):
        new_steps = steps
    else:
        zero_speed_index = max([(i, steps[i]) for i in range(DIMS) if current_speeds[i] == 0], key=lambda x: x[1])[0]
        new_steps = []
        # zero_speed_time_to_reach_max = time_to_reach_speed(MAX_SPEED, current_speeds[zero_speed_index], MAX_ACCELERATION)
        # zero_speed_steps_to_reach_max = steps_to_reach_speed(MAX_SPEED, current_speeds[zero_speed_index], MAX_ACCELERATION)
        for i in range(DIMS):
            if i == zero_speed_index or current_speeds[i] == 0:
                new_steps.append(steps[i])
            else:
                zero_speed_time_to_reach_cur_speed = time_to_reach_speed(current_speeds[i], current_speeds[zero_speed_index], MAX_ACCELERATION)
                zero_speed_steps_to_reach_cur_speed = steps_to_reach_speed(current_speeds[i], current_speeds[zero_speed_index], MAX_ACCELERATION)
                steps_to_hold_before_catchup = steps_by_time(current_speeds[i], current_speeds[i], zero_speed_time_to_reach_cur_speed)
                steps_to_hold[i] = steps_to_hold_before_catchup
                diff = steps_to_hold_before_catchup - zero_speed_steps_to_reach_cur_speed
                new_steps.append(steps[i] - diff)
    
    max_steps = max(new_steps)

    result = []
    for i in range(DIMS):
        ratio = new_steps[i] / max_steps
        result.append(MotorParams(MAX_SPEED * ratio, MAX_ACCELERATION * ratio, steps_to_hold[i], 0))
    return result

def calculate_corner_step(steps):
    # steps_to_max = steps_to_reach_speed(MAX_SPEED, 0, MAX_ACCELERATION)
    # ratio = (max([abs(x) for x in steps]) - steps_to_max) / max([abs(x) for x in steps])
    # print(ratio, steps_to_max, max([abs(x) for x in steps]))
    # return [int(steps[i] * ratio) for i in range(DIMS)]
    return [int(steps[i] * 0.5) for i in range(DIMS)]

def linear_interpolation_speeds(start_array, end_array, max_speed = MAX_SPEED, acceleration = MAX_ACCELERATION, should_corner=True):
    current_speeds = [x_vol, y_vol, z_vol]
    # print(leftover_from_last_interpolation)
    steps = [max(abs(end_array[i] - start_array[i]) - 1, 0) for i in range(DIMS)]
    print(steps)
    print([max(abs(end_array[i] - start_array[i]) - 1, 0) for i in range(DIMS)])
    corner_step = calculate_corner_step(steps)
    results = [[] for _ in range(DIMS)]
    motorParams = calculate_motor_params(steps, current_speeds)
    # leftover_from_this_interpolation = [0] * DIMS
    # print(start_array)
    # print(steps)
    for i in range(DIMS):
        # print("i", i)
        if steps[i] == 0:
            continue
        step_kinds = get_step_kinds(steps[i], motorParams[i].speed, MIN_SPEED, motorParams[i].acceleration, current_speeds[i], motorParams[i].init_steps_to_brake)
        print(len(step_kinds), steps[i])
        if current_speeds[i] < MIN_SPEED: current_speeds[i] = MIN_SPEED
        results[i].append(current_speeds[i])
        for j in range(steps[i]):
            # print(current_speeds[i])
            current_speeds[i] += step_kinds[j] * motorParams[i].acceleration * 1 / current_speeds[i]
            current_speeds[i] = max(min(current_speeds[i], MAX_SPEED), MIN_SPEED)
            update_speeds(current_speeds)
            results[i].append(current_speeds[i])
            if should_corner and j == corner_step[i]: 
                # leftover_from_this_interpolation[i] = steps[i] - j
                break

        # results[i].append(MIN_SPEED)
        # distance_in_accel = 0.5 * (max_speed + MIN_SPEED) * time_to_max

    # Here is what the function should do:
    # The start_array is an array with positions where each element is currently
    # The end array is where the positions should be after interpolation
    # Each array is of length DIMS and the difference in the arrays is how many steps you have to interpolate over
    # You should initialize the speeds to be MIN_SPEED and using ACCELERATION you should go towards MAX_SPEED, keep it there and go back
    # when you have to in order to be at MIN_SPEED when you have no more steps to go
    # You have to make sure that you start deaccelerating in time to reach the final points, if you cannot reach MAX_SPEED before doing so
    # that is okay
    # Before proceeding ask ANY questions you might have, and you can then proceed to implement the function
    return results

def update_positions(positions, dim):
    addition = lambda i: 1 if dim == i else 0
    for i in range(DIMS):
        positions[i].append(positions[i][-1] + addition(i))
    global x_pos, y_pos, z_pos
    x_pos = int(positions[0][-1])
    y_pos = int(positions[1][-1])
    z_pos = int(positions[2][-1])

def update_speeds(speeds):
    global x_vol, y_vol, z_vol
    x_vol = speeds[0]
    y_vol = speeds[1]
    z_vol = speeds[2]

def rollout_from_speeds(speeds):
    # for i in range(DIMS):
    #     speeds[i].append(EPSILON)
    # Initialize positions for each dimension
    global time

    vol_idxs = [0] * DIMS
    positions = [[x_pos], [y_pos], [z_pos]]
    new_speeds = [{time: speeds[i][0] if len(speeds[i]) > 0 else 0} for i in range(DIMS)]
    time_to_step = [time + 1000/new_speeds[i][time] if new_speeds[i][time] != 0 else np.inf for i in range(DIMS)]
    
    # Find the dimension that will take the longest time
    max_steps = sum(len(speed_list) for speed_list in speeds) - sum([1 for i in range(DIMS) if len(speeds[i]) > 0])
    current_steps = 0
        
    # Simulate with small time steps
    dt = 1  # 100ms time steps

    while current_steps < max_steps:
        # print(current_steps, max_steps)
        time += dt
        for dim in range(DIMS):
            if time_to_step[dim] > time:
                continue
            
            if vol_idxs[dim] >= len(speeds[dim]) - 1:
                speeds[dim].append(0)
                vol_idxs[dim] += 1
                time_to_step[dim] = np.inf
                # print("DIM: ", dim, "SPEEDS: ", len(speeds[dim]), "SPEED: ", speeds[dim][vol_idxs[dim]])
                # raise NotImplementedError("This should not happen")
            else:
                current_steps += 1
                vol_idxs[dim] += 1
                time_to_step[dim] = time_to_step[dim] + 1000/speeds[dim][vol_idxs[dim]]
            # print(time_to_step[dim], 1000/speeds[dim][vol_idxs[dim]]) 
            # current_steps += 1

            update_positions(positions, dim)
            for i in range(DIMS):
                if len(speeds[i]) == 0:
                    new_speeds[i][time] = 0
                else:
                    new_speeds[i][time] = speeds[i][vol_idxs[i]]
            
    # print("rollout")
    # print(vol_idxs, [len(x) for x in speeds])
    return positions, new_speeds

# def linear_interpolation(start_array, end_array):
#     steps = int(max([abs(start_array[i] - end_array[i]) for i in range(DIMS)]))
#     result = [[] for _ in range(DIMS)]
#     for j in range(DIMS):
#         for i in range(steps):
#             result[j].append(start_array[j] + i * (end_array[j] - start_array[j]) / steps)
#     return result

if __name__ == "__main__":
    # print(*z_vols.items(), sep="\n")
    # leftover = [0] * DIMS
    for i in range(len(all_positions)):
        positions = [int(x) for x in all_positions[i]]
        linear_interpolations = linear_interpolation_speeds([x_pos, y_pos, z_pos], positions, should_corner=i!=len(all_positions)-1)
        rollouts, new_speeds = rollout_from_speeds(linear_interpolations)
        x_vols.update(new_speeds[0]) # TODO: You need to account for the time when next cycle begins
        y_vols.update(new_speeds[1])
        z_vols.update(new_speeds[2])
        x_poss.extend(rollouts[0])
        y_poss.extend(rollouts[1])
        z_poss.extend(rollouts[2])
        # print(*z_vols.items(), sep="\n")

    points = Points(x_poss, y_poss, z_poss, x_vols, y_vols, z_vols, len(x_poss))
    # print(*list(points.vz.items()), sep="\n") 
    simulate_points(points)
    

