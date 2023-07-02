import numpy as np
import navpy as nv
from drone import Drone
from path_generator import *
from util import densify_curve, point_to_parameter, parameter_to_index
from simple_pid import PID
import time
np.set_printoptions(precision=2)

lim = 2.0

def main():
    drone = Drone("tcp:127.0.0.1:5762", 0)
    drone.connect()
    while not drone.vehicle.home_location:
        cmds = drone.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not drone.vehicle.home_location:
            print(" Waiting for home location ...")

    lat_ref, lon_ref, alt_ref = drone.vehicle.home_location.lat, drone.vehicle.home_location.lon, drone.vehicle.home_location.alt

    path = circular_path(radius = 100.0, center=np.array([0, 0, 10]), num_points=360)
    dense_path = path # densify_curve(path, 360)

    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(5)
    control_rate = 50.0 # Hz
    control_period = 1 / control_rate # sec
    parameter = 0.0
    d_param = 0.075
    v_lim = 1.0
    k_p, k_d, k_i = 0.75, 0.2, 0.05
    pid_x = PID(k_p, k_d, k_i, setpoint=0, sample_time=control_period, output_limits=(-v_lim, v_lim))
    pid_y = PID(k_p, k_d, k_i, setpoint=0, sample_time=control_period, output_limits=(-v_lim, v_lim))
    pid_z = PID(k_p, k_d, k_i, setpoint=0, sample_time=control_period, output_limits=(-v_lim, v_lim))
    try:
        while parameter < 1.0:
            loop_start = time.time()
            pose_lla = drone.get_position_lla()
            pose_ned = nv.lla2ned(pose_lla[0], pose_lla[1], pose_lla[2], lat_ref, lon_ref, alt_ref)
            pose_ned[2] = pose_lla[2]
            drone_parameter, curve_index_min_dis, dis_min = point_to_parameter(dense_path, pose_ned)
            # dr = dense_path[curve_index_min_dis] - dense_path[parameter_to_index(parameter, dense_path)] # taking projection on curve  edge case found, if projected point on curve and parameter point is same, erroe is 0
            pi = parameter_to_index(parameter, dense_path)
            tgt_index = pi + 1 if pi < dense_path.shape[0] else pi
            dr = dense_path[tgt_index] - pose_ned

            dx, dy, dz = dr [0], dr[1], dr[2]
            error = np.sqrt(dx**2 + dy**2 + dz**2)


            error_coefficient = np.exp(-abs(error - lim)) if error >=lim else 1 
            inc = d_param * error_coefficient
            if abs(parameter - drone_parameter) < 0.1:
                parameter += inc

            vx, vy, vz = pid_x(-dx), pid_y(-dy), pid_z(dz)
            # vx, vy, vz = np.clip(vx, -v_lim, v_lim), np.clip(vy, -v_lim, v_lim), np.clip(vz, -v_lim, v_lim)
            drone.send_ned_velocity(vx, vy, vz)
            loop_end = time.time()
            dt_loop = loop_end - loop_start
            if  dt_loop < control_period:
                time.sleep(control_period - dt_loop)
            else:
                print(f" Control Loop exceeds {control_period * 1000} ms")
            print(f"{drone_parameter=:.2f} | {parameter=:.2f} | {error=:.2f} | {error_coefficient=:.2f} | {inc=:.2f} | {dx=:.2f} | {dy=:.2f} | {dz=:.2f} | {vx=:.2f} | {vy=:.2f} | {vz=:.2f} ")
            # print(f"{pose_lla=} | {pose_ned=} | {parameter=:.2f} | {drone_parameter=:.2f} | {dis_min=:.2f} | {dr=} | {error=:.2f} | ")
        drone.send_ned_velocity(0, 0, 0)

    except Exception as error:
        print(error)
    finally:
        drone.vehicle.close()
        print("vehicle closed")
if __name__ == '__main__':
    main()