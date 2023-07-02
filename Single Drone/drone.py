import time
from typing import Optional
import navpy as nv
import numpy as np
from dronekit import CommandSequence, LocationGlobalRelative, Vehicle, connect  
from pymavlink import mavutil # Needed for velocity command message definitions

def wait_while(method, delay=0.2):
    import time

    bar = [
        " [=     ]",
        " [ =    ]",
        " [  =   ]",
        " [   =  ]",
        " [    = ]",
        " [     =]",
        " [    = ]",
        " [   =  ]",
        " [  =   ]",
        " [ =    ]",
    ]
    i = 0

    while method():
        # print(bar[i % len(bar)], end="\r")
        time.sleep(delay)
        i += 1

class Drone:
    def __init__(self, connection_string: str, index: int):
        self.connection_string = connection_string

        self.vehicle: Optional[Vehicle] = None

        self.in_air: bool = False
        self.reached: bool = True
        self.distance_to_wp: float = 0.0
        self.wp_radius: float = 1.0

        self.index: int = index

    def connect(self):
        try:
            print("Connecting to vehicle :", self.connection_string)
            self.vehicle = connect(self.connection_string, wait_ready=True)
        except Exception as e:
            print(self.index, " ->", " can't connect")
            print(e)
            
            
    def disconnect(self):
        try:
            if self.vehicle is not None and self.connected:

                print("Disonnecting from vehicle :", self.connection_string)
                self.vehicle.close()

        except Exception as e:
            print(self.index, " ->", " can't disconnect")
            print(e)

    @property
    def connected(self) -> bool:
        # in future explore self.vehicle.heartbeat
        #print(f"Drone {self.mavID} :  last heartbeat {self.vehicle.last_heartbeat} seconds ago")
        return self.vehicle is not None and self.vehicle.last_heartbeat < 1.5

    @property
    def mavID(self) -> Optional[int]:
        if self.vehicle is not None:
            return int(self.vehicle.parameters["SYSID_THISMAV"]) - 1
        else:
            return None

    @property
    def armed(self):
        if self.vehicle is not None:
            return self.vehicle.armed
        else:
            return False

    def set_gcs_failsafe(self):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        gcs_fs = self.vehicle.parameters["FS_GS_ENABLE"]
        if gcs_fs != 1.0:
            print("WARNING: GCS failsafe not set to RTL")
            print("Setting GCS failesafe...")
            self.vehicle.parameters["FS_GS_ENABLE"] = 1.0
            self.vehicle.wait_for(
                lambda: self.vehicle.parameters["FS_GS_ENABLE"] == 1.0
            )
            print("Setting GCS failesafe... -> DONE")
        else:
            print("GCS failsafe set to RTL")

    def arm(self):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        self.vehicle.wait_for_armable()
        self.vehicle.arm()
        
    def disarm(self):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        self.vehicle.disarm()
        #self.vehicle.wait_for(not self.vehicle.armed)!!!!!
        
    def set_mode(self, mode: str):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        print(self.index, " -> Switching to ", mode)
        self.vehicle.wait_for_mode(mode)

    def get_position_lla(self):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        return np.asarray(
            [
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon,
                self.vehicle.location.global_relative_frame.alt,
            ]
        )


    def get_position_ned(self) :
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        return np.asarray(
            [
                self.vehicle.location.local_frame.north,
                self.vehicle.location.local_frame.east,
                self.vehicle.location.local_frame.down,
            ]
        )

    def takeoff(self, tgt_altitude: float = 10, epsilon: float = 0.2):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")


        # self.set_mode("GUIDED")
        if self.vehicle.system_status.state is "ACTIVE":
            print(self.index, " ->", self.vehicle.system_status.state)
            return
        self.vehicle.simple_takeoff(tgt_altitude)
        print(self.index, " -> Takeoff")
        self.vehicle.wait_for_alt(tgt_altitude, epsilon)
        print(self.index, " -> reached target altitude!")


    def goto(self, waypoint_lla):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        self.vehicle.wait_for(lambda: self.vehicle.mode.name == "GUIDED")

        lon, lat, alt = waypoint_lla
        # Remove `abs` later
        waypoint = LocationGlobalRelative(lon, lat, abs(alt))

        self.reached = False

        print(self.index, " -> Going to waypoint : ", waypoint)
        self.vehicle.simple_goto(waypoint, groundspeed=20)

        # TODO: Idk wtf this is doing
        while True:
            lat, lon, alt = self.get_position_lla()
            delta_lat = abs(lat - waypoint.lat)
            delta_long = abs(lon - waypoint.lon)
            delta_alt = abs(alt - waypoint.alt)
            horizontal_distance = (
                np.sqrt((delta_lat * delta_lat) + (delta_long * delta_long))
                * 1.113195e5
            )
            if horizontal_distance <= self.wp_radius and delta_alt <= self.wp_radius:
                self.reached = True
                self.distance_to_wp = horizontal_distance
                print(self.index, " -> Destination Reached")
                break
            else:
                # print(self.index, " -> Distance : ", distance, end ='\r')
                self.distance_to_wp = horizontal_distance
                pass
            time.sleep(0.1)  # may check later

    def comeback(self, mode: str = "RTL"):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        print(self.index, " -> Coming back to LAND")
        self.set_mode(mode)

        wait_while(lambda: self.armed, 1.0)  # condition, interval
        print(mode, "complete")



    def spline_goto(self, commands: CommandSequence):
        if self.vehicle is None:
            raise Exception("Vehicle not connected!")

        # self.set_mode("GUIDED")

        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        print("mission items downloaded")

        print("count : ", cmds.count)
        cmds.clear()
        print("count : ", cmds.count)

        for cmd in commands:
            cmds.add(cmd)

        for cmd in cmds:
            print(cmd)

        cmds.upload()

        self.set_mode("AUTO")

        while self.vehicle.commands.next != cmds.count:
            print("Next Waypoint: ", self.vehicle.commands.next)
            time.sleep(2)


    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.
        By default the yaw of the vehicle will follow the direction of travel. After setting 
        the yaw using this function there is no way to return to the default yaw "follow direction 
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
        For more information see: 
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,          #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        
        
    def send_global_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        print(self.index, " -> setting GLOBAL Velocity : ", velocity_x, velocity_y, velocity_z, duration)

        """
        Move vehicle in direction based on specified velocity vectors.
        This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b110111000111 , # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 


        # send command to vehicle on freq Hz cycle
        freq = 100
        for x in range(0,duration*freq):
            self.vehicle.send_mavlink(msg)
            time.sleep(1/freq)  

    """
    Functions that move the vehicle by specifying the velocity components in each direction.
    The two functions use different MAVLink commands. The main difference is
    that depending on the frame used, the NED velocity can be relative to the vehicle
    orientation.
    The methods include:
    * send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
    * send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
    """

    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration=None):
        # print(self.index, " -> setting NED Velocity : ", velocity_x, velocity_y, velocity_z, duration)
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.
        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b110111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        
        self.vehicle.send_mavlink(msg)
        return
        '''
        # send command to vehicle on freq Hz cycle
        start_time = time.time()
        freq = 100.0
        period = 1 / freq
        # print(f" Frequency : {freq} Hz | Period : {period} s")
        while (time.time() - start_time) < duration:
            d1 = time.time()
            ##########elf.vehicle.send_mavlink(msg) ###############

            d2 = time.time()
            dt = d2 - d1
            if dt < period:
                if (time.time() - start_time - period + dt) > duration:
                    break
                else:
                    time.sleep(period - dt)
            else:
                print("Rate failed")
            # print(f"Duration : {duration} | D_time : {time.time() - start_time} | dt : {dt} | Sleep time : {period - dt}")

        for x in range(0,duration*freq):
            self.vehicle.send_mavlink(msg)
            time.sleep(1/freq)  
        '''

    #-- Define the function for sending mavlink velocity command in body frame
    def set_velocity_body(self, velocity_x=0, velocity_y=0, velocity_z=0, yaw_rate=0,duration=0):
        print(self.index, " -> setting NED Velocity : ", velocity_x, velocity_y, velocity_z, duration)

        """ Remember: vz is positive downward!!!
        http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        
        Bitmask to indicate which dimensions should be ignored by the vehicle 
        (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
        none of the setpoint dimensions should be ignored). Mapping: 
        bit 1: x,  bit 2: y,  bit 3: z, 
        bit 4: velocity_x, bit 5: velocity_y, bit 6: velocity_z, 
        bit 7: ax, bit 8: ay, bit 9:
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                1479  , #-- BITMASK -> Consider only the velocities
                0, 0, 0,        #-- POSITION
                velocity_x, velocity_y, velocity_z,     #-- VELOCITY
                0, 0, 0,        #-- ACCELERATIONS
                0, yaw_rate)
        # send command to vehicle on freq Hz cycle
        freq = 100
        dot = '.'
        for x in range(0,duration*freq):
            self.vehicle.send_mavlink(msg)
            time.sleep(1/freq)  
            self.vehicle.flush()
            print(dot,end='\r')
            dot += '.'
        print(dot)

if __name__ == "__main__":
    drone = Drone("tcp:127.0.0.1:5762", 0)

    drone.connect()

    while not drone.vehicle.home_location:
        cmds = drone.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not drone.vehicle.home_location:
            print(" Waiting for home location ...")
    lat_ref, lon_ref, alt_ref = drone.vehicle.home_location.lat, drone.vehicle.home_location.lon, drone.vehicle.home_location.alt
    pose_lla = drone.get_position_lla()
    print(drone.get_position_ned() -  nv.lla2ned(pose_lla[0], pose_lla[1], pose_lla[2], lat_ref, lon_ref, alt_ref))

    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(5)
    drone.send_ned_velocity(10, 0, 0, 10)
    drone.send_ned_velocity(0, 0, 0, 2)

    time.sleep(1)

    while not drone.vehicle.home_location:
        cmds = drone.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not drone.vehicle.home_location:
            print(" Waiting for home location ...")
    lat_ref, lon_ref, alt_ref = drone.vehicle.home_location.lat, drone.vehicle.home_location.lon, drone.vehicle.home_location.alt
    pose_lla = drone.get_position_lla()
    print(drone.get_position_ned() -  nv.lla2ned(pose_lla[0], pose_lla[1], pose_lla[2], lat_ref, lon_ref, alt_ref) - [0.5, -0.0025 , 0.0])

    drone.send_ned_velocity(0, 10, 0, 20)
    drone.send_ned_velocity(0, 0, 0, 2)
    time.sleep(1)

    while not drone.vehicle.home_location:
        cmds = drone.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not drone.vehicle.home_location:
            print(" Waiting for home location ...")
    lat_ref, lon_ref, alt_ref = drone.vehicle.home_location.lat, drone.vehicle.home_location.lon, drone.vehicle.home_location.alt
    pose_lla = drone.get_position_lla()
    print(drone.get_position_ned() -  nv.lla2ned(pose_lla[0], pose_lla[1], pose_lla[2], lat_ref, lon_ref, alt_ref) - [0.5, -0.0025 , 0.0]) 
    #drone.send_ned_velocity(1,0,0,5)
    #drone.send_ned_velocity(0,1,0,5)
    #drone.send_ned_velocity(0,-1,0,5)
    #drone.send_ned_velocity(-1,0,0,5)
    #drone.send_ned_velocity(0,0,-1,5)
    drone.comeback()
    drone.vehicle.close()
    print("vehicle closed")
    while not drone.vehicle.home_location:
        cmds = drone.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not drone.vehicle.home_location:
            print(" Waiting for home location ...")
    lat_ref, lon_ref, alt_ref = drone.vehicle.home_location.lat, drone.vehicle.home_location.lon, drone.vehicle.home_location.alt
    pose_lla = drone.get_position_lla()
    print(drone.get_position_ned() -  nv.lla2ned(pose_lla[0], pose_lla[1], pose_lla[2], lat_ref, lon_ref, alt_ref) - [0.5, -0.0025 , 0.0])
    