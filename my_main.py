import asyncio
import time
import math
from geographiclib.geodesic import Geodesic
from mavsdk import System
from mavsdk.telemetry import (
    Odometry,
    PositionBody,
    VelocityBody,
    AngularVelocityBody,
    Position,
)


class Drone:
    def clip(self, minval, maxval, value):
        return min(max(value, minval), maxval)

    def __init__(self, is_sim):
        self.yaw_step = 4
        self.yaw_substep = 3
        self.max_rate_in_hz = 2
        self.max_yaw = int(self.max_rate_in_hz * 360)
        self.yaw_change_rate = self.max_yaw / self.yaw_step
        self.yaw_step_change_t = 2
        self.yaw_substp_change_t = 1
        self.yaw_descend_step_change_t = 0.25
        self.yaw_rate_setpoint = 0
        self.gps_position = Position(0, 0, 0, 0)
        self.odometry = Odometry(0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.position_body = PositionBody(0, 0, 0)
        self.velocity_body = VelocityBody(0, 0, 0)
        self.angular_vel_body = AngularVelocityBody(0, 0, 0)
        if is_sim == 1:
            self.target_latitude = 47.3976262
            self.target_longitude = 8.5458304
            self.system_addr = "udp://:14540"
        else:
            self.target_latitude = 39.87218987
            self.target_longitude = 32.73215061
            self.system_addr = "serial:///dev/ttyS0:921600"

        self.home = Position(0, 0, 0, 0)

    async def run(self):
        # Init the drone
        drone = System()
        await drone.connect(system_address=self.system_addr)
        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

        # Start the tasks
        async for pos in drone.telemetry.home():
            self.home = pos
            break

        self.brng = self.calculate_bearing(
            self.home.latitude_deg,
            self.home.longitude_deg,
            self.target_latitude,
            self.target_longitude,
        )
        await drone.action.set_takeoff_altitude(3)

        print("-- Arming")
        await drone.manual_control.set_manual_control_input(0, 0, 0.5, 0)
        await drone.action.arm()
        await drone.action.takeoff()
        await asyncio.sleep(8)

        await drone.telemetry.set_rate_odometry(20)
        await drone.telemetry.set_rate_position(20)

        flying_alt = self.home.absolute_altitude_m + 3

        await drone.action.goto_location(
            self.target_latitude, self.target_longitude, flying_alt, self.brng
        )
        await asyncio.sleep(10)

        await drone.manual_control.set_manual_control_input(
            0,
            0,
            0.5,
            0,
        )
        await drone.manual_control.start_position_control()

        step_size = int(self.max_yaw / self.yaw_step)
        sub_step_size = int(step_size / self.yaw_substep)
        print(sub_step_size)
        steps = []


        self.brng = self.calculate_bearing(
            self.gps_position.latitude_deg,
            self.gps_position.longitude_deg,
            self.home.latitude_deg,
            self.home.longitude_deg,
        )

        timer = time.time()
        while time.time() - timer <= 3:
            await drone.manual_control.set_manual_control_input(
                0,
                0,
                0.5,
                0,
            )
            await asyncio.sleep(0.05)

        await drone.action.goto_location(
            self.home.latitude_deg,
            self.home.longitude_deg,
            flying_alt,
            self.brng,
        )
        await asyncio.sleep(10)
        await drone.action.land()
        await asyncio.sleep(10)

    def calculate_bearing(
        self,
        lat1,
        lon1,
        lat2,
        lon2,
    ):
        angle = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)["azi1"]
        return angle


 

    def radians_to_degrees(self, value):
        value_in_degrees = value * 180 / math.pi
        return value_in_degrees


if __name__ == "__main__":
    # Run the asyncio loop
    controller = Drone(0)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(controller.run())
