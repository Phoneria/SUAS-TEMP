import asyncio

from mavsdk import System
import time

async def px4_connect_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            return drone


async def run_drone(drone, plan_name):

    mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission(
        plan_name
    )
    print(f"{len(mission_import_data.mission_items)} mission items imported")
    await drone.mission_raw.upload_mission(mission_import_data.mission_items)
    print("Mission uploaded")

    await drone.action.arm()
    await drone.mission.start_mission()


    async for mission_progress in drone.mission_raw.mission_progress():
        print("mission progress: " f"{mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == len(mission_import_data.mission_items):
            print("Finished")
            break



async def run():
    drone = await px4_connect_drone()
    await run_drone(drone,"mainmission.plan")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
