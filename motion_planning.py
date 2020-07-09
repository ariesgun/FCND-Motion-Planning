import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from pruning import prune_path
from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, helix):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.perform_helix = helix
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        # DONE: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as fd:
            line = fd.readline().split(',')
            lat0, lon0 = (float(line[0].split()[1]), float(line[1].split()[1]))
        
        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        
        # DONE: retrieve current global position
        (glob_lon, glob_lat, glob_alt) = self.global_position

        # DONE: convert to current local position using global_to_local()
        # print(global_to_local(self.global_position, self.global_home))
        ## which is the same as self.local_position
        local_position = global_to_local(self.global_position, self.global_home)
        loc_north, loc_east = (int(np.ceil(local_position[0])), int(np.ceil(local_position[1])))
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # DONE: convert start position to current position rather than map center
        grid_start = (loc_north - north_offset, loc_east - east_offset)
        
        # Extra: Draw circle waypoints
        if self.perform_helix:
            n_s = grid_start[0]
            e_s = grid_start[1]
            pruned_path = [(n_s, e_s),
                           (n_s + 6, e_s - 3),
                           (n_s + 3, e_s - 6),
                           (n_s - 3, e_s - 6),
                           (n_s - 6, e_s - 3),
                           (n_s - 6, e_s + 3),
                           (n_s - 3, e_s + 6),
                           (n_s + 3, e_s + 6),
                           (n_s + 6, e_s + 3),
                           (n_s + 6, e_s - 3),
                           (n_s + 3, e_s - 6),
                           (n_s - 3, e_s - 6),
                           (n_s - 6, e_s - 3),
                           (n_s - 6, e_s + 3),
                           (n_s - 3, e_s + 6),
                           (n_s + 3, e_s + 6),
                           (n_s + 6, e_s + 3),
                           (n_s + 6, e_s - 3)]

            # Convert path to waypoints
            waypoints = []
            for i, p in enumerate(pruned_path):
                wp = [p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0]
                if len(waypoints) == 0:
                    # First waypoint
                    prev_wp = wp
                else:
                    wp[3] = np.arctan2((wp[1]-prev_wp[1]), (wp[0]-prev_wp[0]))
                    wp[2] = prev_wp[2] + 1
                    prev_wp = wp

                waypoints += [wp]

        else:
            self.target_position[2] = TARGET_ALTITUDE

            # Set goal as some arbitrary position on the grid
            #grid_goal = (-north_offset + 10, -east_offset + 10)
            # DONEdapt to set goal as latitude / longitude position and convert
            goal_lon = -122.397134
            goal_lat = 37.792928
            goal_position = global_to_local((goal_lon, goal_lat, 0), self.global_home)
            goal_north = int(np.ceil(goal_position[0]))
            goal_east  = int(np.ceil(goal_position[1]))
            grid_goal = (goal_north - north_offset, goal_east - east_offset)

            # Run A* to find a path from start to goal
            # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
            # or move to a different search space such as a graph (not done here)
            print('Local Start and Goal: ', grid_start, grid_goal)
            path, _ = a_star(grid, heuristic, grid_start, grid_goal)

            # DONE: prune path to minimize number of waypoints
            pruned_path = prune_path(path)
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
            
        # Set self.waypoints
        print(waypoints)
        self.waypoints = waypoints
        # DONE: send waypoints to sim (this is just for visualization of waypoints)
        print("Waypoints Count {}".format(len(self.waypoints)))
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--helix', type=bool, default=True, help="Perform Helix plan")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, args.helix)
    time.sleep(1)

    drone.start()
