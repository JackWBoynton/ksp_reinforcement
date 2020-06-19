import time
import requests
import sys, io
import krpc
import math

import tensorflow.keras as keras

import gym
import random as rd
import tensorflow as tf

import numpy as np
import itertools

class KSPController(object): # object that controls the connection to the kOS terminal in KSP
    def __init__(self):

      self.conn = krpc.connect(name="rlagent")

      self.vessel = self.conn.space_center.active_vessel
      self.ref_frame = self.conn.space_center.ReferenceFrame.create_hybrid(
        position=self.vessel.orbit.body.reference_frame,
        rotation=self.vessel.surface_reference_frame)
      self.waypoint_manager = self.conn.space_center.waypoint_manager
      self.bodies = self.conn.space_center.bodies
      self.flight_info = self.vessel.flight(self.ref_frame)
      self.control_info = self.vessel.control
      self.control_info.input_mode = self.conn.space_center.ControlInputMode.override
      self.resources_info = self.vessel.resources
      self.parts = self.vessel.parts
      self.autopilot = self.vessel.auto_pilot # can set direction rather than just changes
      self.R = 600000 # mean radius of Kerbin
      # landing site waypoint
      self.waypoint_manager.add_waypoint(self.flight_info.latitude, self.flight_info.longitude, self.bodies["Kerbin"], "Landing Site")

    def reset_gamestate(self):
      self.conn.space_center.load("quicksave #3")
      time.sleep(1) # physics load
      self.vessel = self.conn.space_center.active_vessel
    
    def telemetry_stream(self):
      n = 0
      altitude = self.conn.add_stream(getattr, self.flight_info, 'surface_altitude')
      latitude = self.conn.add_stream(getattr, self.flight_info, 'latitude')
      longitude = self.conn.add_stream(getattr, self.flight_info, 'longitude')
      h_speed = self.conn.add_stream(getattr, self.flight_info, 'horizontal_speed')
      v_speed = self.conn.add_stream(getattr, self.flight_info, 'vertical_speed')
      #rot = self.conn.add_stream(getattr, self.flight_info, "rotation")
      #pointing = self.conn.add_stream(getattr, self.flight_info, "direction")

      #rel_pos = self.conn.add_stream(self.vessel.position(self.vessel.surface_reference_frame)) # -> x:zenith, y:north, z:east
      #rel_vel = self.conn.add_stream(self.vessel.velocity(self.vessel.surface_reference_frame)) # -> x:zenith, y:north, z:east

      #obt_normal = self.conn.add_stream(getattr, self.flight_info, "normal")
      #aero = self.conn.add_stream(getattr, self.flight_info, "aerodynamic_force")
      #aoa = self.conn.add_stream(getattr, self.flight_info, "angle_of_attack")
      #slid_slip_a = self.conn.add_stream(getattr, self.flight_info, "sideslip_angle")
      #drag_coeff = self.conn.add_stream(getattr, self.flight_info, "drag_coefficient")
      #lift_coeff = self.conn.add_stream(getattr, self.flight_info, "lift_coefficient")
      
      max_thrust = self.conn.add_stream(getattr, self.vessel, "max_thrust")
      mass = self.conn.add_stream(getattr, self.vessel, "mass")
      #pos = self.conn.add_stream(getattr, self.vessel, "surface_reference_frame")

      legs = self.conn.add_stream(getattr, self.control_info, "legs") # can be set
      throttle = self.conn.add_stream(getattr, self.control_info, "throttle") # can be set
      pitch = self.conn.add_stream(getattr, self.control_info, "pitch") # can be set
      yaw = self.conn.add_stream(getattr, self.control_info, "yaw") # can be set
      roll = self.conn.add_stream(getattr, self.control_info, "roll") # can be set

      fuel = self.conn.add_stream(self.resources_info.amount, "LiquidFuel")
       
      pitch_error = self.conn.add_stream(getattr, self.autopilot, "pitch_error")
      heading_error = self.conn.add_stream(getattr, self.autopilot, "heading_error")

      sit = self.vessel.situation
      while str(sit) != "VesselSituation.landed" and n < 1000:
        n += 1
        sit = self.vessel.situation
        long1 = math.radians(longitude())
        lat1 = math.radians(latitude())
        long2 = math.radians(self.waypoint_manager.waypoints[0].longitude)
        lat2 = math.radians(self.waypoint_manager.waypoints[0].latitude)

        # calc bearing of velocity:
        rel_vel = self.vessel.velocity(self.ref_frame)
        north_vel = rel_vel[1] 
        east_vel = rel_vel[2]

        vel_heading = math.atan2(east_vel, north_vel)
        vel_bearing = (vel_heading + 360) % 360 # -> 0 to 360 deg bearing

        # calculate the bearing between 2 (lat,long) tuples:
        y = math.sin(long2 - long1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)
        theta = math.atan2(y, x)
        bearing = (theta*180/math.pi + 360) % 360

        # dist delta in meters
        up, north, east = self.vessel.position(self.ref_frame)
        a = math.sin((lat2-lat1)/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin((long2-long1)/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = self.R * c

        # dist to waypoint in each direction (north, east) meters
        dx = d * math.cos(math.radians(bearing))
        dy = d * math.sin(math.radians(bearing))
        north_delta = np.dot(np.array([dx, dy]), np.array([1, 0])) # north comp of dist
        east_delta = np.dot(np.array([dx, dy]), np.array([0, 1])) # east comp of dist

        # velocity in each direction : NOTE: not necessary
        dx = north_vel * math.cos(math.radians(vel_bearing))
        dy = east_vel * math.sin(math.radians(vel_bearing))
        north_delta_v = np.dot(np.array([dx, dy]), np.array([1, 0]))
        east_delta_v = np.dot(np.array([dx, dy]), np.array([0, 1]))

        # TWR
        TWR = max_thrust()/(mass() * 9.81) if mass() != 0 else 0 # thrust / weight (stopping TWR at kerbin's surface) ! fix for /0

        print(f"north vel: {north_vel} m/s, dist: {north_delta} m:: east vel: {east_vel} m/s, dist: {east_delta} m, TWR: {TWR}, vert vel: {rel_vel[0]} m/s")
        tmp = np.array([altitude(), north_vel, east_vel, north_delta, east_delta, pitch_error(), heading_error(), TWR, rel_vel[0]]) # add distance in same ref frame to target landing site
        yield np.nan_to_num(tmp.flatten(), 0)
    
    def set_throttle(self, val):
      if val > 1 or val < 0:
        return 0
      self.vessel.control.throttle = val
      return 1
    
    def set_pitch(self, val):
      if val < -1 or val > 1:
        return 0
      self.vessel.control.pitch = val
      return 1
    
    def set_target_pitch(self, val):
      if val < -90 or val > 90:
        return 0
      self.autopilot.target_pitch = val
      return 1

    def set_target_heading(self, val):
      if val < 0 or val > 360:
        return 0
      self.autopilot.target_heading = val
      return 1
    
    def set_target_roll(self, val):
      if val < -180 or val > 180:
        return 0
      self.autopilot.target_roll = val
      return 1

    def set_yaw(self, val):
      if val < -1 or val > 1:
        return 0
      self.vessel.control.yaw = val
      return 1

    def set_roll(self, val):
      if val < -1 or val > 1:
        return 0
      self.vessel.control.roll = val
      return 1

    def set_legs(self, val):
      if val not in [True, False]:
        return 0
      self.vessel.control.legs = val
      return 1
      
    def engage_auto(self):
      self.autopilot.engage()
      self.autopilot.reference_frame = self.vessel.surface_reference_frame
    
    def set_target_direction(self, pitch, north, east):
      self.autopilot.target_direction = (pitch, north, east) # up, north, east


class KSPPilot(object):

    def __init__(self, state=None):
      self.action_space = gym.spaces.Box(low=-1, high=1,shape=(4,),dtype=np.float32)
      #Box(low=-1.0, high=2.0, shape=(3, 4), dtype=np.float32)
      self.observation_space = gym.spaces.Box(low=-1000, high=10000, shape=(9,), dtype=np.float64)
      self.conn = KSPController()
      self.reset()
      if state is not None:
          self.state = state

    def reset(self):
      # reset KSP quicksave
      self.conn.reset_gamestate()
      # telemetry generator
      self.telemetry = self.conn.telemetry_stream()
      self.conn.engage_auto() # enable autopilot
      self.state = next(self.telemetry) 
      self.done = False
      print(self.state.shape)
      return self.state

    class action_spacea():
      #legs = [0, 1]
      throt = np.linspace(0, 1, 5)
      #pitch = np.linspace(-1, 1, 20)
      pitch = np.linspace(-1, 1, 20) # autopilo control
      #yaw = np.linspace(-1, 1, 20)
      north = np.linspace(-1, 1, 20) # actually heading lol
      east = np.linspace(-1, 1, 20)
      #roll = np.linspace(-1, 1, 20)
      #roll = np.linspace(0, 360, 360)
      ACTS = list(itertools.product(throt, pitch, north, east))
      print(len(ACTS))
      # TODO: implement a way to perform accurate movements (more than one step per step of the agent)
      n = len(ACTS)
      num_actions = n
      

    def value(self, act):
      # calculate the deltas for each coordinate in the coordinate plane
      # implement the reward function -> 
      #   1. minimize fuel usage (dV)
      #   2. minimize distance to target
      #   3. minimize g-forces : TODO
      #   4. minimize engine firings : TODO

      rew = 0
      
      # CRASH DETECT:
      if len(self.conn.parts.all) < 23:
        self.done = True # crashed
      
      # TOO HIGH:
      if self.state[0] > 10000:
        self.done = True # went too high
        return -100000000

      if self.state[-1] > 10 and self.state[0] < 100:# if going back up from bounce
        self.done = True
        return (-abs(self.prev_state[-1]))
      elif self.state[-1] > 10: # going up not from bounce
        self.done = True
        return -10000000 * self.state[0] # extra bad

      """# GEAR
      if str(self.conn.vessel.situation) == "VesselSituation.landed" and self.state[-8] == 1:
        rew += 1 # bonus for landing with gear down
      elif str(self.conn.vessel.situation) == "VesselSituation.landed":
        rew -= 1"""

      # AERO:
      # TODO: calculation of g-forces max

      #rew -=  * 0.001 # altitude diff
      #rew -= abs(self.state[-1] - self.state[6]) # bearing diff
      if not self.done:
        if abs(self.prev_state[-1]) > abs(self.state[-1]*1.5): # hitting ground messes up the actual hitting velocity
          rew = (-abs(self.prev_state[-1]))
          rew += -(abs(self.state[4]) + abs(self.state[3]))
        else:
          rew = (-abs(self.state[-1])) # altitude
          rew += -(abs(self.state[3]) + abs(self.state[4]))
          #rew += -(abs(self.state[3]) + abs(self.state[4])) # add difference to north,east landing pt
      else:
        rew = (-abs(self.prev_state[-1])) # remaining vert velocity

      #rew *= (abs(self.state[4]) + abs(self.state[4])) if rew < 0 else -(abs(self.state[3]) + abs(self.state[4]))# remaining north, east velocity magnitudes
      #rew -= (self.state[0])
      
      # CRASH ALT:
      #rew *= abs((self.state[0]))
      # FUEL:
      #rew += (5760.0 - self.state[-3])/5760# minimize fuel loss
      # VELOCITY DELTA: learn to minimize vert and horiz velocity

      return rew

    def step(self, act):

      self.prev_state = self.state
      # take actions
      throttle, pitch, north, east = act[0]
      #print(f"act: {self.action_space.ACTS[act]}")
      
      self.conn.set_throttle(float(throttle))
      self.conn.set_target_direction(pitch=float(pitch), north=float(north), east=float(east))

      new_s = next(self.telemetry, None) # increment data

      if new_s is not None:
          self.state = new_s
          rew = self.value(act)
      else:
          self.done = True
          rew = self.value(act)
      #print(self.conn.vessel.situation)
      
      return self.state, rew, self.done, None
