import time
import requests
import sys, io
import krpc
import math

import tensorflow.keras as keras
import keras_gym as km
import gym
import random as rd
import tensorflow as tf

import numpy as np
import itertools

class MLP(km.FunctionApproximator):
    """ multi-layer perceptron with one hidden layer """
    def body(self, S):
        X = keras.layers.Dense(units=7)(S)
        X = keras.layers.Dense(units=1024,activation="relu")(X)
        return X

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
        tmp = np.array([altitude(), north_vel, east_vel, north_delta, east_delta, TWR, rel_vel[0]]) # add distance in same ref frame to target landing site
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
      self.action_space = gym.spaces.Discrete(self.action_spacea().n)
      #Box(low=-1.0, high=2.0, shape=(3, 4), dtype=np.float32)
      self.observation_space = gym.spaces.Box(low=-10000, high=100000, shape=(7,), dtype=np.float64)
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
      update_ = [0, 1]
      throt = np.linspace(0, 1, 5)
      #pitch = np.linspace(-1, 1, 20)
      pitch = np.linspace(-1, 1, 20) # autopilo control
      #yaw = np.linspace(-1, 1, 20)
      north = np.linspace(-1, 1, 20) # actually heading lol
      east = np.linspace(-1, 1, 20)
      #roll = np.linspace(-1, 1, 20)
      #roll = np.linspace(0, 360, 360)
      ACTS = list(itertools.product(update_, throt, pitch, north, east))
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

      print(rew)
      return rew

    def step(self, act):
      self.prev_state = self.state
      # take actions
      u, throttle, pitch, north, east = self.action_spacea.ACTS[act]
      #print(f"act: {self.action_space.ACTS[act]}")
      if u == 1: # only update actions if want to
        self.conn.set_throttle(throttle)
        self.conn.set_target_direction(pitch=pitch, north=north, east=east)

      new_s = next(self.telemetry, None) # increment data

      if new_s is not None:
          self.state = new_s
          rew = self.value(act)
      else:
          self.done = True
          rew = self.value(act)
      #print(self.conn.vessel.situation)
      
      return self.state, rew, self.done, None


"""import gym
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import keras
import datetime
import tensorboard
import keras.backend as K

import itertools
import numpy as np

from tqdm import tqdm
import tensorflow as tf
from tensorflow.python.keras.layers import Dense, Dropout, BatchNormalization, Reshape, Conv1D, Input
from tensorflow.python.keras import Sequential
from tensorflow.keras.optimizers import Adam, SGD
from tensorflow.python.keras.models import load_model
import warnings
warnings.filterwarnings('ignore')
tf.get_logger().setLevel('ERROR')"""

# set the env
env= KSPPilot()# env to import
#env.reset() # reset to env


def named_logs(model, logs):
  result = {}
  for l in zip(model.metrics_names, logs):
    result[l[0]] = l[1]
  return result

class REINFORCE:
  def __init__(self, env, path=None):
    self.env=env #import env
    self.state_shape=env.observation_space.shape # the state space
    self.action_shape=env.action_space.n # the action space
    self.gamma=0.99 # decay rate of past observations
    self.alpha=0.01 # learning rate in the policy gradient
    self.learning_rate=0.1 # learning rate in deep learning

    self.model=self._create_model() #build model
    
    # record observations
    self.states=[]
    self.gradients=[] 
    self.id = 0
    self.rewards=[]
    self.probs=[]
    self.discounted_rewards=[]
    self.total_rewards=[]
  
  def _create_model(self):
    ''' builds the model using keras'''
    # input shape is of observations
    model = Sequential()
    #model.add(Reshape(()))
    model.add(Dense(1280, input_shape=self.state_shape, activation="sigmoid"))
    model.add(Dropout(0.1))
    model.add(Dense(1280, activation="tanh"))
    model.add(Dense(4096, activation="relu"))
    model.add(Dense(1280, activation="relu"))
    
    # output shape is according to the number of action
    # The softmax function outputs a probability distribution over the actions
    model.add(Dense(self.action_shape, activation="softmax"))
    model.compile(loss="binary_crossentropy",
            optimizer=Adam(lr=self.learning_rate))
    self.tensorboard = keras.callbacks.TensorBoard(
      log_dir='logs/'+datetime.datetime.now().strftime("%Y%m%d-%H%M%S"),
      histogram_freq=1,
      write_graph=True,
      write_grads=True,
      update_freq="batch"
    )
    self.model = model
    self.tensorboard.set_model(model)
        
    return model

  def hot_encode_action(self, action):
    '''encoding the actions into a binary list'''

    action_encoded=np.zeros(self.action_shape, np.float64)
    action_encoded[action]=1

    return action_encoded
  
  def remember(self, state, action, action_prob, reward):
    '''stores observations'''
    encoded_action=self.hot_encode_action(action)
    self.gradients.append(encoded_action-action_prob)
    self.states.append(state)
    self.rewards.append(reward)
    self.probs.append(action_prob)

  
  def get_action(self, state):
    '''samples the next action based on the policy probabilty distribution 
      of the actions'''

    # transform state
    
    state=state.reshape([1, self.state_shape[0]])
    # get action probably
    action_probability_distribution=self.model.predict(state).flatten()
    # norm action probability distribution
    action_probability_distribution/=np.sum(action_probability_distribution)
    
    #print(action_probability_distribution.shape)
    #print(np.random.choice(self.action_shape,1,p=action_probability_distribution[0:16]))
    # sample action
    best_act = np.argmax(action_probability_distribution)
    action=np.random.choice(self.action_shape,1,
                            p=action_probability_distribution)[0]

    return action, action_probability_distribution


  def get_discounted_rewards(self, rewards): 
    '''Use gamma to calculate the total reward discounting for rewards
    Following - \gamma ^ t * Gt'''
    
    discounted_rewards=[]
    cumulative_total_return=0
    # iterate the rewards backwards and and calc the total return 
    for reward in rewards[::-1]:      
      cumulative_total_return=(cumulative_total_return*self.gamma)+reward
      discounted_rewards.insert(0, cumulative_total_return)

    # normalize discounted rewards
    mean_rewards=np.mean(discounted_rewards)
    std_rewards=np.std(discounted_rewards)
    norm_discounted_rewards=(discounted_rewards-
                          mean_rewards)/(std_rewards+1e-7) # avoiding zero div
    
    return norm_discounted_rewards


  def update_policy(self):
    '''Updates the policy network using the NN model.
    This function is used after the MC sampling is done - following
    \delta \theta = \alpha * gradient + log pi'''
    self.id += 1
      
    # get X
    states=np.vstack(self.states)

    # get Y
    gradients=np.vstack(self.gradients)
    rewards=np.vstack(self.rewards)
    discounted_rewards=self.get_discounted_rewards(rewards)
    gradients *= discounted_rewards
    gradients=self.alpha*np.vstack([gradients])+self.probs

    history=self.model.train_on_batch(states, gradients)
    self.tensorboard.on_epoch_end(self.id, {"loss":history})
    self.states, self.probs, self.gradients, self.rewards=[], [], [], []

    return history



  def train(self, episodes, rollout_n=1, render_n=50):
    '''train the model
        episodes - number of training iterations 
        rollout_n- number of episodes between policy update
        render_n - number of episodes between env rendering ''' 
    
    env=self.env
    total_rewards=np.zeros(episodes)

    for episode in range(episodes):
      # each episode is a new game env
      state=env.reset()
      done=False          
      episode_reward=0 #record episode reward
      
      while not done:
        # play an action and record the game state & reward per episode
        action, prob = self.get_action(state)
        next_state, reward, done, _ = env.step(action)
        self.remember(state, action, prob, reward)
        state=next_state
        episode_reward += reward
        if done:
          print(episode_reward)
          history=self.update_policy() # update policy each iteration?
         
            

      total_rewards[episode]=episode_reward
      
    self.total_rewards=total_rewards
  def save_model(self):
    '''saves the moodel // do after training'''
    self.model.save('REINFORCE_model.h5')
  
  def load_model(self, path):
    '''loads a trained model from path'''
    return load_model(path)


if __name__ == "__main__":
    """agent=REINFORCE(env)
    agent.train(100)
    import matplotlib.pyplot as plt
    import math"""

    env = KSPPilot()
    function_approximator = MLP(env, lr=0.1)
    pi = km.SoftmaxPolicy(function_approximator, update_strategy='vanilla')
    v = km.V(function_approximator, gamma=0.9, bootstrap_n=1)
    # combine them into a single actor-critic
    actor_critic = km.ActorCritic(pi, v)
    for ep in range(100):
      s = env.reset()

      for t in range(10000):
          a = pi(s, use_target_model=True)
          s_next, r, done, info = env.step(a)

          # small incentive to keep moving
          if np.array_equal(s_next, s):
              r = -0.1

          actor_critic.update(s, a, r, done)

          if t % 2 == 0:
              pi.sync_target_model(tau=1.0)

          if done:
              break

          s = s_next


    """plt.title('REINFORCE Reward')
    plt.xlabel('Episode')
    plt.ylabel('Average reward (Episode length)')
    plt.plot(agent.total_rewards)
    plt.show()"""