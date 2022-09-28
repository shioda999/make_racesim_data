from racecar_gym.envs import gym_api
from racecar_gym import SingleAgentScenario
import cv2
import numpy as np
import pybullet
from copy import deepcopy
import pickle
import datetime
from select_file import select_file
import math
from scipy.signal import medfilt

action_repeat = 4
obs_save = True
video_on = False

def save_obs(obss, actions, fname):
    data = {}
    for i in range(len(obss)):
        for k, v in obss[i].items():
            if i == 0: data[k] = []
            data[k].append(v)
        for k, v in actions[i].items():
            if i == 0: data[k] = []
            data[k].append(v)
    for k in data:
        data[k] = np.array(data[k])
        if data[k].dtype != np.bool: data[k] = data[k].astype(np.float64)
    np.savez_compressed(fname, **data)
    np.savez(fname + "(2)", **data)
    print("%s saved." % fname)
    #data = np.load(fname + '.npz')
    #print([k for k in data])
    #for k in data:
    #    print(k, data[k])
    #    input()

def save_video(imgs, file_name):
  Y1, X1, channels = imgs[0].shape[:3] 
  frame_rate = 100 // action_repeat
  fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
  video = cv2.VideoWriter(file_name, fourcc, frame_rate, (X1, Y1))
  for img in imgs : video.write(img[:,:,::-1])
  video.release()

def preprocess_lidar(ranges, kernel_size=5):
    # Step 1: interpolate nan values
    proc_ranges = np.array(ranges, dtype=np.float32)
    nans = np.isnan(proc_ranges) | (proc_ranges > 13)
    nan_idx = np.where(nans)
    if len(nan_idx) == 0: return proc_ranges
    if len(nan_idx) == 1080: return proc_ranges
    x = np.where(~nans)[0]
    y = proc_ranges[~nans]
    proc_ranges[nan_idx] = np.interp(nan_idx, x, y)
    # Step 2: apply a median filter to the interpolated values
    proc_ranges = medfilt(proc_ranges, kernel_size)
    return proc_ranges

def obs_convert(obs, infos=None, rate=None, angle=None):
    new_obs = {}
    lidar = preprocess_lidar(obs['lidar'], 3)
    bias = 0.1
    new_obs['lidar'] = (np.log(lidar - 0.25 + bias)) / (math.log(15 + bias) - math.log(bias)) - 0.5
    new_obs['progress'] = infos['progress'] if infos else 0
    new_obs['wall_collision'] = infos['wall_collision'] if infos else False
    new_obs['velocity'] = obs['velocity'] * 0.1
    return new_obs

def main():
    dir = 'action/*.pickle'
    fname = select_file(dir)
    #fname = "action/austria_0924_13h23m.pickle"
    with open(fname, mode='rb') as f:
        actions = pickle.load(f)

    scenario = SingleAgentScenario.from_spec('world.yml')
    env = gym_api.SingleAgentRaceEnv(scenario=scenario)
    obs = env.reset()

    step = 0
    imgs = []
    obss = []
    acts = []
    while step < len(actions):
        obs,_,_,_ = env.step(actions[step])
        if step % action_repeat == 0:
            if obs_save:
                obss.append(obs_convert(obs))
                acts.append({'action':[actions[step]['motor'],actions[step]['steering']]})
            if video_on:
                img = env.render('birds_eye')
                img = img.astype(np.uint8)
                imgs.append(img)
        step += 1
    print(step)
    if obs_save: save_obs(obss, acts, fname.split('.', 1)[0])
    if video_on: save_video(imgs, "%s.mp4" % fname.split('.', 1)[0])

main()