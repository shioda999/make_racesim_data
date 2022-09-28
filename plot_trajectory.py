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
import yaml
import matplotlib.pyplot as plt

action_repeat = 4
obs_save = True
video_on = False

def save_video(imgs, file_name):
  Y1, X1, channels = imgs[0].shape[:3] 
  frame_rate = 100 // action_repeat
  fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
  video = cv2.VideoWriter(file_name, fourcc, frame_rate, (X1, Y1))
  for img in imgs : video.write(img[:,:,::-1])
  video.release()

def load_map(track):
    dir = 'racecar_gym/models/scenes/%s' % track
    with open('%s/%s.yml' % (dir, track), 'r') as yml:
        config = yaml.safe_load(yml)
    map = np.load('%s/%s' % (dir, config['map']['maps']))
    #print([k for k in map])
    #input()
    map = map['norm_distance_from_start']
    map[map > 0.995] = 0
    map[map > 0] = 255
    shape = map.shape
    map = map.reshape(shape + (1,))
    map = np.broadcast_to(map, shape + (3,))
    map = map.astype(np.uint8)
    return map, config['map']

def crop_img(img):
    K = 10
    img2 = img.copy()
    img2[img2 > 0] = 255
    contours, hierarchy = cv2.findContours(img2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv2.boundingRect(contours[0])
    x, y, w, h = int(x - K), int(y - K), int(w + 2 * K), int(h + 2 * K)
    return img[y:y+h, x:x+w]

def to_pixel(pos, H, config):
    origin_x, origin_y = config['origin'][0], config['origin'][1]
    res = config['resolution']
    x, y = pos[0], pos[1]
    px = int(H - (y - origin_y) / res)
    py = int((x - origin_x) / res)
    return py, px

def plot_trajectory(img, config, pos, color=(255, 0, 0), thickness=5):
    n = len(pos)
    for i in range(n):
        pos[i] = to_pixel(pos[i], img.shape[1], config)
    for i in range(n - 1):
        c = plt.cm.jet(i % 256)
        cv2.line(img, pos[i], pos[i+1], color=(c[0] * 255, c[1] * 255, c[2] * 255), thickness=thickness)

def simulate(env, episode, video_on=False):
    env.reset()
    #action = episode["action"]
    action = episode
    n = len(action)#.shape[0]
    imgs = []
    pos = []
    ret = {}
    #env.step({"motor":0, "steering":0})
    for i in range(n):
        #motor, steering = action[i+1]
        #for _ in range(action_repeat):
        #obs, reward, done, info = env.step({"motor":motor, "steering":steering})
        obs, reward, done, info = env.step(action[i])
        pos.append(tuple(info["pose"][:2]))
        if video_on:
            if i % action_repeat == 0:
                img = env.render('birds_eye')
                img = img.astype(np.uint8)
                imgs.append(img)
    ret["pos"] = pos
    if video_on: ret["video"] = imgs            
    return ret

def main():
    dir = 'action/*.pickle'
    fname = select_file(dir)
    #fname = "action/austria_0925_00h53m(2).npz"
    with open(fname, mode='rb') as f:
        episode = pickle.load(f)

        scenario = SingleAgentScenario.from_spec('world.yml')
        env = gym_api.SingleAgentRaceEnv(scenario=scenario)
        
        ret = simulate(env, episode, video_on)

        if video_on: save_video(ret["video"], "%s(test).mp4" % fname.split('.', 1)[0])
        map, config = load_map("austria")
        plot_trajectory(map, config, ret["pos"])
        cv2.imwrite("result.png", map)

main()