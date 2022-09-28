from racecar_gym.envs import gym_api
from racecar_gym import SingleAgentScenario
import cv2
import numpy as np
import pybullet
from copy import deepcopy
import pickle
import datetime

track = 'austria'

def clamp(v, min_v, max_v):
    return max(min(v, max_v), min_v)

def save_history(history):
    dt_now = datetime.datetime.now()
    fname = 'action/%s_%s.pickle' % (track, dt_now.strftime('%m%d_%Hh%Mm'))
    with open(fname, 'wb') as f:
        pickle.dump(history, f)
        print("%s saved." % fname)

def main():
    scenario = SingleAgentScenario.from_spec('world.yml')
    env = gym_api.SingleAgentRaceEnv(scenario=scenario)

    done = False
    obs = env.reset()
    t = 0
    freq = 8
    motor = steering = 0
    v_motor = 0
    ESC = 27
    EOT = 3
    save_data = [0] * 10
    history = []
    while not done:
        if t % freq == 0:
            while True:
                img = env.render('birds_eye')[:,:,::-1]
                img = img.astype(np.uint8)
                cv2.imshow("image", img)
                key = cv2.waitKey(0)
                if key >= ord('0') and key <= ord('9'):
                    id = key - ord('0')
                    save_data[id] = (pybullet.saveState(), motor, steering, v_motor, deepcopy(history))
                    print('saved: %d' % id)
                elif key == ord('l'):
                        while True:
                            key2 = cv2.waitKey(0)
                            if key2 >= ord('0') and key2 <= ord('9'):
                                id = key2 - ord('0')
                                if save_data[id] == 0:
                                    print('id: %d is invalid !!!' % id)
                                    continue
                                state, motor, steering, v_motor, history_tmp = save_data[id]
                                history = deepcopy(history_tmp)
                                pybullet.restoreState(state)
                                print('loaded: %d' % id)
                                print(len(history), motor, steering)
                                break
                elif key == ord('p'):
                    save_history(history)
                else: break
            if key == ESC or key == EOT:
                break
            if key == ord('s'):
                motor = -1
            elif motor == -1:
                motor = 0
            if key == ord('w'):
                v_motor = 0.2
                if abs(steering) > 0.1: steering -= steering / abs(steering) * 0.2
            elif key == ord('e'):
                v_motor = 0.01
            else: v_motor = 0
            if key == ord('a'):
                steering -= 0.2
            elif key == ord('d'):
                steering += 0.2
            if not key == ord('r'):
                motor += v_motor
                motor = clamp(motor, -1, 1)
            steering = clamp(steering, -1, 1)
            if abs(motor) < 0.01:
                motor = 0
            if abs(steering) < 0.01:
                steering = 0
            print(len(history), motor, steering)
        action = {'motor':motor, 'steering':steering}
        obs, reward, done, info = env.step(action)
        history.append(action)
        t += 1
    env.close()

main()