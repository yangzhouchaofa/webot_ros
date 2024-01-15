import numpy as np

from SAC_runner import agent, env, rb_node, DETERMINISTIC
from main import STEPS_PER_EPISODE, path_history_path, create_path

load_path = 'model/sac3/'

agent.load_model(load_path)
path_history = []

observation = env.initialization(rb_node=rb_node, next=4, x=2, z=0, rotation=180
                                 )
episode_reward = 0
step = 0
done = False
env.disRewardOld = env.disReward
positions = []
vws = []
while not done and step < STEPS_PER_EPISODE:
    path_history.append((env.position[0], env.position[1], env.position[2], 0))
    np.save(path_history_path, path_history)
    positions.append(env.position)  # yt:input of mpc
    action = agent.policy_net.get_action(observation, deterministic=DETERMINISTIC)
    action = action/2
    observation_new, reward, done, _ = env.step(action)
    print(action)
    vws.append([(action[0] + action[1]) * 3.14/ 2, (action[0] - action[1]) * 0.628/ 0.398])
    episode_reward += reward
    observation = observation_new
    step += 1
print('| Episode Reward: ', episode_reward, '| Done: ', done)
print(len(positions))
print(len(vws))

f = open('/home/ylc/Reinforcement/simulator/controllers/yt/test.txt', 'w')
print(f)
for line in positions:
    for x in line:
        f.write(str(x) + ' ')
    f.write('\n')
f.close()

f = open('/home/ylc/Reinforcement/simulator/controllers/yt/vw.txt', 'w')
for line in vws:
    for x in line:
        f.write(str(x) + ' ')
    f.write('\n')
f.close()
