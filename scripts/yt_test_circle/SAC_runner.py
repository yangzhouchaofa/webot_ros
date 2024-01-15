import math
from tqdm import tqdm
from environment import Hunter
from SAC import SAC_Trainer
import torch
from main import EPISODE_LIMIT, STEPS_PER_EPISODE, score_history_path, score_history_continue_path, fig_path, \
    path_history_path
from replay_buffer import ReplayBuffer
from config import Arguments
from ros_for_SAC import ros_Hunter
from utilities import plotLearning_PG
from plot_tools import *
from IPython.display import clear_output

env_args = {
    "env_num": 1,
    "env_name": 'yt',
    "max_step": 150,
    "state_dim": 82,
    "action_dim": 2,
    "if_discrete": False,
    "target_return": 200,
    "id": "1",
}
env = Hunter()
env_id = "yt"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# rb_node = env.getFromDef("yt")
seed = 0
torch.manual_seed(seed)
np.random.seed(seed)
batch_size = 2048
update_itr = 1
AUTO_ENTROPY = True
print(f"Using device {device} \n")
score_history_path = score_history_path
score_history_continue_path = score_history_continue_path
fig_path = fig_path

args = Arguments(agent=SAC_Trainer, env=env, env_args=env_args)
replay_buffer = ReplayBuffer(max_size=args.max_memo, state_dim=args.state_dim, action_dim=args.action_dim)
agent = SAC_Trainer(input_dims=args.state_dim, n_actions=args.action_dim, action_range=1., buffer=replay_buffer)
DETERMINISTIC = False


def train(save_path):
    score_history = []
    load_checkpoint = False
    step_sum = 0
    count_done = []
    Radius = 0.5
    next_target = 0.8
    ep = 0
    avg_score = 0
    model_path = save_path
    frame_idx = 0
    explore_steps = 0  # for random action sampling in the beginning of training

    for episodes in range(50000):
        if np.mean(count_done[-50:]) > 0.8:
            next_target += 0.1
            count_done = []

        observation = env.random_initialization(rb_node=rb_node, Radius=Radius, next=next_target)
        env.disRewardOld = env.disReward
        done = False
        score = 0
        step = 0

        while not done and step < STEPS_PER_EPISODE:
            if frame_idx > explore_steps:
                action = agent.policy_net.get_action(observation, deterministic=DETERMINISTIC)
            else:
                action = agent.policy_net.sample_action()
            # print(action)
            observation_new, reward, done, _ = env.step(action)
            score += reward

            replay_buffer.add(observation, action, reward, observation_new, done)

            observation = observation_new
            step += 1
            frame_idx += 1

            if replay_buffer.len() > batch_size:
                for i in range(update_itr):
                    _ = agent.learn(batch_size, reward_scale=10., auto_entropy=AUTO_ENTROPY,
                                    target_entropy=-1. * args.action_dim)

        step_sum += step
        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if episodes % 20 == 0 and episodes > 0:
            plot(score_history)
            np.save(score_history_path, score_history)
            agent.save_model(model_path)


        if env.complete:
            count_done.append(1)
        else:
            count_done.append(0)

        print('episode: {:^3d} | score: {:^10.2f} | avg_score: {:^10.2f} |step_sum: {} |'.format(episodes, score,
                                                                                                 avg_score, step_sum))


def train_continue(load_path):
    agent.load_model(load_path)
    score_history = []
    step_sum = 0
    count_done = []
    Radius = 0.5
    next_target = 2

    model_path = load_path
    frame_idx = 0
    explore_steps = 0  # for random action sampling in the beginning of training

    for episodes in range(500000):
        if np.mean(count_done[-50:]) > 0.8 and len(count_done)>50:
            next_target += 0.1
            count_done = []

        observation = env.random_initialization(rb_node=rb_node, next=next_target)
        env.disRewardOld = env.disReward
        done = False
        score = 0
        step = 0

        while not done and step < STEPS_PER_EPISODE:
            if frame_idx > explore_steps:
                action = agent.policy_net.get_action(observation, deterministic=DETERMINISTIC)
            else:
                action = agent.policy_net.sample_action()
            observation_new, reward, done, _ = env.step(action)
            score += reward

            replay_buffer.add(observation, action, reward, observation_new, done)

            observation = observation_new
            step += 1
            frame_idx += 1

            if replay_buffer.len() > batch_size:
                for i in range(update_itr):
                    _ = agent.learn(batch_size, reward_scale=10., auto_entropy=AUTO_ENTROPY,
                                    target_entropy=-1. * args.action_dim)

        step_sum += step
        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if episodes % 20 == 0 and episodes > 0:
            plot(score_history)
            np.save(score_history_path, score_history)
            agent.save_model(model_path)


        if env.complete:
            count_done.append(1)
        else:
            count_done.append(0)

        print('episode: {:^3d} | score: {:^10.2f} | avg_score: {:^10.2f} |step_sum: {} |'.format(episodes, score,
                                                                                                 avg_score, step_sum))


def test(load_path):
    agent.load_model('model/sac1/')
    path_history = []
    for eps in range(100):
        observation = env.get_observations
        episode_reward = 0
        step = 0
        done = False
        env.disRewardOld = env.disReward
        action_pre = [0, 0]
        while not env.disReward < 0.8 and step < STEPS_PER_EPISODE:
            path_history.append((env.position[0], env.position[1], env.position[2], eps))
            np.save(path_history_path, path_history)
            action = agent.policy_net.get_action(observation, deterministic=DETERMINISTIC)
            action[0] = (action[0] + action_pre[0])/2
            action[1] = (action[1] + action_pre[1])/2
            observation_new, reward, done, _ = env.step(action)
            action_pre = action
            episode_reward += reward
            observation = observation_new
            step += 1
        agent.load_model('model/sac1/')
        while not done and step < STEPS_PER_EPISODE:

            path_history.append((env.position[0], env.position[1], env.position[2], eps))
            np.save(path_history_path, path_history)
            # np.save("path/path_history_obs_x", env.obs_x)
            # np.save("path/path_history_obs_y", env.obs_y)
            # if (step % 10 == 0):
                # print(env.position)  # x.y.r as input of MPC
            action = agent.policy_net.get_action(observation, deterministic=DETERMINISTIC)
            action[0] = (action[0] + action_pre[0])/2
            action[1] = (action[1] + action_pre[1])/2
            observation_new, reward, done, _ = env.step(action)
            action_pre = action
            episode_reward += reward
            observation = observation_new
            step += 1
        print('Episode: ', eps, '| Episode Reward: ', episode_reward, '| Done: ', done)


def plot(rewards):
    clear_output(True)
    plt.figure()
    plt.plot(rewards)
    plt.ylabel("Score", fontsize=16)
    plt.xlabel("episode", fontsize=16)
    plt.title("SAC", fontsize=20)
    plt.savefig(fig_path)
