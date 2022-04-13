
#coding=UTF-8
from env import Env
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal, Categorical
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler

# gamma = 0.99
# render = False
# seed = 1
#log_interval = 10
num_state =28
num_action = 5
env=Env(num_action)
# torch.manual_seed(seed)#为CPU设置种子用于生成随机数，以使得结果是确定的
# env.seed(seed)
# Transition = namedtuple('Transition', ['state', 'action',  'a_log_prob', 'reward', 'next_state'])
#Actor网络
class Actor(nn.Module):
    def __init__(self):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(num_state, 100)
        self.action_head = nn.Linear(100, num_action)
    def forward(self, x):
        x = F.relu(self.fc1(x))
        action_prob = F.softmax(self.action_head(x), dim=1)
        return action_prob
#Critic网络
class Critic(nn.Module):
    def __init__(self):
        super(Critic, self).__init__()
        self.fc1= nn.Linear(num_state, 100)
        self.state_value = nn.Linear(100, 1)
    def forward(self, x):
        x = F.relu(self.fc1(x))
        value = self.state_value(x)
        return value
#智能体
class PPO(object):
    clip_param = 0.2
    max_grad_norm = 0.5
    ppo_update_time = 10
    buffer_capacity = 1000
    batch_size = 128

    def __init__(self):
        super(PPO, self).__init__()
        self.actor_net = Actor()
        self.critic_net = Critic()
        self.buffer = []
        # self.counter = 0
        self.training_step = 0
        self.action_loss= 0.
        self.value_loss =0.
        self.load_models =True
        self.load_ep =175
        self.actor_optimizer = optim.Adam(self.actor_net.parameters(), 1e-3)
        self.critic_net_optimizer = optim.Adam(self.critic_net.parameters(), 3e-3)
        # Adam(Adaptive Moment Estimation)本质上是带有动量项的RMSprop，它利用梯度的一阶矩估计和二阶矩估计动态调整每个参数的学习率。它的优点主要在于经过偏置校正后，每一次迭代学习率都有个确定范围，使得参数比较平稳。
        #加载模型
        if self.load_models:
            load_model1 = torch.load("/home/ffd/catkin_ws/src/model_test/src/PPO/model/175c1.pt")
            self.actor_net.load_state_dict(load_model1['actor_net'])
            self.critic_net.load_state_dict(load_model1['critic_net'])
            print("load model:",str(self.load_ep))
            print("load model successful!!!!!!")
    #选择动作
    def select_action(self, state):
        state = torch.from_numpy(state).float().unsqueeze(0) 
        with torch.no_grad():
            action_prob = self.actor_net(state)
        c = Categorical(action_prob)
        action = c.sample()
        return action.item(), action_prob[:,action.item()].item()
    #获取值函数
    def get_value(self, state):
        state = torch.from_numpy(state)
        with torch.no_grad():
            value = self.critic_net(state)
        return value.item()