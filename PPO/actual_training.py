#coding=UTF-8
from webbrowser import get
import rospy
import time
from collections import namedtuple
from agent import PPO
from env import Env
from std_msgs.msg import Float32MultiArray
EPISODES = 3000

PPO=PPO()
action_size =5
env =Env(action_size)
if __name__ == '__main__':
    rospy.init_node('PPO')
    start_time =time.time()
    for e in range(300):
        state = env.reset()#env.reset()函数用于重置环境
        episode_reward_sum = 0  # 初始化该循环对应的episode的总奖励
        done=False
        episode_step=6000
        for t in range(episode_step):
            action, action_prob = PPO.select_action(state)
            next_state, reward, done= env.step(action)
            get_goalbox= env.get_goalbox
            state = next_state
            episode_reward_sum+=reward
            if  env.get_goalbox:
                m,s =divmod(int(time.time()- start_time),60)
                h,m =divmod(m,60)
                rospy.loginfo('Ep: %d score: %.2f time: %d:%02d:%02d' , e ,episode_reward_sum, h, m, s)
                break
            if done :
                m,s =divmod(int(time.time()- start_time),60)
                h,m =divmod(m,60)
                rospy.loginfo('Ep: %d score: %.2f  time: %d:%02d:%02d' , e ,episode_reward_sum, h, m, s)
                break
