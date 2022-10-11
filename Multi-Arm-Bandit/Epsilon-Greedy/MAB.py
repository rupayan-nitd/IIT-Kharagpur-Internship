from __future__ import division
import numpy as np
import matplotlib.pyplot as plt

import random
import pprint
from collections import Counter
import copy
import functools
import time
import copy

NO_OF_RULES = 100001
proto_list = ['0', '1'] #['tcp', 'udp']
port_list = ['1883', '8883', '5683', '5672', '443']
MAX_SRC = 10000
MAX_DST = 10
MAX_PORTS = 3
MAX_Type = 5
MAX_TOS_BITS = 5
# store OpenFlow table
flow_table = []
rule_list = []

index_action = {}
mydict = {}

dict_keyName = {"0#1#":"in_port+eth_src", "0#2#":"in_port+eth_dst", "0#3#":"in_port+eth_type", "0#4#":"in_port+vlan_id", "0#5#":"in_port+vlan_prior", "0#6#":"in_port+src_IP", "0#7#":"in_port+dst_IP", "0#8#":"in_port+proto", "0#9#":"in_port+tos", "0#10#":"in_port+src_port", "0#11#":"in_port+dst_port", "1#2#":"eth_src+eth_dst", "1#3#":"eth_src+eth_type", "1#4#":"eth_src+vlan_id", "1#5#":"eth_src+vlan_prior", "1#6#":"eth_src+src_IP", "1#7#":"eth_src+dst_IP", "1#8#":"eth_src+proto", "1#9#":"eth_src+tos", "1#10#":"eth_src+src_port", "1#11#":"eth_src+dst_port", "2#3#":"eth_dst+eth_type", "2#4#":"eth_dst+vlan_id", "2#5#":"eth_dst+vlan_prior", "2#6#":"eth_dst+src_IP", "2#7#":"eth_dst+dst_IP", "2#8#":"eth_dst+proto", "2#9#":"eth_dst+tos", "2#10#":"eth_dst+src_port", "2#11#":"eth_dst+dst_port", "3#4#":"eth_type+vlan_id", "3#5#":"eth_type+vlan_prior", "3#6#":"eth_type+src_IP", "3#7#":"eth_type+dst_IP", "3#8#":"eth_type+proto", "3#9#":"eth_type+tos", "3#10#":"eth_type+src_port", "3#11#":"eth_type+dst_port", "4#5#":"vlan_id+vlan_prior", "4#6#":"vlan_id+src_IP", "4#7#":"vlan_id+dst_IP", "4#8#":"vlan_id+proto", "4#9#":"vlan_id+tos", "4#10#":"vlan_id+src_port", "4#11#":"vlan_id+dst_port", "5#6#":"vlan_prior+src_IP", "5#7#":"vlan_prior+dst_IP", "5#8#":"vlan_prior+proto", "5#9#":"vlan_prior+tos", "5#10#":"vlan_prior+src_port", "5#11#":"vlan_prior+dst_port", "6#7#":"src_IP+dst_IP", "6#8#":"src_IP+proto", "6#9#":"src_IP+tos", "6#10#":"src_IP+src_port", "6#11#":"src_IP+dst_port", "7#8#":"dst_IP+proto", "7#9#":"dst_IP+tos", "7#10#":"dst_IP+src_port", "7#11#":"dst_IP+dst_port", "8#9#":"proto+tos", "8#10#":"proto+src_port", "8#11#":"proto+dst_port", "9#10#":"tos+src_port", "9#11#":"tos+dst_port", "10#11#":"src_port+dst_port"}

def create_rule():
    """
    Create dictionary to represent OpenFlow rules
    """
    global rule_list
    for i in range(NO_OF_RULES):
        # create flow-rule structure
        rule = {'0' : str(random.randint(1, MAX_PORTS)),'1' : random.randint(1, MAX_SRC),'2' : random.randint(1, MAX_DST),'3' : random.randint(1, MAX_Type),'4' : random.randint(1, 2),'5' : random.randint(1, 2),'6' : random.randint(1, MAX_SRC),'7' : random.randint(1, MAX_DST),'8' : random.choice(proto_list),'9' : random.randint(1, MAX_TOS_BITS),'10' : str(random.randint(49152, 65535)),'11' : random.choice(port_list),'12' : random.randint(1, MAX_PORTS)}
        """rule = 
                                {
                                    '0' : str(random.randint(1, MAX_PORTS)),#ingress port
                                    '1' : random.randint(1, MAX_SRC),#Ether src
                                    '2' : random.randint(1, MAX_DST),#Ether dst
                                    '3' : random.randint(1, MAX_Type),#Ether Type
                                    '4' : random.randint(1, 2), # "vlan_id"
                                    '5' : random.randint(1, 2), # "vlan_priority"
                                    '6' : random.randint(1, MAX_SRC),#IP src
                                    '7' : random.randint(1, MAX_DST),#IP dst
                                    '8' : random.choice(proto_list), # "proto"
                                    '9' : random.randint(1, MAX_TOS_BITS),#TOS_BITS
                                    '10' : str(random.randint(49152, 65535)),# "src_port"
                                    '11' : random.choice(port_list), # "dst_port"
                                    '12' : random.randint(1, MAX_PORTS) #"action"
                                }"""
        rule_list.append(rule)

def aggregate():
    global mydict
    global flow_table
    global rule_list
    for rul in flow_table: # TO DO : HOW TO DO FOR MORE THAN 2 HEADERS EFFICIENTLY?		
        rule=copy.deepcopy(rul)
        rule_action = rule['12']
        for i in range(0,11):
            for j in range(i+1,12):
                x=str(i)
                y=str(j)
                key = ""
                key = x + "#" + y + "#" #str(rule[x]) + "#" + str(rule[y]) + "#"
                flag=0
                if key in mydict:			
                    for thisrule in mydict[key]:
                        if thisrule[x]==rule[x] and thisrule[y]==rule[y] and  thisrule['12'] == rule_action:
                            index = 0
                            while index<12: # making other star except ith and jth index and which does'nt match in thisrule
                                if thisrule[str(index)] != rule[str(index)]:
                                    thisrule[str(index)]='*'                                
                                index = index + 1
                            flag = 1                            
                            break
                    if flag == 0:					                   
                        mydict[key].append(rule)
                else:
                    mydict[key] = []
                    mydict[key].append(rule)

def main():
    # =========================
    # Settings
    # =========================

    create_rule() # Total Rule Dataset

    N_experiments = 2000  # number of experiments to perform
    N_episodes = 10000  # number of episodes per experiment
    epsilon = 0.1  # probability of random exploration (fraction)
    save_fig = True  # if false -> plot, if true save as file in same directory
    save_format = ".png"  # ".pdf" or ".png"

    # =========================
    # Define Bandit and Agent class
    # =========================
    class Bandit:
        def __init__(self):
            self.N = 66  # number of bandits
            #self.prob = bandit_probs  # success probabilities for each bandit

        # Get reward (1 for success, 0 for failure)
        def get_reward(self, action):
            #rand = np.random.random()  # [0.0,1.0)
            global mydict_copy
            reward = 1/mydict_copy[action] # 1/length of the flow table size with this action chosen
            return reward

    class Agent:
        def __init__(self, bandit, epsilon):
            self.epsilon = epsilon
            self.k = np.zeros(bandit.N, dtype=np.int)  # number of times action was chosen
            self.Q = np.zeros(bandit.N, dtype=np.float)  # estimated value

        # Update Q action-value using:
        # Q(a) <- Q(a) + 1/(k+1) * (r(a) - Q(a))
        def update_Q(self, action, reward):
            self.k[action] += 1  # update action counter k -> k+1
            self.Q[action] += (1./self.k[action]) * (reward - self.Q[action])

        # Choose action using an epsilon-greedy agent
        def choose_action(self, bandit, force_explore=False):
            rand = np.random.random()  # [0.0,1.0)
            if (rand < self.epsilon) or force_explore:
                action_explore = np.random.randint(bandit.N)  # explore random bandit
                return action_explore
            else:
                #action_greedy = np.argmax(self.Q)  # exploit best current bandit
                action_greedy = np.random.choice(np.flatnonzero(self.Q == self.Q.max()))
                return action_greedy

    # =========================
    # Define an experiment
    # =========================
    def experiment(agent, bandit, N_episodes):
        action_history = [] # 10000
        reward_history = [] # 10000
        for episode in range(N_episodes):
            # Choose action from agent (from current Q estimate)
            action = agent.choose_action(bandit)
            # Pick up reward from bandit for chosen action
            reward = bandit.get_reward(action)
            # Update Q action-value estimates
            agent.update_Q(action, reward)
            # Append to history
            action_history.append(action) # size 10000
            reward_history.append(reward) # size 10000
        return (np.array(action_history), np.array(reward_history)) # each array size of N_episodes=10000

    # =========================
    #
    # Start multi-armed bandit simulation
    #
    # =========================
    N_bandits = 66 #len(bandit_probs)
    print("Running multi-armed bandits with N_bandits = {} and agent epsilon = {}".format(N_bandits, epsilon))
    reward_history_avg = np.zeros(N_episodes)  # reward history experiment-averaged
    action_history_sum = np.zeros((N_episodes, N_bandits))  # sum action history
    increase_flow = 0
    for i in range(N_experiments):
        global flow_table
        global rule_list
        flow_table=[]
        for flow in range(50): #adding 50 rules each time
            flow_table.append(rule_list[increase_flow])
            increase_flow = increase_flow + 1

        global mydict_copy
        mydict_copy={}
        global index_action
        index_action={}

        aggregate()
        k=0
        for key in mydict:
            mydict_copy[k] = len(mydict[key])
            index_action[k] = key
            k=k+1

        bandit = Bandit()  # initialize bandits
        agent = Agent(bandit, epsilon)  # initialize agent
        (action_history, reward_history) = experiment(agent, bandit, N_episodes)  # perform experiment

        if (i + 1) % (N_experiments / 20) == 0: #print after every 100 Experiments
            print("[Experiment {}/{}]".format(i + 1, N_experiments))
            print("  N_episodes = {}".format(N_episodes))
            print("  bandit choice history = {}".format(
                action_history + 1))
            print("  reward history = {}".format(
                reward_history))
            print("  average reward = {}".format(np.sum(reward_history) / len(reward_history)))
            print("")
        # Sum up experiment reward (later to be divided to represent an average)
        reward_history_avg += reward_history
        # Sum up action history
        for j, (a) in enumerate(action_history):
            action_history_sum[j][a] += 1

    reward_history_avg /= np.float(N_experiments)
    print("reward history avg = {}".format(reward_history_avg))

    # =========================
    # Plot reward history results
    # =========================
    plt.plot(reward_history_avg)
    plt.xlabel("Episode number")
    plt.ylabel("Rewards collected".format(N_experiments))
    plt.title("Bandit reward history averaged over {} experiments (epsilon = {})".format(N_experiments, epsilon))
    ax = plt.gca()
    ax.set_xscale("log", nonposx='clip')
    plt.xlim([1, N_episodes])
    if save_fig:
        output_file = "MAB_rewards" + save_format
        plt.savefig(output_file, bbox_inches="tight")
    else:
        plt.show()

    global dict_keyName

    # =========================
    # Plot action history results
    # =========================
    plt.figure(figsize=(18, 12))
    for i in range(N_bandits):
        action_history_sum_plot = 100 * action_history_sum[:,i] / N_experiments
        plt.plot(list(np.array(range(len(action_history_sum_plot)))+1),
                 action_history_sum_plot,
                 linewidth=5.0,
                 label="{}".format(dict_keyName[index_action[i]]))
    plt.title("Bandit action history averaged over {} experiments (epsilon = {})".format(N_experiments, epsilon), fontsize=26)
    plt.xlabel("Episode Number", fontsize=26)
    plt.ylabel("Bandit Action Choices (%)", fontsize=26)
    leg = plt.legend(loc='upper left', shadow=True, fontsize=26)
    ax = plt.gca()
    ax.set_xscale("log", nonposx='clip')
    plt.xlim([1, N_episodes])
    plt.ylim([0, 100])
    plt.xticks(fontsize=24)
    plt.yticks(fontsize=24)
    for legobj in leg.legendHandles:
        legobj.set_linewidth(16.0)
    if save_fig:
        output_file = "MAB_actions" + save_format
        plt.savefig(output_file, bbox_inches="tight")
    else:
        plt.show()



# Driver
if __name__ == "__main__":
    main()
