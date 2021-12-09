import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import time
from random import seed

import sim
import players
import src.shufflebot as shufflebot
from cv_stuff.shuffle_board_cv import get_real_board_state

class ShuffleBoardGame:
    length = 2.4384
    width = 0.4572
    puck_rad = 0.026
    num_pucks = 8
    sl_3 = length * (91.0/96.0)
    sl_2 = length * (86.0/96.0)
    sl_1 = length * (48.0/96.0)

    def __init__(self):
        self.dt = 0.01
        self.sim = sim.ShuffleBoardSim(self.dt)

    def sim_turn(self, state, puck, x_pos, y_pos, x_vel, y_vel):
        original_x = state.get_all(puck)
        state.set_all(puck, np.array([x_pos, y_pos, x_vel, y_vel]))
        xf, xs = sim.simulate(self.sim, state, self)
        state.set_all(puck, original_x)
        return xf, xs

    def score_board(self, state):
        width = ShuffleBoardGame.width
        length = ShuffleBoardGame.length
        
        scores = [0,0]
        y_pos = [[self.sl_1],[self.sl_1]]

        for i in range(self.num_pucks):
            pos = state.get_x(i)
            if pos[0] >= 0 and pos[0] <= width and pos[1] <= length and pos[1] > self.sl_1:
                y_pos[i%2].append(pos[1])
        if max(y_pos[0]) > max(y_pos[1]):
            winner = 0
        else:
            winner = 1
        all_winning_y_pos = [pos for pos in y_pos[winner] if pos > max(y_pos[1 - winner])]
        for winning_y_pos in all_winning_y_pos:
            if winning_y_pos > self.sl_3:
                scores[winner] += 3
            elif winning_y_pos > self.sl_2:
                scores[winner] += 2
            else:
                scores[winner] += 1
        return scores

    def board_heuristics(self, state):
        pucks_on_board = [0,0]
        alt_score = [0,0]

        width = ShuffleBoardGame.width
        length = ShuffleBoardGame.length
        
        for i in range(state.num_pucks):
            pos = state.get_x(i)
            if pos[0] >= 0 and pos[0] <= width and pos[1] <= length and pos[1] > 0:
                pucks_on_board[i%2] += 1
                if pos[1] > self.sl_3:
                    alt_score[i%2] += 3
                elif pos[1] > self.sl_2:
                    alt_score[i%2] += 2
                elif pos[1] > self.sl_1:
                    alt_score[i%2] += 1

        return pucks_on_board, alt_score

    def display_board(self):
        fig, ax = plt.subplots(figsize=(3, 6))
        for puck in self.board:
            if puck[2] == 0:
                ax.plot(puck[0], puck[1], 'bo')
            if puck[2] == 1:
                ax.plot(puck[0], puck[1], 'ro')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        plt.axis('square')
        plt.xlim([0, ShuffleBoardGame.width])
        plt.ylim([0, ShuffleBoardGame.length])
        plt.show()

    def run_sim_game(self):
        state = sim.State(self.num_pucks)
        p1 = players.SimPlayer(self)
        p2 = players.SimPlayer(self)
        # seed(42)
        # p2 = players.RandomPlayer()
        teams = {} # This should only be used for visualization
        for i in range(self.num_pucks):
            if i % 2 == 0:
                teams[i] = 'blue'
                # state, xs = self.sim_turn(state, i, self.width/2,0.1,0.0,1.1)
                state, xs = self.sim_turn(state, i, *p1.calc_move(0, state, i))
            else:
                teams[i] = 'red'
                # state, xs = self.sim_turn(state, i, 0,0,0,0)
                state, xs = self.sim_turn(state, i, *p2.calc_move(1, state, i))
            print(self.score_board(state))
            print(self.board_heuristics(state))
            sim.animate(xs, self.dt, self.length, self.width, teams)
            # sim.visualize_traj(xs[1:], fig, ax, self.length, self.width, teams)
        
        print(self.score_board(state))
        print(self.board_heuristics(state))

        plt.close('all')
        fig, ax = plt.subplots(figsize=(3, 6))
        for puck in range(self.num_pucks):
            pos = state.get_x(puck)
            if puck in teams.keys():
                ax.add_artist(plt.Circle(pos, self.puck_rad, color=teams[puck]))
            else:
                ax.add_artist(plt.Circle(pos, self.puck_rad, color="pink"))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        plt.axis('square')
        plt.xlim([0, self.width])
        plt.ylim([0, self.length])
        plt.axhline(y=self.sl_3)
        plt.axhline(y=self.sl_2)
        plt.axhline(y=self.sl_1)
        plt.show()

    def run_real_game(self):
        state = sim.State(self.num_pucks)
        ai = players.SimPlayer(self)

        teams = {0:'blue', 1:'red', 2:'blue', 3:'red', 4:'blue', 5:'red', 6:'blue', 7:'red'}
        for i in range(self.num_pucks):
            if i % 2 == 0:
                raw_input("Press enter once human shot is complete")
            else:
                robo_shot = ai.calc_move(1, state, i)
                _, xs = self.sim_turn(state, i, *robo_shot)
                sim.animate(xs, self.dt, self.length, self.width, teams)
                # shufflebot.perform_shot(robo_shot)
            puck_locs = get_real_board_state(game.length, game.width)
            state = sim.State(self.num_pucks)
            for idx, blue_puck in enumerate(puck_locs[0]):
                state.set_x(idx*2, blue_puck[::-1])
            for idx, red_puck in enumerate(puck_locs[1]):
                state.set_x(idx*2+1, red_puck[::-1])
        
        score = self.score_board(state)
        print("Human Score: " + str(score[0]))
        print("Robot Score: " + str(score[1]))

        plt.close('all')
        fig, ax = plt.subplots(figsize=(3, 6))
        for puck in range(self.num_pucks):
            pos = state.get_x(puck)
            if puck in teams.keys():
                ax.add_artist(plt.Circle(pos, self.puck_rad, color=teams[puck]))
            else:
                ax.add_artist(plt.Circle(pos, self.puck_rad, color="pink"))

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        plt.axis('square')
        plt.xlim([0, self.width])
        plt.ylim([0, self.length])
        plt.axhline(y=self.sl_3)
        plt.axhline(y=self.sl_2)
        plt.axhline(y=self.sl_1)
        plt.show()

if __name__ == '__main__':
    game = ShuffleBoardGame()
    game.run_real_game()
    # game.run_sim_game()

    # puck_locs = get_real_board_state(game.length, game.width)
    # state = sim.State(game.num_pucks)
    # for i in range(game.num_pucks):
    #     state.set_x(i, puck_locs[i%2][int(i/2)][::-1])

    # print(game.score_board(state))
    # print(game.board_heuristics(state))

    # teams = {0:'red', 1:'blue', 2:'red', 3:'blue', 4:'red', 5:'blue', 6:'red', 7:'blue'}
    
    # fig, ax = plt.subplots(figsize=(3, 6))
    # for puck in range(game.num_pucks):
    #     pos = state.get_x(puck)
    #     if puck in teams.keys():
    #         ax.add_artist(plt.Circle(pos, game.puck_rad, color=teams[puck]))
    #     else:
    #         ax.add_artist(plt.Circle(pos, game.puck_rad, color="pink"))
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # plt.axis('square')
    # plt.xlim([0, game.width])
    # plt.ylim([0, game.length])
    # plt.axhline(y=game.sl_3)
    # plt.axhline(y=game.sl_2)
    # plt.axhline(y=game.sl_1)
    # plt.show()
    