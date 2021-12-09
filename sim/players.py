import copy
import time
from random import uniform
from contextlib import closing

import numpy as np
import multiprocessing as mp
from scipy.ndimage import gaussian_filter

import sim

class Player:
    def __init__(self, game):
        self.game = game
        self.player_sim = sim.ShuffleBoardSim(0.01)
        self.start_x = game.width/2
        self.start_y = 0.1

    def calc_move(self, player, state, puck, use_mp=True):
        raise NotImplementedError("Player should be subclassed")

class RandomPlayer(Player):
    def calc_move(self, player, state, puck, use_mp=True):
        return self.start_x, self.start_y, uniform(-0.25, 0.25), uniform(1.0, 1.1)

class SimPlayer(Player):
    def calc_move(self, player, state, puck, use_mp=True):
        start = time.time()

        turn = int(puck/2)
        if player == 0:
            num_puck_div = 4
            true_score_div = [5,3,2,1]
            alt_score_div = [2,2,3,3]
        else:
            num_puck_div = 4
            true_score_div = [5,3,2,1]
            alt_score_div = [2,2,3,5]
    
        poss_x_pos = np.array([self.game.width/4, self.game.width/2, 3*self.game.width/4])
        poss_x_vel = np.arange(-0.18, 0.19, .01)
        poss_y_vel = np.arange(1.2, 1.27, .01)

        shots = []
        for x_pos in poss_x_pos:
            for x_vel in poss_x_vel:
                for y_vel in poss_y_vel:
                    shots.append((x_pos, x_vel, y_vel))

        shot_scores = np.empty((len(poss_x_pos), len(poss_x_vel), len(poss_y_vel)))
        with closing(mp.Pool(processes=12)) as pool:
            results = []
            for shot in shots:
                copied_state = copy.deepcopy(state)
                copied_state.set_all(puck, np.array([shot[0], self.start_y, shot[1], shot[2]]))
                results.append((pool.apply_async(sim.simulate, args=(self.player_sim, copied_state, self.game)), shot))

            pool.close()
            
            for r in results:
                xf = r[0].get()[0]

                score = self.game.score_board(xf)
                # Number of pucks in play on board, score with all pucks counted
                num_pucks, alt_score = self.game.board_heuristics(xf)

                scaled_score_diff = (score[0] - score[1])/true_score_div[turn] # Usually max 3
                scaled_alt_score_diff = (alt_score[0] - alt_score[1])/alt_score_div[turn] # Max possible 12
                scaled_num_pucks_diff = (num_pucks[0] - num_pucks[1])/num_puck_div # Max possible 4

                x_pos_loc = np.searchsorted(poss_x_pos, r[1][0])
                x_vel_loc = np.searchsorted(poss_x_vel, r[1][1])
                y_vel_loc = np.searchsorted(poss_y_vel, r[1][2])
                shot_scores[x_pos_loc,x_vel_loc,y_vel_loc] = scaled_score_diff + scaled_alt_score_diff + scaled_num_pucks_diff

            np.random.seed(3)
            if player == 1:
                shot_scores = -shot_scores

            np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
            # print(shot_scores)
            shot_scores = gaussian_filter(shot_scores, sigma=(0,1,1), mode='constant')

            max_shot_val = np.max(shot_scores)
            # print(max_shot_val)
            max_shot_choices = np.isclose(shot_scores, max_shot_val, atol=0.01).nonzero()
            # print(max_shot_choices)
            choice = np.random.randint(len(max_shot_choices[0]))
            chosen_x_pos = poss_x_pos[max_shot_choices[0][choice]]
            chosen_x_vel = poss_x_vel[max_shot_choices[1][choice]]
            chosen_y_vel = poss_y_vel[max_shot_choices[2][choice]]
            best_shot = (chosen_x_pos, chosen_x_vel, chosen_y_vel)

            pool.join()
            pool.terminate()


        print("Shot Time: " + str(time.time() - start))
        # print("Predicted Score: " + str(shot_scores.max()))
        # print("Velocity: " + str(best_vel))

        return best_shot[0], self.start_y, best_shot[1], best_shot[2]

# class QSimPlayer(Player):
#     def calc_move(self, player, game, state, puck, use_mp=True):
#         start = time.time()
#         model = load_model('q_model')
    
#         vels = []
#         for x_vel in np.arange(-0.25, 0.25, .05):
#             for y_vel in np.arange(0.4, 1.2, .05):
#                 vels.append((x_vel, y_vel))

#         player_sim = sim.ShuffleBoardSim(0.01)

#         if use_mp:
#             with mp.Pool(processes=12) as pool:
#                 results = []
#                 for vel in vels:
#                     copied_state = copy.deepcopy(state)
#                     copied_state.set_all(puck, np.array([0.5, 0.1, vel[0], vel[1]]))
#                     results.append((pool.apply_async(player_sim.simulate, args=(copied_state,)), vel)) # Result and vel tuple

#                 pool.close()

#                 all_new_locations = []
#                 result_vels = []
#                 for r in results:
#                     xf = r[0].get()[0]
                    
#                     new_locations = []
#                     for i in range(6):
#                         new_locations.extend(xf.get_x(i))
#                     all_new_locations.append(new_locations)
#                     result_vels.append(r[1])

#                 shot_scores = model.predict(all_new_locations).flatten()
#                 if player == 1:
#                     shot_scores = -shot_scores
#                 best_vel = result_vels[shot_scores.argmax()]

#                 pool.join()
#         else:
#             all_new_locations = []
#             for vel in vels:
#                 copied_state = copy.deepcopy(state)
#                 copied_state.set_all(puck, np.array([0.5, 0.1, vel[0], vel[1]]))

#                 xf, _ = player_sim.simulate(copied_state)
#                 new_locations = []
#                 for i in range(6):
#                     new_locations.extend(xf.get_x(i))
#                 all_new_locations.append(new_locations)

#             shot_scores = model.predict(all_new_locations).flatten()
#             if player == 1:
#                 shot_scores = -shot_scores
#             best_vel = vels[shot_scores.argmax()]


#         # print("Shot Time: " + str(time.time() - start))
#         # print("Predicted Score: " + str(shot_scores.max()))
#         # print("Velocity: " + str(best_vel))

#         return 0.5, 0.1, best_vel[0], best_vel[1]