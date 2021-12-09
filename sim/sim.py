import itertools
import time

import matplotlib.pyplot as plt
import numpy as np

GRAVITY = 9.8
KINETIC_FRICTION = 0.1


class State:

    @classmethod
    def from_vec(cls, v):
        s = cls(len(v) // 4)
        s._state = v
        return s

    def __init__(self, num_pucks):
        self.num_pucks = num_pucks
        self._state = np.zeros(self.num_pucks * 4)

    def __mul__(self, other):
        return State.from_vec(self._state * other)

    def __add__(self, other):
        assert self.num_pucks == other.num_pucks
        return State.from_vec(self._state + other.get_state())

    def __sub__(self, other):
        assert self.num_pucks == other.num_pucks
        return State.from_vec(self._state - other.get_state())

    # def __str__(self):
    #     s = ""
    #     for puck in range(self.num_pucks):
    #         x = self.get_x(puck)
    #         dx = self.get_x_dot(puck)
    #         s += f"puck #{puck}\n   x: {x[0]}\n   y: {x[1]}\n  dx: {dx[0]}\n  dy: {dx[1]}\n"
    #     return s

    def __array__(self):
        return self._state

    def get_x(self, puck):
        return self._state[puck * 2:puck * 2 + 2]

    def get_x_dot(self, puck):
        return self._state[self.num_pucks * 2 + puck * 2:self.num_pucks * 2 + puck * 2 + 2]

    def get_all(self, puck):
        res = np.empty(4)
        res[0:2] = self.get_x(puck)
        res[2:4] = self.get_x_dot(puck)
        return res

    def set_x(self, puck, value):
        self._state[puck * 2:puck * 2 + 2] = value

    def set_x_dot(self, puck, value):
        self._state[self.num_pucks * 2 + puck * 2:self.num_pucks * 2 + puck * 2 + 2] = value

    def set_all(self, puck, value):
        self.set_x(puck, value[0:2])
        self.set_x_dot(puck, value[2:4])

    def get_state(self):
        return np.copy(self._state)


class ShuffleBoardSim:
    def __init__(self, dt, min_velocity=1e-2):
        # # board dimensions (feet)
        # self.length = 2
        # self.width = 0.5

        # mass in kg
        self.m = 0.27
        # radius in m
        self.r = 0.0265

        self.dt = dt
        self.min_velocity = min_velocity

def simulate(sim, initial_state, game, tol=1e-3):
    # for friction models see http://www.mate.tue.nl/mate/pdfs/11194.pdf
    def closed_loop(x, in_play):
        x_dot = State(x.num_pucks)

        def get_friction(v, m):
            friction = m * GRAVITY * KINETIC_FRICTION
            if np.linalg.norm(v) > 0:
                f = - friction * v / np.linalg.norm(v)
                return f
            return np.zeros_like(v)

        moving = []
        for puck in in_play:
            puck_x_dot = x.get_x_dot(puck)
            if puck_x_dot[0] >= sim.min_velocity or puck_x_dot[1] >= sim.min_velocity:
                moving.append(puck)

        for puck in moving:
            x_dot.set_x(puck, np.where(abs(x.get_x_dot(puck)) < sim.min_velocity, 0, x.get_x_dot(puck)))
            x_dot.set_x_dot(puck, get_friction(x_dot.get_x(puck), sim.m))

        for pair in itertools.combinations(in_play, 2):
            if pair[0] in moving or pair[1] in moving:
            # Collision physics
                p = x.get_x(pair[0]) - x.get_x(pair[1])
                distance_btw_pucks = np.linalg.norm(p)
                distance_btw_pucks_after_step = np.linalg.norm((x.get_x(pair[0]) + x_dot.get_x(pair[0]) * sim.dt) - (
                        x.get_x(pair[1]) + x_dot.get_x(pair[1]) * sim.dt))
                if distance_btw_pucks < 2 * sim.r and (distance_btw_pucks - distance_btw_pucks_after_step) > 0:
                    vref = x_dot.get_x(pair[1])
                    v_p2 = np.matmul(p, (x_dot.get_x(pair[0]) - vref)) / np.matmul(p, p) * p
                    v_p1 = (x_dot.get_x(pair[0]) - vref) - v_p2
                    x_dot.set_x_dot(pair[0], (v_p1 - x_dot.get_x(pair[0]) + vref) / sim.dt)
                    x_dot.set_x_dot(pair[1], (v_p2 - x_dot.get_x(pair[1]) + vref) / sim.dt)
        
        return x_dot

    in_play = []
    for i in range(initial_state.num_pucks):
        p = initial_state.get_x(i)
        if not ((p[0] == 0 and p[1] == 0) or \
            p[0] < -sim.r or p[0] > game.width + sim.r or \
            p[1] < -sim.r or p[1] > game.length + sim.r):
            in_play.append(i)
    xs = [initial_state]
    while True:
        x_dot = closed_loop(xs[-1], in_play)
        xs.append(xs[-1] + x_dot * sim.dt)
        if np.linalg.norm(xs[-1] - xs[-2]) < tol:
            return xs[-1], xs


def visualize(state, fig, ax, length, width, teams={}, r=0.0265):
    sl_3 = length * (23.0/24.0)
    sl_2 = length * (22.0/24.0)
    sl_1 = length * (12.0/24.0)
    for puck in range(state.num_pucks):
        pos = state.get_x(puck)
        if puck in teams.keys():
            ax.add_artist(plt.Circle(pos, r, color=teams[puck]))
        else:
            ax.add_artist(plt.Circle(pos, r, color="pink"))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.axis('square')
    plt.xlim([0, width])
    plt.ylim([0, length])
    plt.axhline(y=sl_3)
    plt.axhline(y=sl_2)
    plt.axhline(y=sl_1)


def animate(states, dt, length, width, teams={}, r=0.0265):
    fig, ax = plt.subplots(figsize=(3, 6))
    less_states = states[0::20]
    for state in less_states:
        visualize(state, fig, ax, length, width, teams, r)
        plt.pause(dt)
        ax.clear()


def visualize_traj(states, fig, ax, length, width, teams={}, r=0.0265):
    states_arr = np.array([np.array(state) for state in states])
    for puck in range(states[0].num_pucks):
        ax.plot(states_arr[:, puck * 2], states_arr[:, puck * 2 + 1])
        pos = states[-1].get_x(puck)
        if puck in teams.keys():
            ax.add_artist(plt.Circle(pos, r, color=teams[puck]))
        else:
            ax.add_artist(plt.Circle(pos, r, color="pink"))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.axis('square')
    plt.xlim([0, width])
    plt.ylim([0, length])
    # plt.axhline(y=length * (15.0/16.0))
    # plt.axhline(y=length * (14.0/16.0))
    # plt.axhline(y=length * (10.0/16.0))


if __name__ == '__main__':
    dt = 0.01
    board = ShuffleBoardSim(dt)
    initial_state = State(6)

    teams = {
        0: 'red',
        1: 'blue',
        2: 'red',
        3: 'blue',
        4: 'red',
        5: 'blue',
    }

    # initial_state.set_x(0, np.array([0.48, 0.1]))
    # initial_state.set_x_dot(0, np.array([0, 1.03]))
    # initial_state.set_x(1, np.array([0.5, 1]))
    # initial_state.set_x_dot(1, np.array([0, 0]))
    # initial_state.set_x(2, np.array([0.6, 1.2]))
    # initial_state.set_x_dot(2, np.array([0, 0]))
    # initial_state.set_x(3, np.array([0.73, 1.4]))
    # initial_state.set_x_dot(3, np.array([0, 0]))
    # initial_state.set_x(4, np.array([0.5, 1.6]))
    # initial_state.set_x_dot(4, np.array([0, 0]))
    # initial_state.set_x(5, np.array([0.5, 1.8]))
    # initial_state.set_x_dot(5, np.array([0, 0]))

    initial_state.set_x(0, np.array([0.5, 1.8]))
    initial_state.set_x_dot(0, np.array([0, 0]))
    initial_state.set_x(1, np.array([0.5, 1]))
    initial_state.set_x_dot(1, np.array([0, 0]))
    initial_state.set_x(2, np.array([0.6, 1.2]))
    initial_state.set_x_dot(2, np.array([0, 0]))
    initial_state.set_x(3, np.array([0.73, 1.4]))
    initial_state.set_x_dot(3, np.array([0, 0]))
    initial_state.set_x(4, np.array([0.5, 1.6]))
    initial_state.set_x_dot(4, np.array([0, 0]))
    initial_state.set_x(5, np.array([0.48, 0.1]))
    initial_state.set_x_dot(5, np.array([0, 1.03]))

    start = time.time()
    xf, xs = board.simulate(initial_state)
    print("Solve time:", time.time() - start)

    print(xf)
    fig, ax = plt.subplots(figsize=(12, 6))
    visualize_traj(xs, fig, ax, teams)
    plt.show()
    # animate(xs, dt, teams)
