from gameObjects import *
from assets import *
from utils import checkTerminal
import random

layout = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
          [1, 0, 0, 0, 0, 0, 0, 1, 0, 1],
          [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
          [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
          [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
          [1, 0, 0, 1, 0, 0, 0, 0, 0, 1],
          [1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
          [1, 1, 0, 0, 0, 0, 0, 0, 2, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]


class ActionSpace:
    def __init__(self):
        self.actions = list(range(0, 14 + 1))
        self.shape = 1
        self.high = self.actions[-1]
        self.low = self.actions[0]

    def sample(self):
        # return [random.sample([0, 1, 2], 1), random.sample([0, 1, 2], 1)]
        return np.random.randint(self.low, self.high, 1)


class Maze3D:
    def __init__(self):
        self.board = GameBoard(layout)
        self.keys = {pg.K_UP: 1, pg.K_DOWN: 2, pg.K_LEFT: 4, pg.K_RIGHT: 8}
        self.running = True
        self.done = False
        self.observation = self.get_state()  # must init board fisrt
        self.action_space = ActionSpace()
        self.observation_shape = (len(self.observation),)

    def step(self, action, timedout):
        self.board.handleKeys(action)  # action is int
        self.board.update()
        glClearDepth(1000.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.board.draw()
        pg.display.flip()
        clock.tick()
        fps = clock.get_fps()
        pg.display.set_caption("Running at " + str(int(fps)) + " fps")
        self.observation = self.get_state()
        if checkTerminal(self.board.ball):
            self.done = True
        reward = self.reward_function_maze(timedout)
        return self.observation, reward, self.done

    def get_state(self):
        # [ball pos x | ball pos y | ball vel x | ball vel y|  theta(x) | phi(y) ]
        return np.asarray(
            [self.board.ball.x, self.board.ball.y, self.board.ball.velocity[0], self.board.ball.velocity[1], self.board.rot_x, self.board.rot_y])

    def reset(self):
        self.__init__()
        return self.observation

    def reward_function_maze(self, timedout):
        # For every timestep -1
        # Timed out -50
        # Reach goal +100
        if self.done:
            # solved
            return 100
        # if not done and timedout
        if timedout:
            return -50
        # return -1 for each time step
        return -1
