import time
from datetime import timedelta
from experiment import Experiment
from maze3D_new.Maze3DEnv import Maze3D
from maze3D_new.assets import *
from rl_models.sac_agent import Agent
from rl_models.sac_discrete_agent import DiscreteSACAgent
from rl_models.utils import get_config, get_plot_and_chkpt_dir, get_sac_agent
from maze3D.utils import save_logs_and_plot
import sys

"""
The code of this work is based on the following github repos:
https://github.com/kengz/SLM-Lab
https://github.com/EveLIn3/Discrete_SAC_LunarLander/blob/master/sac_discrete.py
"""

def main(argv):
    # get configuration
    config = get_config(argv[0])

    # creating environment
    maze = Maze3D(config_file=argv[0])

    chkpt_dir, load_checkpoint_name = [None, None]
    if config["game"]["save"]:
        # create the checkpoint and plot directories for this experiment
        chkpt_dir, plot_dir, load_checkpoint_name = get_plot_and_chkpt_dir(config)

    # create the SAC agent
    sac = get_sac_agent(config, maze, chkpt_dir)

    # create the experiment
    experiment = Experiment(maze, sac, config=config)
    start_experiment = time.time()

    # set the goal
    goal = config["game"]["goal"]

    # training loop
    loop = config['Experiment']['loop']
    if loop == 1:
        # Experiment 1
        experiment.loop_1(goal)
    else:
        # Experiment 2
        experiment.loop_2(goal)

    end_experiment = time.time()
    experiment_duration = timedelta(seconds=end_experiment - start_experiment - experiment.duration_pause_total)

    print('Total Experiment time: {}'.format(experiment_duration))

    if config["game"]["save"]:
        # save training logs to a pickle file
        experiment.df.to_pickle(plot_dir + '/training_logs.pkl')

        if not config['game']['test_model']:
            total_games = experiment.max_episodes if loop == 1 else experiment.game
            # save rest of the experiment logs and plot them
            save_logs_and_plot(experiment, chkpt_dir, plot_dir, total_games)
            experiment.save_info(chkpt_dir, experiment_duration, total_games, goal)
    pg.quit()


if __name__ == '__main__':
    main(sys.argv[1:])
    exit(0)
