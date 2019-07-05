
from retrack_old import Retrack

RUN = Retrack()

root 		= '/media/jean-michel/My Book/EXPERIMENTS/Experiment_SOS_v2/bag'
filespec 	= 'fly_1_*.bag'
duration 	= 30

RUN.run(root, filespec, duration)