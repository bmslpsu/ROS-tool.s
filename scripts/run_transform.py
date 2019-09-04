
from bag_transform import BagTransform

RUN = BagTransform()

root 		= '/media/jean-michel/MyBook/EXPERIMENTS/Experiment_ChirpLog_HeadFree/Vid'
vid_name 	= 'vidData'
time_name 	= 't_v'
debug 		= False

RUN.batch_mat(root, vid_name, time_name, debug)