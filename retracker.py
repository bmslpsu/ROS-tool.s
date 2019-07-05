import glob, os, time
import rospy , rosparam

class Retracker:
	
	def __init__(self):
		self.nodename	= 'kinefly'
		#self.yamlfile 	= os.path.expanduser(rospy.get_param(self.nodename + '/yamlfile', '~/%s.yaml' % self.nodename.strip('/')))
		self.yamlfile	= ''
		self.root 		= ''
		self.filespec 	= ''
		self.duration 	= 0
		self.save 		= ''
		self.files 		= {}
		self.n_file 	= 0
		self.file_path 	= ''
		self.file_root	= ''
		self.file_name 	= ''
		self.mask 		= ''
		
		os.environ["RETRACK"] = '1'  # set Kinefly to retrack
		
	def get_files(self,root,filespec):
	# Get all files in directory that fit filespec
		self.root 		= root
		self.filespec 	= filespec
		self.save 		= self.root + '/retrack'
		
		if not os.path.isdir(self.save):
			print "Making ", self.save
			print ""
			os.mkdir(self.save, 0755)
		else:
			print self.save, "exists"
		
		os.chdir(self.root)
		self.files = glob.glob(self.filespec)
		self.files.sort()
		self.n_file = len(self.files)
		
	def get_file_data(self, filepath):
	# Extract root, name, and extension from file path; make retrack file path
		self.file_path = filepath
		self.file_root = os.path.dirname(self.file_path)
		self.file_name = os.path.basename(self.file_path)
		self.file_name = os.path.splitext(self.file_name)[0]
		
		self.save = self.file_root + '/retrack'
		
		if not os.path.isdir(self.save):
			os.mkdir(self.save, 0755)
			
	def make_mask(self,root,filespec):
		self.get_files(root, filespec)
		self.mask = root + '/' + 'mask'
		
		if not os.path.isdir(self.mask):
			print "Making mask directory..."
			print "   " , self.mask
			os.mkdir(self.mask, 0755)
			
		os.environ["BAGFILE"] = self.root + '/' + self.files[0]  # set bagfile to 1st file fitting filespec
		os.system('roslaunch Kinefly main.launch &')  # run Kinefly
		time.sleep(20)
		
		#cp_cmd = 'cp ' + self.yamlfile + ' ' + self.mask
		#os.system('')
		
		print "Mask stored"

	def set_mask(self):
		self.params = rosparam.load_file(self.yamlfile)
		
	def retrack(self,filepath,duration):
		self.get_file_data(filepath)
		self.duration = duration
		
		record_cmd = 'roslaunch Kinefly record.launch ' + 'root:=' + self.save + ' prefix:=' + self.file_name + ' time:=' + str(
			self.duration) + ' &'
		
		os.environ["BAGFILE"] = self.file_path  # set bagfile
		print "   ", os.environ["BAGFILE"]
		
		os.system(record_cmd)  # start to record
		time.sleep(5)
		os.system('roslaunch Kinefly main.launch &')  # run Kinefly
		time.sleep(2*self.duration)
		os.system('rostopic pub - 1 kinefly/command Kinefly/MsgCommand exit')  # stop Kinefly
		os.system('killall -9 rosmaster')  # kill ros
		time.sleep(3)
		
		print "Done"
	
	def batch_retrack(self, root, filespec, duration):
		self.get_files(root, filespec)

		print "Retracking all ", filespec, " in ", root
		for fIdx in range(self.n_file):
			self.retrack(self.root + '/' + self.files[fIdx], duration)
		
		print('-------------------------DONE-------------------------')


if __name__ == '__main__':
	main = Retracker()
	#main.make_mask('/media/jean-michel/MyBook/EXPERIMENTS/Experiment_SOS_v2/bag', 'fly_10_*.bag')
	main.batch_retrack('/media/jean-michel/MyBook/EXPERIMENTS/Experiment_SOS_v2/bag','fly_10_*.bag', 45)
	# 'fly_2_*.bag', 30)
	#main.get_files('/media/jean-michel/MyBook/EXPERIMENTS/Experiment_SOS_v2/bag', 'fly_2_*.bag')
	#main.get_file_data(main.file_root + main.files[0])
	#main.retrack(main.root + '/' +  main.files[0],30)
	#main.make_mask('/media/jean-michel/MyBook/EXPERIMENTS/Experiment_SOS_v2/bag', 'fly_10_*.bag')