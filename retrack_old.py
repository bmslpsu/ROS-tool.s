
import  glob , os , time

class Retracker:
	
	def __init__(self):
		self.root 		= ''
		self.filespec 	= ''
		self.duration 	= 0
		self.save 		= ''
		self.files		= {}
		self.n_file 	= {}

		os.environ["RETRACK"] = '1' # set Kinefly to retrack
		
	def run(self,root,filespec,duration):
		self.root 		= root
		self.filespec 	= filespec
		self.duration	= duration
		self.save 		= self.root + '/retrack'
		
		if not os.path.isdir(self.save):
			print "Making " , self.save
			os.mkdir(self.save, 0755)
		else:
			print self.save , "exists"
		
		os.chdir(self.root)
		self.files 	= glob.glob(self.filespec)
		self.files.sort()
		self.n_file	= len(self.files)
		
		print "Retracking all ", filespec, " in ", root
		for fIdx in range(self.n_file):
			fname 		= os.path.splitext(self.files[fIdx])[0]
			bagfile 	= self.root + '/' + self.files[fIdx]
			record_cmd 	= 'roslaunch Kinefly record.launch ' + 'root:=' + self.save + ' prefix:=' + fname + ' time:=' + str(self.duration) + ' &'
			
			os.environ["BAGFILE"] = bagfile # set bagfile
			
			print "   " , os.environ["BAGFILE"]
			
			os.system(record_cmd) # start to record
			time.sleep(5)
			os.system('roslaunch Kinefly main.launch &') # run Kinefly
			time.sleep(self.duration + 20)
			os.system('rostopic pub - 1 kinefly/command Kinefly/MsgCommand exit') # stop Kinefly
			#os.system('rosnode kill /' + bagfile )
			os.system('killall -9 rosmaster')  # kill ros
			time.sleep(3)
			
			print('done')
		
		print('-------------------------DONE-------------------------')
		

if __name__ == '__main__':
	main = Retracker()
	main.run('/media/jean-michel/MyBook/EXPERIMENTS/Experiment_SOS_v2/bag','fly_2_*.bag',30)