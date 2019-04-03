from z3 import *
import numpy as np 

# Curve Coverage Criteria [n_min, n_max, theta_min, theta_max, d_min, d_max]
# n_min: min number of curves
# n_max: max number of curves
# theta_min: min curvature of each curve
# theta_max: max curvature of each curve
# d_min: min distance of any two adjacent curves
# d_max: max distance of any two adjacent curves

def abs(x):
    return If(x >= 0, x, -x)

class NoSol(Exception):
	def __init__(self, value='No solution found for given criteria.'):
		self.value = value
	def __str__(self):
		return repr(self.value)

class GlobalTimeOut(Exception):
	def __init__(self, value='Global timeout reached.'):
		self.value = value
	def __str__(self):
		return repr(self.value)

def road_seg_gen(X, Y, Z, x_min, y_min, z_min, x_max, y_max, z_max, d_min, d_max, theta_min, theta_max, N, s):
	cnt = 0
	S = None # Set()
	phi_C = None
	tmax = 5
	# s = Solver()
	set_option(timeout=1)
	for index in range(0, len(X)):
		s.add(X[index] >= x_min, X[index] <= x_max)
		s.add(Y[index] >= y_min, Y[index] <= y_max)
		s.add(Z[index] >= z_min, Z[index] <= z_max)
		if(index < len(X)-1):
			s.add((X[index+1]-X[index]) <= d_max)
			s.add((X[index+1]-X[index]) >= d_min)
			s.add((Y[index+1]-Y[index]) <= d_max)
			s.add((Y[index+1]-Y[index]) >= d_min)
			s.add((Z[index+1]-Z[index]) <= d_max)
			s.add((Z[index+1]-Z[index]) >= d_min)
	print(s.check())

	try:
		while(cnt < N):
			print(cnt)
			# try to solve with the coverage criteria
			# r_i = Solver(phi_C, tmax)
			print(s.check())
			print(s.model())
			r_i = s.model()
			if(s.check() == sat):
				print('s.check was sat')
				S = r_i # .append(r_i) # S.update(r_i)
				#s.add(Not(r_i))
				return r_i
			elif(s.check() == unsat):
				raise NoSol()
			elif(r_i == 'timeout'):
				raise GlobalTimeOut()
			cnt += 1
	except GlobalTimeOut as gto:
		print(gto.value)
	except NoSol as ns:
		print(ns.value)
	except Exception as e:
		print(e)
	return S


def coverage_consistence(C_l, C_g, K):
	# consistency check for criteria of N
	sum_local_mins = 0
	for index in range(0, len(C_l)):
		sum_local_mins += C_l[index][0]
	sum_local_maxes = 0
	for index in range(0, len(C_l)):
		sum_local_maxes += C_l[index][1]
	theta_mins = []
	theta_mins += [C_l[index][2] for index in range(0, len(C_l))]
	theta_maxes = []
	theta_maxes += [C_l[index][3] for index in range(0, len(C_l))]
	d_mins = []
	d_mins += [C_l[index][4] for index in range(0, len(C_l))]
	d_maxes = []
	d_maxes += [C_l[index][5] for index in range(0, len(C_l))]
	if(sum_local_mins + K-1 < C_g[0]):
		return False
	elif(sum_local_maxes + K-1 > C_g[1]):
		return False
	# consistency check for criteria of theta
	elif(min(theta_mins) < C_g[2]):
		return False
	elif(max(theta_maxes) > C_g[3]):
		return False
	# consistency check for criteria of D
	elif(min(d_mins) < C_g[4]):
		return False
	elif(max(d_maxes) > C_g[5]):
		return False
	else:
		return True


def main():
	s = Solver()
	s.reset()
	print(s)
	n = Int('n')
	n_min = 4
	n_max = 7
	s.add(n >= n_min, n <= n_max)
	print(s)
	print(s.check())
	print(s.model())

	s = Solver()
	s.reset()
	print(s)
	n = Int('n')
	n_min = 2
	n_max = 7
	s.add(n >= n_min, n <= n_max)
	print(s)
	print(s.check())
	print(s.model())
	exit()

	C_g = [5, 100, 0, 35, 5, 22]  # global coverage criteria

	C_l = []  # local coverage criteria
	solN = 5
	# Adding local coverage criteria
	C_l.append([4, 7, 0, 1, 10, 14])
	C_l.append([2, 7, 1, 3, 11, 15])
	C_l.append([3, 9, 0, 2, 8, 12])
	C_l.append([3, 8, 1, 2, 12, 20])
	C_l.append([2, 10, 0, 3, 10, 15])

	# Boundary Information
	x_min = 0
	y_min = 0
	z_min = 0
	x_max = 300
	y_max = 300
	z_max = 300

	s = Solver()
	K = solN
	consistent = coverage_consistence(C_l, C_g, K)
	print(consistent)
	# exit()

	X = []
	Y = []
	Z = []
	d_min = Int('d_min')
	d_max = Int('d_max')
	theta_min = Int('theta_min')
	theta_max = Int('theta_max')
	n_min = Int('n_min')
	n_max = Int('n_max')
	n = Int('n')
	s.add(n_min >= 0)
	s.add(n_max >= 0, n_max >= n_min)
	s.add(d_min >= 0)
	s.add(d_max >= 0, d_max >= d_min)
	s.add(theta_min >= 0)
	s.add(theta_max >= 0, theta_max >= theta_min)
	n_maxes = [] # np.array(len(C_l))
	theta_maxes = []
	d_maxes = []
	for index in range(0, len(C_l)):
		# print(C_l[index][1])
		n_maxes.append(C_l[index][1])
		theta_maxes.append(C_l[index][3])
		d_maxes.append(C_l[index][5])
		#s.add(n_max <= C_l[index][1])
		#s.add(theta_max <= C_l[index][3])
		#s.add(d_max <= C_l[index][5])

		s.add(n_min >= C_l[index][0])
		s.add(theta_min >= C_l[index][2])
		s.add(d_min >= C_l[index][4])
	s.add(n_max == min(n_maxes))
	s.add(theta_max == min(theta_maxes))
	s.add(d_max == min(d_maxes))
	s.add(n >= n_min, n <= n_max)
	print(s.check())
	model = s.model()
	# print(model)
	n_min = model[n_min]
	d_min = model[d_min].as_long()
	# print(type(d_min))
	d_max = model[d_max].as_long()
	n = model[n].as_long()
	theta_min = model[theta_min].as_long()
	theta_max = model[theta_max].as_long()
	
	X += [Int('x_%d' %index) for index in range(0, n)]
	Y += [Int('y_%d' %index) for index in range(0, n)]
	Z += [Int('z_%d' %index) for index in range(0, n)]
	'''
	for index in range(0, n):
		X.append(Int('x_%d' %index))
		Y.append(Int('y_%d' %index))
		Z.append(Int('z_%d' %index))
	# s = Solver()

	for index in range(0, len(X)):
		s.add(X[index] >= x_min, X[index] <= x_max)
		s.add(Y[index] >= y_min, Y[index] <= y_max)
		s.add(Z[index] >= z_min, Z[index] <= z_max)
		if(index < len(X)-1):
			s.add((X[index+1]-X[index]) <= d_max)
			s.add((X[index+1]-X[index]) >= d_min)
			s.add((Y[index+1]-Y[index]) <= d_max)
			s.add((Y[index+1]-Y[index]) >= d_min)
			s.add((Z[index+1]-Z[index]) <= d_max)
			s.add((Z[index+1]-Z[index]) >= d_min)
	print(s.check())
	print(s.model())
	'''

	try:
		print('Trying with n_min')
		N =1 # n_min.as_long()
		outcome = road_seg_gen(X, Y, Z, x_min, y_min, z_min, x_max, y_max, z_max, d_min, d_max, theta_min, theta_max, N, s)
	except:
		print('Trying with n_max')
		N = n_max
		outcome = road_seg_gen(X, Y, Z, x_min, y_min, z_min, x_max, y_max, z_max, d_min, d_max, theta_min, theta_max, N)
	print(outcome)
	#print(outcome.model())


if __name__ == '__main__':
    main()