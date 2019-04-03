# Created by Minbiao Han and Roman Sharykin
# CS6501-003 Spring 2019
# Ref: The SMT_Based Automatic Road Network Generation in Vehicle Simulation Environment (BaekGyu Kim et al. 2016)

from z3 import *

# Curve Coverage Criteria [n_min, n_max, theta_min, theta_max, d_min, d_max]
# n_min: min number of curves
# n_max: max number of curves
# theta_min: min curvature of each curve
# theta_max: max curvature of each curve
# d_min: min distance of any two adjacent curves
# d_max: max distance of any two adjacent curves

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

#s = Solver()


def abs(x):
    return If(x >= 0, x, -x)

#####################################
# Complete the code
# Q1: Road Segment Generation Algorithm
#####################################
# Note: Curves need to be generated with curves in order across a certain axis
# Constraints:
# ∀ 0 ≤ i ≤ n − 1, dmin ≤ xi+1 − xi ≤ dmax
# ∀ 0 ≤ i ≤ n − 1, dmin ≤ yi+1 − yi ≤ dmax
# ∀ 0 ≤ i ≤ n − 1, dmin ≤ zi+1 − zi ≤ dmax

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
        # global s
        cnt = 0
        S = None
        tmax = 1
        s.check()
        set_option(timeout=tmax)
        #print('X: '+str(X))
        print("\nROAD SEG GEN")
        # s.add(n_min >= 0)
        # s.add(n_max >= 0, n_max >= n_min)
        # s.add([d_min >= 0, d_max >= 0, d_max >= d_min, theta_min >= 0, theta_max >= 0, theta_max >= theta_min])
        
        for index in range(0, len(X)):
            s.add(X[index] >= x_min, X[index] <= x_max)
            s.add(Y[index] >= y_min, Y[index] <= y_max)
            s.add(Z[index] >= z_min, Z[index] <= z_max)
            #curve distance constraint
            if(index < len(X)-1):
                s.add((X[index+1]-X[index]) <= d_max)
                s.add((X[index+1]-X[index]) >= d_min)
                s.add((Y[index+1]-Y[index]) <= d_max)
                s.add((Y[index+1]-Y[index]) >= d_min)
                s.add((Z[index+1]-Z[index]) <= d_max)
                s.add((Z[index+1]-Z[index]) >= d_min)
            
            if(index < len(X)-3):
                #alternating road constraint
                fml1 = ((Y[index+1]-Y[index])/(X[index+1]-X[index])) * ((Y[index+2]-Y[index+1])/(X[index+2]-X[index+1]))
                #s.add(fml1 <= 0)
                #curvature constraint
                fml2 = ((Z[index+1]-Z[index])/(X[index+1]-X[index])) * ((Z[index+2]-Z[index+1])/(X[index+2]-X[index+1]))
                s.add(theta_min <= abs(fml1), theta_max >= abs(fml1))
                s.add(theta_min <= abs(fml2), theta_max >= abs(fml2))
        try:
            while(cnt < N):
                print("ASSUMPTIONS:\n"+str(s))
                print(cnt)
                # try to solve with the coverage criteria
                result = s.check()
                print(result)
                r_i = s.model()
                print("MODEL:\n"+str(r_i))
                #print("s.check() == sat: "+str(s.check() == sat))
                if(result == sat):
                    print('s.check was sat')
                    #print(s.model())
                    r_i = s.model()
                    S = r_i
                    #s.add(Not(r_i))
                    return r_i
                elif(str(s.check()) == str(unknown)):
                    raise NoSol()
                #elif(s.check() == timeout):
                #	raise GlobalTimeOut()
                cnt += 1
        except GlobalTimeOut as gto:
                print(gto.value)
        except NoSol as ns:
                print(ns.value)
        except Exception as e:
                print(e)
        return S


#####################################
# Complete the code
# Q2: Coverage Consistency Check
#####################################
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


# Sequential Road Network Information
# Each array is a 2D array
# e.g. solArrayX = [[1, 2, 3], [4, 5, 6]] means the road network has 2 segments and each segment has 3 waypoints.
#      The X-coordinates of the 3 waypoints of Segment1 are 1, 2, and 3, the X-coordinates of the 3 waypoints of Segment2 are 4, 5, and 6.
# For your outputs, the size of your solArrayX/Y/Z will be (solN * N), which means your road network has solN segments, and each segment has N waypoints
solArrayX = []
solArrayY = []
solArrayZ = []
curNumArray = []


def main():
    # global s
    # Coverage consistency check
    cov_con = coverage_consistence(C_l, C_g, solN)
    if(cov_con == False):
        print('No-Sol\n')
        return 0
    else:
        #####################################
        # Complete the code
        # Q3: Sequential Road Network Generation Algorithm
        #####################################
        # generate the first segment
        N = 1
        last_x = -1
        last_y = -1
        last_z = -1
        second_to_last_x = -1
        second_to_last_y = -1
        second_to_last_z = -1
        for sol_ind in range(solN):
            print('\nsol_ind: '+str(sol_ind))
            X = []
            Y = []
            Z = []
            C_i = C_l[sol_ind]
            s = Solver()
            s.reset()
            s.check()

            #find n
            n_min = int(C_i[0])
            n_max = int(C_i[1])
            n = Int('n')
            s.add(n >= n_min)
            s.add(n <= n_max)
            #print(s.assertions())
            s.check()
            mdl = s.model()
            n = mdl[n].as_long()
            s.reset()
            s.check()
            X += [Int('x_%d' %index) for index in range(0, n)]
            Y += [Int('y_%d' %index) for index in range(0, n)]
            Z += [Int('z_%d' %index) for index in range(0, n)]

            theta_min = C_i[2]
            theta_max = C_i[3]
            d_min = C_i[4]
            d_max = C_i[5]
            print(X)
            #exit()

            #constrain endpoint of last segment to match start of this segment
            print('last_x >= 0: '+str(last_x >= 0))
            if(last_x > -1):
                s.add(X[0] == last_x, Y[0] == last_y, Z[0] == last_z)
                print('last_x/y/z: '+str(last_x)+", "+str(last_y)+", "+str(last_z))
                #global theta constraints
                fml1 = (Y[0]-second_to_last_y) / (X[0]-second_to_last_x) - (Y[1]-last_y) / (X[1]-last_x)
                #s.add(C_g[2] <= abs(fml1), C_g[3] >= abs(fml1))
                fml2 = (Z[0]-second_to_last_z) / (X[0]-second_to_last_x) - (Z[1]-last_z) / (X[1]-last_x)
                #s.add(C_g[2] <= abs(fml2), C_g[3] >= abs(fml2))
            print("FORMULA:\n"+str(s))
            outcome = road_seg_gen(X, Y, Z, x_min, y_min, z_min, x_max, y_max, z_max, d_min, d_max, theta_min, theta_max, N, s)
            print()
            print(outcome)
            #print(Int('x_%d' %(n-1)))
            # update theta continuity constraints
            last_x = outcome[Int('x_%d' %(n-1))].as_long()
            last_y = outcome[Int('y_%d' %(n-1))].as_long()
            last_z = outcome[Int('z_%d' %(n-1))].as_long()
            second_to_last_x = outcome[Int('x_%d' %(n-2))].as_long()
            second_to_last_y = outcome[Int('y_%d' %(n-2))].as_long()
            second_to_last_z = outcome[Int('z_%d' %(n-2))].as_long()
            for index in range(n):
                    id = Int('x_%d' %index)
                    solArrayX.append(outcome[id])
                    id = Int('y_%d' %index)
                    solArrayY.append(outcome[id])
                    id = Int('z_%d' %index)
                    solArrayZ.append(outcome[id])
            print(solArrayX)
            print(solArrayY)
            print(solArrayZ)
        #exit()

    # Write the generated road information into a file
    file = open("coordinates.txt", "w")
    for seg in range(solN):
        file.write('Seg ' + str(seg + 1) + ':\n')
        for point in range(curNumArray[seg]):
            file.write(
                str(30 * int(str(solArrayY[seg][point]))) + ", 0, " + str(30 * int(str(solArrayZ[seg][point]))) + "\n")
    file.close()


    # DO NOT CHANGE THIS PART OF THE CODE
    # Additional part for generating the correct output format for visualising the generated road in Unity

    # if you wish to view the generated coordinates without alterations for Unity, just comment out
    # this last part up to if __name__ == '__main__':
    with open('coordinates.txt') as f:
        content = f.readlines()

    content = [x.strip() for x in content]
    last = ''
    to_remove = []
    for cont in content:
        if 'Seg' in cont:
            to_remove.append(cont)

    for rem in to_remove:
        content.remove(rem)

    to_remove = []
    for cont in content:
        if last == cont:
            to_remove.append(cont)
        last = cont

    for rem in to_remove:
        content.remove(rem)

    with open('coordinates.txt', 'w') as f:
        for item in content:
            f.write("%s\n" % item)


if __name__ == '__main__':
    main()