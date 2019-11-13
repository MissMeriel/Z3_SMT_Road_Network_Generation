#/usr/bin/python

from matplotlib import pyplot as plt

def main():
	with open('coordinates.txt') as f:
		content = f.readlines()
		print(content) #.split('\n'))
		X = []
		Y = []
		for line in content:
			digits = line.split(',')
			x = digits[0].replace(' ', '')
			x = x.replace('\n', '')
			X.append(int(x)/5.0)
			y = digits[2].replace(' ', '')
			y = y.replace('\n', '')
			Y.append(int(y)/6.0)

	plt.plot(X, Y, '-o')
	plt.show()

	with open('coordinates_scale5.txt', 'w') as f:
		for i in range(len(X)):
			print("%s, %s, %s\n" % (int(X[i]),  "0",  int(Y[i])))
			f.write("%s, %s, %s\n" % (int(X[i]),  "0",  int(Y[i])))


if __name__ == '__main__':
	main()
