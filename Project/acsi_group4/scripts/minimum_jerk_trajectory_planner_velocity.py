import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def trajectory_planner(current_position, destination, current_velocity, desired_velocity, moving_time, frequency = 1000):
	trajectory = []
	trajectory_derivative = []
	time_stamp = []
	timefreq = moving_time * frequency
	next_postion = 0
	next_derivative = 0
	current_position = [float(current_position[i]) for i in range(len(current_position))]
	destination = [float(destination[i]) for i in range(len(current_position))]
	# timefreq = float(timefreq)

	for time in range(timefreq):
		time = float(time)
		timefreq = float(timefreq)
		increment1 = np.dot(np.subtract(destination, current_position),
					(10.*(time/timefreq)**3 - 15.*(time/timefreq)**4 + 6.*(time/timefreq)**5))
		increment2 = np.dot(np.dot(np.subtract(desired_velocity,current_velocity), moving_time*time/timefreq),
									 (6.*(time/timefreq)**2-8.*(time/timefreq)**3+3.*(time/timefreq)**4))
		# print(increment2)
		increment = increment1 + increment2
		next_postion = np.add(np.add(current_position,np.dot(current_velocity, moving_time*time/timefreq)), increment)
		trajectory.append(next_postion)

		increment_derivative1 = frequency * (30.*(time**2)*(1/timefreq)**3 \
							-60.*(time**3)*(1/timefreq)**4 + 30.*(time**4)*(1/timefreq)**5)
		increment_derivative2 = 18*(time/timefreq)**2 - 32 * (time/timefreq)**3 + 15*(time/timefreq)**4
		next_derivative = np.add(np.dot(destination, increment_derivative1),
									np.dot(np.subtract(desired_velocity, current_velocity), increment_derivative2))
		next_derivative = np.add(next_derivative,current_velocity)
		trajectory_derivative.append(next_derivative)
		# print(next_derivative)
		time_stamp.append(time/frequency)

	return time_stamp, trajectory, trajectory_derivative

if __name__ == "__main__":
	current_position = [0,0,0]
	destination = [2,2,2]
	current_velocity = [0.5,0.5,0.5]
	desired_velocity = [0.0,0.0,0.0]
	frequency = 1000
	moving_time = 3
	time, trajectory, trajectory_derivative = trajectory_planner(current_position, destination,current_velocity, desired_velocity, moving_time, frequency)
	# print([trajectory[i][0] for i in range(frequency * moving_time)])
	# print(time)
	# plt.figure()
	# plt.plot(time,[trajectory[i][0] for i in range(frequency * moving_time)])
	# plt.savefig('trajectory_x.png')
	# plt.plot(time,[trajectory[i][1] for i in range(frequency * moving_time)])
	# plt.savefig('trajectory_y.png')
	# plt.plot(time,[trajectory[i][2] for i in range(frequency * moving_time)])
	# plt.savefig('trajectory_z.png')

	plt.plot(time,[trajectory_derivative[i][0] for i in range(frequency * moving_time)])
	plt.savefig('trajectory_x_v.png')
	plt.plot(time,[trajectory_derivative[i][1] for i in range(frequency * moving_time)])
	plt.savefig('trajectory_y_v.png')
	plt.plot(time,[trajectory_derivative[i][2] for i in range(frequency * moving_time)])
	plt.savefig('trajectory_z_v.png')
	# plt.show()

