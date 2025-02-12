import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import math

# SP: SetPoint
# PV: ProcessValue
# Err: SP - PV
# P: Err * Kp
# I: somme(Err) * Ki
# D: dérivée(Err) * Kd

class PID():
	def __init__(self, Kp, Ki, Kd):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd

		self.SP, self.PV = 0, 0
		self.Err, self.last_Err, self.sum_Err = 0, 0, 0
		self.Output = 0
	
	def get_SP(self):
		return self.SP

	def set_SP(self, SP):
		self.SP = SP
	
	def get_PV(self):
		return self.PV
	
	def set_PV(self, PV):
		self.PV = PV

	def update(self, ts):
		self.Err = self.SP - self.PV
		self.sum_Err += self.Err

		P = self.Err * self.Kp
		I = self.sum_Err * self.Ki
		D = (self.last_Err - self.Err) * self.Kd / ts

		self.Output = P + I + D

		self.last_Err = self.Err

		return self.Output

class PIDMonitor():
	def __init__(self, pid, sample_count):
		self.pid = pid
		self.sample_count = sample_count

		self.SP = np.zeros((sample_count))
		self.PV = np.zeros((sample_count))
		self.Err = np.zeros((sample_count))
		self.Output = np.zeros((sample_count))

		self.index = 0
	
	def update(self):
		if (self.index > self.sample_count - 1):
			return
		
		self.SP[self.index] = self.pid.SP
		self.PV[self.index] = self.pid.PV
		self.Err[self.index] = self.pid.Err
		self.Output[self.index] = self.pid.Output

		self.index += 1

if __name__ == "__main__":
	ts = 0.005
	sample_count = 1000

	pid = PID(2.0, 0.1, 0.0)
	pidm = PIDMonitor(pid, sample_count)

	PV = 0

	for i in range(sample_count):
		pid.set_SP(math.cos(i / 100) * math.pi / 5 + math.cos(i / 50) * math.pi / 7 + (abs(math.sin(i / 40)) * math.pi / 10))
		pid.set_PV(PV)

		pid.update(ts)
		pidm.update()

		PV += pid.Output * 0.03

	xs = np.linspace(0, sample_count * ts, sample_count)
	fig = plt.figure()
	sbp = fig.add_subplot(1, 1, 1)

	# def animate(t):
	# 	t = math.floor(t * 10)
	t = sample_count

	sbp.clear()

	for t in range(sample_count):
		sbp.plot(xs[:t], pidm.SP[:t], 'b', ls='--')
		sbp.plot(xs[:t], pidm.PV[:t], 'b')
		sbp.plot(xs[:t], pidm.Err[:t], 'r')
		sbp.plot(xs[:t], pidm.Output[:t], 'g')
		sbp.legend(['SP', 'PV', 'Err', 'Output'])
		plt.pause(0.05)

	# anim = animation.FuncAnimation(fig=fig, func=animate, frames=math.floor(sample_count / 10), interval=ts * 1000)
	# anim.save('pid.gif', writer="imagemagick", fps=5)

	plt.show()