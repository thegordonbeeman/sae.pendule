import math, time

from resources.utils import RobotController
import resources.robot_config as configs

from pid import PID

def shutdown_pendulums(pendulums):
	for pendulum in pendulums:
		pendulum.set_joint_torque(0)
	
	time.sleep(2)

	for pendulum in pendulums:
		pendulum.shutdown()

class Pendulum():
	ROLE_MASTER = 0
	ROLE_SLAVE = 1

	def __init__(self, typename, role):
		if (not typename in configs):
			print("Le pendule a un type invalide.")
			exit()
		if (not role in [self.ROLE_MASTER, self.ROLE_SLAVE]):
			print("Le pendule a un rôle invalide.")
			exit()

		self.params = configs[typename]

		self.controller = RobotController(typename, False)
		self.pid = PID(self.params["kp"], self.params["ki"], self.params["kd"])

		self.length = self.params["length"]
		self.weight = self.params["weight"]

		self.angle = 0
		self.torque = 0
		
		print(f"Pendule {typename} créé avec le rôle {(role == self.ROLE_MASTER and 'Master' or 'Slave')}")

	def init():
		self.controller.init_offset()
		self.controller.set_joint_position(0)

		# Etablir la connection avec le serveur, préciser son rôle, récupérer son identifiant

	def fetch_position(self):
		self.controller.set_joint_torque(self.torque)
		self.angle = math.radians(self.controller.get_joint_position())

		self.pid.set_PV(self.angle)
	
	def fetch_setpoint(self):
		# Récupérer le setpoint grâce aux sockets
		# Les esclaves récupèrent le setpoint et le suivent
		# Les maîtres envoient leur position au serveur, récupèrent le nouveau setpoint, et s'y ajustent

		# Récupérer SetPoint
		#	self.pid.set_SP(oijqozijfqoijqzoijf)
		if (self.role == self.ROLE_MASTER):
			# Envoyer une requête au serveur, en précisant notre position actuelle
			pass
		else:
			# Demander au serveur la position, sans rien préciser
			pass
	
	def calc_tq_pid(self):
		return self.pid.update(0.005)

	def calc_tq_gcomp(self):
		return math.sin(self.angle) * self.length * self.weight * -9.81
	
	def calc_torque(self):
		self.torque = self.calc_tq_gcomp() + self.calc_tq_pid()
		return self.torque

	def apply_torque(self, torque=None):
		if (torque is not None):
			self.controller.set_joint_torque(torque)
		else:
			self.controller.set_joint_torque(self.torque)

if __name__ == "__main__":
	pndl_master = Pendulum("PENDULUM", Pendulum.ROLE_MASTER, (1, 0, 0))

	if (input('Go?\n:') not in ['y', 'Y']):
		print("Sortie.")
		exit()
	
	pndl_master.init()

	while (True):
		pndl_master.fetch_position() # Récupérer notre position physique
		setpoint = pndl_master.fetch_setpoint() # *Donner notre position au serveur*, et récupérer un setpoint
		pndl_master.calc_torque() # Calculer le torque à appliquer selon PV et SP
		pndl_master.apply_torque() # L'appliquer
	
	shutdown_pendulums([pndl_master])