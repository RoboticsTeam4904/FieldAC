from networktables import NetworkTables
from autocalibrate import calibrate
import config
NetworkTables.setTeam(config.team)
NetworkTables.initialize(server=config.ip)
table = NetworkTables.getTable('Vision')

while True:
	if table.getBoolean('Autocalibration', False):
		print "Calibrating the camera due to button press"
		calibrate()
		table.putBoolean('Autocalibration complete', True)
		table.putBoolean('Autocalibration', False)
