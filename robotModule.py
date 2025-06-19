from ctypes import *
from encodings import utf_8

g_RobotModule = None
g_ModuleData = None
g_Logger = None

class TModuleObject(Structure):
	_fields_ = [("Robot", c_void_p)]

def log(m):
	global g_Logger
	g_Logger.debug(m)

def robotInit(logger):
	
	global g_RobotModule
	global g_ModuleData
	global g_Logger

	g_Logger = logger
	
	if g_RobotModule is None:
		g_RobotModule = cdll.LoadLibrary('/home/pi/git/sprobot/module.so')
		g_RobotModule.init.argtypes = [POINTER(TModuleObject)]
		g_RobotModule.init.restype = c_int
		g_RobotModule.shutdown.argtypes = [POINTER(TModuleObject)]
		g_RobotModule.run_cmd.argtypes = [POINTER(TModuleObject), c_char_p, c_char_p]
		g_RobotModule.run_cmd.restype = c_int

		g_ModuleData =  TModuleObject()
		g_RobotModule.init(g_ModuleData)
	return []
	
def robotShutdown():
	global g_RobotModule
	global g_ModuleData

	if g_RobotModule is not None:
		g_RobotModule.shutdown(g_ModuleData)
		g_RobotModule = None

def robotCmd(name, value):
	global g_RobotModule
	global g_ModuleData

	return g_RobotModule.run_cmd(g_ModuleData, name.encode("UTF-8"), value.encode("UTF-8"))
