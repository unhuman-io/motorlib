import os
import sys
import subprocess
import argparse
from calibration_file import CalibrationFile
from logger import Logger
import psutil
import time

logger = Logger(__name__)

# Read the environment variable "OBOT_PATH"
obot_path = os.getenv('OBOT_PATH')

if(obot_path is None):
	logger.error("Please define OBOT_PATH environment variable."\
				  "Run the following command with the correct user path`export OBOT_PATH=/home/user-path/obot-controller/obot_g474`")
	sys.exit()

def kill(proc_pid):
	process = psutil.Process(proc_pid)
	for proc in process.children(recursive=True):
		proc.kill()
	process.kill()

def generate_c_header_file(param_dict, motor_name, incl_file=None, prefix=""):
	'''Function that recursively reads the dictionary to generate the proper variable names'''
	filename = obot_path + f"/param/param_obot_g474_{motor_name}.c"

	if incl_file is not None:
		header_file = "#include \"" + incl_file + "\"\n"
	else:
		header_file = ""

	if prefix == "":
		header_file += "#include \"param_obot_g474.h\"\n"
		header_file += "#include \"math.h\"\n"
		header_file += "const volatile Param __attribute__ ((section (\"flash_param\"))) param_store = {\n"

	for param_name, param_value in param_dict.items():
		if param_name == "filename":
			continue

		if "include" in param_name:
			header_file += param_value +"\n"

		elif type(param_value) == dict:
			header_file += generate_c_header_file(
				param_value, motor_name, incl_file, prefix + "." + param_name
			)
		elif type(param_value) == list:
			elements = ""
			for elem in param_value:
				elements += "\t" + elem + ",\n"
			header_file += prefix + "." + param_name + " = {" + elements + "\t},\n"
		else:
			header_file += prefix + "." + param_name + " = "

			if type(param_value) == str:
				header_file += param_value
			elif type(param_value) == bool:
				header_file += str(param_value).lower()
			else:
				header_file += str(param_value)

			header_file += ",\n"

	# When we've generated the full string write it to the header file
	if prefix == "":
		header_file += "};"
		with open(filename, "w") as f:
			f.write(header_file)
		logger.info(f"Generated header file at {filename}")
		return filename

	return header_file

def read_motor_util_log(filename, api_to_cparams):
	params_to_values = {}
	# Open the file for reading
	with open(filename, 'r') as f:
		# Read the first line to get the number of motors connected
		# Parse the second-to-last line and split it into values
		lines = f.readlines()
		second_to_last_line = lines[-2].strip()
	# Split the data line into separate values
	values = second_to_last_line.strip().split(', ')

	# Assign the value to the correct variable
	i = 0
	for util_param, c_param in api_to_cparams.items():
		if values[i].lower() == 'nan':
			logger.warning(f"Parameter {util_param} is NAN, this value will not be overwritten")
		else:
			params_to_values[c_param] = float(values[i])
		i+=1

	return params_to_values

class MotorHandler:
	def __init__(self, name, config_dir_path, motor_info, client=None):
		self.name = name
		self.motor_info = motor_info
		self.config_file_path = os.path.expanduser(config_dir_path + "/" + name + ".json")
		self.serial_number = motor_info["sn"]
		self.json_config_file = CalibrationFile(self.config_file_path)
		self.client = client
		self.param_address = "0x8060000"
		self.fw_address = "0x8000000"

	def generate_param_c_file(self):
		"""Generate a C file based on the motor's configuration file"""
		data = self.json_config_file.process_file()
		generate_c_header_file(data, self.name)

	def generate_binary(self, param_only = True):
		"""Generate a binary file with either parameters or firmware"""
		binary_path = None
		if param_only:
			# Remove the existing build directory to guarantee new binaries are made
			result = subprocess.run("rm -rf build/".split(), cwd=obot_path, capture_output=True)
			cmd = f"make build_param PARAM={self.name}"
			binary_path = obot_path + "/build/param/param_obot_g474_" + self.name + ".bin"
		else:
			cmd =  f"make -j CONFIG={self.motor_info['fw_type']} C_DEFS=-D{self.motor_info['pcb_type']}"
			binary_path = obot_path + "/build/" + self.motor_info["fw_type"] + "_noparam.bin"

		logger.info(f"Executing: {cmd}")

		result = subprocess.run(cmd.split(), cwd=obot_path, capture_output=True)
		if result.returncode != 0:
			raise Exception(f"Failed to generate binary file for motor {self.name}")

		logger.info(f"Generated binary at {binary_path}")

		if self.client is not None:
			binary_remote_dest = "/tmp/" + binary_path.split("/")[-1]
			self.client.send_file(binary_path, binary_remote_dest)

		return binary_path

	def flash_binary_file(self, file_path, address, leave=True):
		"""Flash the binary file to the given address using dfu-util"""
		leave_dfu = ""
		if leave:
			leave_dfu = ":leave"

		if self.client is None:
			cmd = f"dfu-util -a0 -s {address}{leave_dfu} -D {file_path} -S {self.serial_number}"
			self.flash_binary_file_local(cmd)
		else:
			# the binary is sent to the /tmp folder on the remote machine
			binary_path = "/tmp/" + file_path.split("/")[-1]
			cmd = f"dfu-util -a0 -s {address}{leave_dfu} -D {binary_path} -S {self.serial_number}"
			self.flash_binary_file_remote(cmd)
			self.client.run_command(f"rm -f {binary_path}")

	def flash_binary_file_remote(self, cmd):
		"""Flash the binary file to the motor's flash memory"""
		'''Use dfu-util to flash the binary to the flash memory'''
		logger.info(f"Executing: {cmd}")
		self.client.run_command(cmd)

	def flash_binary_file_local(self, cmd):
		'''Use dfu-util to flash the binary to the flash memory'''
		logger.info(f"Executing: {cmd}")
		result = subprocess.run(cmd.split(), cwd=obot_path, capture_output=True, text=True)
		logger.info(result.stdout)

		# The dfu-util may throw the "dfu-util: Error during download get_status"
		# even when the binary is successfully downloaded
		if result.returncode != 0:
			if "dfu-util: Error during download get_status" in result.stderr:
				pass
			else:
				raise Exception(f"Failed to flash binary file for motor {self.name}")


	def read_values_from_motor_util_local(self, command):
		motor_util_process = subprocess.Popen("exec " + ' '.join(command), 
											  shell=True,
											  stdin=subprocess.PIPE,
											  stdout=subprocess.PIPE,
											  stderr=subprocess.PIPE,
											  preexec_fn=os.setsid)

		# Write data to a file for 1s and then kill the process
		try:
			motor_util_process.wait(timeout=1)
		except subprocess.TimeoutExpired:
			kill(motor_util_process.pid)

		# Propagate any errors from the subprocess
		stdout, stderr = motor_util_process.communicate()
		if stderr:
			logger.error(f"Standard error: {stderr.decode('utf-8')}")

	def read_values_from_motor_util_remote(self, command, param_log_path):
		# create the log with the values on the remote computer
		cmd = ' '.join(command) + "& sleep 2 ; kill $!"
		logger.info(cmd)
		self.client.run_command(cmd)
		# transfer the text file to a local location
		self.client.get_file(param_log_path, param_log_path)

	def read_runtime_values(self):
		# Parameters to read at runtime and overwrite
		api_to_cparams = {
			"phase_mode": "fast_loop_param.phase_mode",
			"index_offset_measured": "fast_loop_param.motor_encoder.index_electrical_offset_pos",
			"obias": "main_loop_param.output_encoder.bias",
			"startup_mbias": "startup_param.motor_encoder_bias",
			"tgain": "main_loop_param.torque_sensor.gain",
			"tbias": "main_loop_param.torque_sensor.bias",
			"state_ff_tau": "main_loop_param.state_controller_param.ff_tau",
			"idmax":"fast_loop_param.foc_param.pi_d.command_max",
			"imax":"fast_loop_param.foc_param.pi_q.command_max",
			"state_command_max":"main_loop_param.state_controller_param.command_max"
		}

		process_command = ["motor_util", "-s", f"{self.serial_number}","read", "--text"]
		for api_param in api_to_cparams.keys():
			process_command.append(api_param)
		process_command.append(">>")
		param_log_path = f"/tmp/motor_{self.serial_number}_params.txt"
		process_command.append(param_log_path)

		# Start the motor_util command-line interface with the --api option
		logger.info(f"Executing {' '.join(process_command)}")

		if self.client is None:
			self.read_values_from_motor_util_local(process_command)
		else:
			self.read_values_from_motor_util_remote(process_command, param_log_path)

		# Read the text log with parameter values and generate a dictionary with the data
		params_to_values = read_motor_util_log(param_log_path, api_to_cparams)

		# Remove the generated text log
		subprocess.run(["rm", f"{param_log_path}"])
		return params_to_values

	def run_flash_params_routine(self):
		logger.info(f"***************** Running Flash Params Routine for Motor: {self.name} SN: {self.serial_number}: ********************* ")

		# Generate a C file given the JSON file with parameters
		self.generate_param_c_file()

		# Generate a binary with only the flash parameters (not full firmware)
		binary_path = self.generate_binary(param_only=True)

		# Flash the param only binary
		self.flash_binary_file(binary_path, self.param_address)

	def run_read_runtime_and_save_to_flash_routine(self):
		logger.info(f"*****************  Running Read Runtime Values and Save to Flash Routine for Motor: {self.name} SN: {self.serial_number} ***************** ")

		# Read the runtime values
		new_values = self.read_runtime_values()

		# Write the runtime values to the associated JSON file
		self.json_config_file.write_runtime_values_to_json(new_values)

		# Reprocess the new files
		data = self.json_config_file.process_file()

		# Run the routine to save values from the JSON file to flash
		self.run_flash_params_routine()

	def run_flash_firmware_routine(self):
		logger.info(f"*****************  Running Flash Firmware Routine for Motor: {self.name} SN: {self.serial_number} ***************** ")

		# Generate a binary with only the firmware (not parameters)
		binary_path = self.generate_binary(param_only=False)

		# Flash the firmware binary
		self.flash_binary_file(binary_path, self.fw_address)

	def run_flash_all_routine(self):
		logger.info(f"*****************  Running Flash All Routine for Motor: {self.name} SN: {self.serial_number} ***************** ")

		# Generate a C file with params given the JSON file with parameters
		self.generate_param_c_file()

		# Generate a binary with only the firmware (no parameters)
		fw_binary_path = self.generate_binary(param_only=False)
		
		# Flash the firmware binary
		self.flash_binary_file(fw_binary_path, self.fw_address, leave=False)

		# Generate a binary with only the flash parameters (no full firmware)
		param_binary_path = self.generate_binary(param_only=True)

		# Flash the param only binary
		self.flash_binary_file(param_binary_path, self.param_address)