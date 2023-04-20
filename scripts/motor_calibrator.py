import json
import os
import subprocess
import sys
# import logging
import argparse
from calibration_file import CalibrationFile
from logger import Logger
import signal
import psutil
import paramiko


logger = Logger(__name__)

# Accessing the value of the environment variable "OBOT_PATH"
obot_path = os.getenv('OBOT_PATH')

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

def parse_value(param, value_str):
	if value_str.lower() == 'nan':
		return float(0)
		# raise ValueError(f"{param} is NAN")
	else:
		return float(value_str)

def read_motor_util_log(filename, api_to_params):
	params_to_values = {}
	# Open the file for reading
	with open(filename, 'r') as f:
		# Read the first line to get the number of motors connected
		first_line = next(f)
		num_motors_connected = int(first_line.split()[0])
		# Determine the number of header lines based on the number of motors connected
		num_header_lines = num_motors_connected + 3
		# Skip the header lines
		for i in range(num_header_lines):
			next(f)
		# Read the first data line
		data = next(f)
	# Split the data line into separate values
	values = data.strip().split(', ')

	# Save the values to the specified variables
	i = 0
	for key, value in api_to_params.items():
		params_to_values[value] = parse_value(value, values[i])
		i+=1

	return params_to_values


class MotorCalibrator:
	def __init__(self, name, config_file_path, serial_number, client=None):
		self.name = name
		self.config_file_path = os.path.expanduser(config_file_path)
		self.serial_number = serial_number
		self.json_config_file = CalibrationFile(self.config_file_path)
		self.param_only_binary_path = None
		self.client = client

	def generate_header_file(self):
		"""Generate a C header file based on the motor's configuration file"""
		data = self.json_config_file.process_file()
		generate_c_header_file(data, self.name)

	def generate_binary_file(self):
		"""Generate a binary file based on the C header file"""
		cmd = f"make build_param PARAM={self.name}"
		logger.info(f"Executing: {cmd}")

		result = subprocess.run(cmd.split(), cwd=obot_path, capture_output=True)
		if result.returncode != 0:
			raise Exception(f"Failed to generate binary file for motor {self.name}")

		self.param_only_binary_path = obot_path + "/build/param/param_obot_g474_" + self.name + ".bin"
		logger.info(f"Generated binary at {self.param_only_binary_path}")
		print(self.param_only_binary_path)
		if self.client is not None:
			self.client.send_file(self.param_only_binary_path)

	def flash_binary_file(self):
		if self.client is None:
			self.flash_binary_file_local()
		else:
			self.flash_binary_file_remote()

	def flash_binary_file_remote(self):
		"""Flash the binary file to the motor's flash memory"""
		'''Use dfu-util to flash the binary to the flash memory'''
		binary_path = "/tmp/" + self.param_only_binary_path.split("/")[-1]
		cmd = f"dfu-util -a0 -s 0x8060000:leave -D {binary_path} -S {self.serial_number}"
		# cmd = "dfu-util"
		logger.info(f"Executing: {cmd}")

		self.client.run_command(cmd)

	def flash_binary_file_local(self):
		"""Flash the binary file to the motor's flash memory"""
		'''Use dfu-util to flash the binary to the flash memory'''
		cmd = f"dfu-util -a0 -s 0x8060000:leave -D {self.param_only_binary_path} -S {self.serial_number}"
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

	def read_runtime_values(self):

		api_to_params = {
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
		for api_param in api_to_params.keys():
			process_command.append(api_param)
		process_command.append(">>")
		textfile_name = f"motor_{self.serial_number}_params.txt"
		process_command.append(textfile_name)

		# Start the motor_util command-line interface with the --api option
		logger.info(f"Executing {' '.join(process_command)}")
		motor_util_process = subprocess.Popen("exec " + ' '.join(process_command), 
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

		# Read the text log with parameter values and generate a dictionary with the data
		params_to_values = read_motor_util_log(textfile_name, api_to_params)

		# Remove the generated text log
		subprocess.run(["rm", f"{textfile_name}"])
		print(params_to_values)
		return params_to_values

	def run_save_to_flash_routine(self):
		logger.info(f"***** Running Save to Flash routine: ****")

		# Generate a header file given the JSON file with parameters
		self.generate_header_file()

		# Generate a binary with only the flash parameters (not full firmware)
		self.generate_binary_file()

		# Save the param only binary to flash
		self.flash_binary_file()

	def run_read_runtime_and_save_to_flash_routine(self):
		logger.info(f"***** Running Read Runtime Values and Save to Flash routine: ****")

		# Read the runtime values
		new_values = self.read_runtime_values()

		# Write the runtime values to the associated JSON file
		self.json_config_file.write_runtime_values_to_json(new_values)

		# Reprocess the new files
		data = self.json_config_file.process_file()

		# Run the routine to save values from the JSON file to flash
		self.run_save_to_flash_routine()
