import os
import sys
import yaml
import logging
import argparse
from calibration_file import CalibrationFile
from motor_handler import MotorHandler
from logger import Logger
from remote_client import RemoteClient

logger = Logger(__name__)

def read_yaml(filename) -> dict:
	package_dict = {}
	with open( os.path.expanduser(filename), 'r') as file:
		d = yaml.safe_load(file)
		for line in d["config"]:
			# for example:
			# config:
			#  - ["207539635356", "motor_aksim", "J1", "R4"]
			sn = line[0]
			fw_type = line[1]
			name = line[2]
			pcb_type = line[3]
			package_dict[name] = {"sn":sn, "fw_type":fw_type, "pcb_type":pcb_type}
	return package_dict

def run_routine(args, motor_handler):
	if args.flash_params:
		motor_handler.run_flash_params_routine()
	elif args.runtime_flash_params:
		motor_handler.run_read_runtime_and_save_to_flash_routine()
	elif args.flash_firmware:
		motor_handler.run_flash_firmware_routine(package_data)
	elif args.flash_all:
		motor_handler.run_flash_all_routine(package_data)

if __name__ == "__main__":
	""" Usage: python3 save_to_flash.py --config_dir=/project-x/tools/obot --base_config=base_test.json --motor_name right_hip_z_010
		   or: python3 save_to_flash.py --config_dir=/project-x/tools/obot --package=package_config_lt_dan.yaml --motor_name=right_hip_z_010"""
	parser = argparse.ArgumentParser()
	parser.add_argument(
		"--motor_name",
		default=None,
		help="Motor name that will be used to generate the header file and binary file names",
	)

	parser.add_argument(
		"--use_package",
		default=True,
		help="Read param file name and motor SN from the provided yaml package file",
	)

	parser.add_argument(
		"--package",
		default=None,
		help="YAML config file containing all connected motor names, SNs and param file names "
	)

	parser.add_argument(
		"--base_config",
		default=None,
		help="Path to the JSON file that contains the params (must be provided if --use_package=False)",
	)
	
	parser.add_argument(
		"--serial_number",
		default=None,
		help="Pass in the serial number of the motor in question directly (must be provided if --use_package=False)",
	)

	parser.add_argument(
		"--config_dir",
		required=True,
		help="Path to the directory containing the base json/config file",
	)

	parser.add_argument(
		"--flash_params",
		action='store_true',
		help="Read JSON config files and save params to flash",
	)

	parser.add_argument(
		"--runtime_flash_params",
		action='store_true',
		help="Read runtime values and save params to flash",
	)

	parser.add_argument(
		"--remote",
		action='store_true',
		help="Run on robot",
	)

	parser.add_argument(
		"--ip",
		help="IP address of the remote machine"
	)

	parser.add_argument(
		"--flash_all",
		action='store_true',
		help="Flash both firmware and params",
	)

	parser.add_argument(
		"--flash_firmware",
		action='store_true',
		help="Flash firmware",
	)

	args = parser.parse_args()

	if args.use_package is True and args.package is None:
		parser.error("--use_package=True requires a --package specified")

	if args.use_package is False and (args.base_config is None or args.serial_number is None):
		parser.error("--use_package=False requires a --base_config and --serial_number specified")

	if args.remote and not args.ip:
		parser.error("--remote requires you to specify the remote machine's IP address with --ip")

	client = None
	if args.remote:
		client = RemoteClient(args.ip)

	# If the user provided the YAML package read the motor details from the package
	if args.use_package:
		package_data = read_yaml(args.package)

		# If the motor_name was specified only run routine on the specified motor
		if args.motor_name is not None:
			try:
				motor_info = package_data[args.motor_name]
			except KeyError as e:
				logger.error(f"{args.motor_name} not found in {args.package}. Please check contents of the package file")
				raise
			motor_handler = MotorHandler(args.motor_name, args.config_dir, motor_info, client)
			run_routine(args, motor_handler)

		else:
			# If no motor_name was specified run the routine on all motors defined in the package
			print(package_data)
			for motor_name in package_data.keys():
				motor_handler = MotorHandler(motor_name, args.config_dir, package_data[motor_name], client)
				run_routine(args, motor_handler)

	# Otherwise the user should have provided a json file and a serial number as input
	else:
		base_config = args.base_config
		serial_number = args.serial_number
		motor_handler = MotorHandler(args.motor_name, base_config, serial_number, client)
		run_routine(args, motor_handler)
