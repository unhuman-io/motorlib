import json
import os
import subprocess
import sys
import yaml
import logging
import argparse
from calibration_file import CalibrationFile
from motor_calibrator import MotorCalibrator
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
			typ = line[1]
			name = line[2]
			param_file = line[3]
			package_dict[name] = {"sn":sn, "typ":typ, "param_file":param_file}
	return package_dict

if __name__ == "__main__":
	""" Usage: python3 save_to_flash.py --use_package=False --base_config=base_test.json --motor_name right_hip_z_010
		   or: python3 save_to_flash.py --package=package_config_lt_dan.yaml --motor_name=right_hip_z_010"""
	parser = argparse.ArgumentParser()
	parser.add_argument(
		"--motor_name",
		required=True,
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
		"--flash",
		action='store_true',
		help="Read JSON config files and save to flash",
	)

	parser.add_argument(
		"--runtime_flash",
		action='store_true',
		help="Read runtime values and save to flash",
	)

	parser.add_argument(
		"--remote",
		action='store_true',
		help="Run on robot",
	)

	args = parser.parse_args()

	if args.use_package is True and args.package is None:
		parser.error("--use_package=True requires a --package specified")

	if args.use_package is False and (args.base_config is None or args.serial_number is None):
		parser.error("--use_package=False requires a --base_config and --serial_number specified")

	base_config = args.base_config
	serial_number = args.serial_number
	if args.use_package:
		# If the user provided the YAML package read the motor details from the package
		package_data = read_yaml(args.package)
		serial_number = package_data[args.motor_name]["sn"]
		base_config = args.config_dir + "/" + args.motor_name + ".json"

	client = None
	if args.remote:
		client = RemoteClient()

	motor_cal = MotorCalibrator(args.motor_name, base_config, serial_number, client)

	if args.flash:
		motor_cal.run_save_to_flash_routine()
	elif args.runtime_flash:
		motor_cal.run_read_runtime_and_save_to_flash_routine()
