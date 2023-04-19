import json
import os
import subprocess
import sys
import logging
import argparse
from logger import Logger

logger = Logger(__name__)

def merge_dicts(dict1, dict2) -> dict:
	"""Recursively merge two dictionaries"""
	merged_dict = dict1.copy()

	for key, value in dict2.items():
		if key in merged_dict and isinstance(merged_dict[key], dict) and isinstance(value, dict):
			merged_dict[key] = merge_dicts(merged_dict[key], value)
		else:
			merged_dict[key] = value

	return merged_dict

class CalibrationFile:
	def __init__(self, file_path):
		self.file_path = file_path
		self.processed_data = self.process_file()

	def process_file(self):
		"""Recursively process JSON files with 'inherits' key"""
		cnt = 0
		inherit_key = f"inherits{cnt}"
		with open(os.path.expanduser(self.file_path)) as f:
			self.data = json.load(f)
		processed_data=self.data.copy()
		while inherit_key in processed_data:
			inherited_file_path = os.path.join(os.path.dirname(self.file_path), processed_data[inherit_key])
			inherited_data = CalibrationFile(inherited_file_path).process_file()
			del processed_data[inherit_key]
			processed_data = merge_dicts(inherited_data, processed_data)
			cnt += 1
			inherit_key = f"inherits{cnt}"

		with open("processed.json","w") as f:
			json.dump(processed_data, f, indent=4)
		return processed_data

	def find_param(self, param_name):
		cnt = 0
		inherit_key = f"inherits{cnt}"

		def traverse(node, path):

			if path == param_name:
				return self.file_path
			elif isinstance(node, dict):
				for key, value in node.items():
					res = traverse(value, f"{path}.{key}" if path else key)
					if res: 
						return res
			elif isinstance(node, list):
				for i, item in enumerate(node):
					res = traverse(item, f"{path}[{i}]")
					if res:
						return res
			return None

		path = traverse(self.data, '')
		if path is not None:
			return path
		else:
			while inherit_key in self.data:
				inherited_file_path = os.path.join(os.path.dirname(self.file_path), self.data[inherit_key])
				res = CalibrationFile(inherited_file_path).find_param(param_name)
				if res is not None:
					return res
				else:
					cnt += 1
					inherit_key = f"inherits{cnt}"
			return None

	def overwrite_params(self, params):
		"""Overwrite specified parameter values in this file"""
		with open(self.file_path, 'r') as f:
			data = json.load(f)

		for p in params:
			for key, value in p.items():
				data = self.overwrite_key(data, key, value)

		with open(self.file_path, 'w') as f:
			json.dump(data, f, indent=4)

	def overwrite_key(self, data, param_name, new_value):
		"""Overwrite a single parameter value in this file"""
		keys = param_name.split('.')
		cur = data
		for key in keys[:-1]:
			cur = cur[key]
		cur[keys[-1]] = str(new_value)
		return data

	def write_runtime_values_to_json(self, params_to_values):
		file_to_params = {}
		for param, value in params_to_values.items():
			file_path = self.find_param(param)
			if file_path:
				if file_path in file_to_params:
					file_to_params[file_path].append({param:value})
				else:
					file_to_params[file_path] = []
					file_to_params[file_path].append({param:value})
			else:
				logger.error(f"Parameter {param} not found in {self.config_file_path} or any inherited files")

		for file_path, params in file_to_params.items():
			file = CalibrationFile(file_path)
			file.overwrite_params(params)