import json
import os
import sys
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

def add_path_to_dict(path, dictionary, param, value):
	if path in dictionary:
		dictionary[path].append({param:value})
	else:
		dictionary[path] = []
		dictionary[path].append({param:value})

def get_data(file_path):
	with open(file_path, 'r') as f:
		data = json.load(f)
	return data

class CalibrationFile:
	def __init__(self, file_path):
		self.file_path = file_path
		self.data = get_data(self.file_path)
		self.inherit_files = []
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
			self.inherit_files.append(inherited_file_path)
			inherited_data = CalibrationFile(inherited_file_path).process_file()
			del processed_data[inherit_key]
			processed_data = merge_dicts(inherited_data, processed_data)
			cnt += 1
			inherit_key = f"inherits{cnt}"

		with open("processed.json","w") as f:
			json.dump(processed_data, f, indent=4)
		return processed_data

	def write_to_file(self):
		with open(self.file_path, 'w') as f:
			json.dump(self.data, f, indent=4)

	def add_param(self, param_string, value):
	    parts = param_string.split('.')
	    current = self.data
	    for part in parts[:-1]:
	    	if part in current.keys():
	    		current = current[part]
	    	else:
	        	current[part] = {}
	        	current = current[part]
	    current[parts[-1]] = value
	    self.write_to_file()

	def write_runtime_values_to_json(self, params_to_values):
		file_to_params = {}
		for param, value in params_to_values.items():
			self.add_param(param, value)
