#!/usr/bin/env python
import yaml

import resource_retriever


class Config(object):
  def __init__(self, config_path, custom_config=None, config_format="yaml"):
    config_path = self.process_path(config_path)
    producer = self.get_config_producer(config_format)
    self.config = producer(config_path)
    self.custom_config = custom_config

  def process_path(self, config_path):
    if "package://" in config_path:
      config_path = resource_retriever.get_filename(config_path, use_protocol=False)
    return config_path

  def get_config_producer(self, config_format):
    if config_format=="yaml":
      return self.yaml_config_producer
    else:
      return ValueError("Config format {} is not supported")

  def return_config_in_list(self, param_list):
    found_keys = []
    empty_keys = []
    values = []
    
    for param in param_list:
      if self.in_custom_config(param):
	found_keys += [param]
        values += [self.custom_config[param]]
      elif self.in_config(param):
        found_keys += [param]
	values += [self.custom[param]]
      else:
	empty_keys += [param]
    return found_keys, values, empty_keys

  def update_config_in_list(self, instance, param_list):
    for param in param_list:
      if self.in_custom_config(param):
	setattr(instance, param, self.custom_config[param])
      elif self.in_config(param):
	setattr(instance, param, self.config[param])
      else:
	setattr(instance, param, None)

  def in_custom_config(self, attr):
    if not self.custom_config:
      return False
    return attr in self.custom_config

  def in_config(self, attr):
    return attr in self.config

  def yaml_config_producer(self, config_path):
    """Load and parse yaml file. Returns the content as a dict.
    """
    with open(config_path, 'rb') as params_yaml:
      try:
        params = yaml.load(params_yaml)
      except yaml.YAMLError as exc:
	raise Exception(exc)
    return params
