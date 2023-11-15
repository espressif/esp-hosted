#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
# Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Credits:
# writeup at https://medium.com/@securisec/building-a-dynamic-and-self-documenting-python-cli-af4dd12eb91
# twitter: @securisec

import sys

if sys.version_info[0] < 3:
	print("please re-run using python3")
	raise SystemExit('Exit')

import inspect
import argparse
import fire
from docstring_parser import parse
from prompt_toolkit.completion import Completer, Completion, FuzzyCompleter
from prompt_toolkit import PromptSession
from prompt_toolkit.history import FileHistory
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory

from py_parse.cmds import ctrl_cmd
from py_parse.process import process_init_control_lib, process_deinit_control_lib, process_heartbeat

import traceback

DEBUG = 0

HIST_FILE = ".hosted_cli_history"



def google_fire(possible_options):
	for method in possible_options:
		if not method.startswith("_") and not isinstance(
			getattr(ctrl_cmd, method), property
		):
			fire.decorators._SetMetadata(
				getattr(ctrl_cmd, method),
				fire.decorators.ACCEPTS_POSITIONAL_ARGS,
				True)


possible_options = dir(ctrl_cmd)
options = []



def get_options():
	global possible_options
	options = dict()
	for method in possible_options:
		available_methods = getattr(ctrl_cmd, method)
		if not method.startswith("_"):
			args = inspect.getfullargspec(available_methods).args
			parsed_doc = parse(available_methods.__doc__)
			options[method] = {
				"options": list(
					map(
						lambda d: {
							"flag": d[1],
							"meta": parsed_doc.params[d[0]].description,
						},
						enumerate(args[1:]),
					)
				),
				"meta": parsed_doc.short_description,
				"returns": parsed_doc.returns.type_name,
			}
	return options



class CustomCompleter(Completer):
	def get_completions(self, document, complete_event):
		global options
		method_dict = get_options()
		word = document.get_word_before_cursor()

		methods = list(method_dict.items())

		selected = document.text.split()
		if len(selected) > 0:
			selected = selected[0]
			if not selected.startswith("-"):
				current = method_dict.get(selected)
				if current is not None:
					has_options = method_dict.get(selected)["options"]
					if has_options is not None:
						options = [
							("--{}".format(o["flag"]), {"meta": o["meta"]})
							for o in has_options
						]
						methods = options + methods
			else:
				methods = options

		for m in methods:
			method_name, flag = m
			if method_name.startswith(word):
				meta = (
					flag["meta"] if isinstance(flag, dict) and flag.get("meta") else ""
				)
				yield Completion(
					method_name, start_position=-len(word), display_meta=meta,
				)



def exit_wrap():
	process_deinit_control_lib(True)
	raise SystemExit('Exit')



def filter_input(cmd):
	global DEBUG
	cmd2 = ''

	if cmd == '':
		pass
	elif (cmd == "quit" or cmd == "q" or cmd == "exit"):
		exit_wrap()
	elif "--help" in cmd:
		cmd2 = cmd.replace('--help', '-- --verbose --help')
		return 0,cmd2
	elif "help" in cmd:
		cmd2 = cmd.replace('help', ' -- --verbose --help')
		return 0,cmd2
	elif cmd == "debug on":
		print("debug is on")
		DEBUG = 1
	elif cmd == "debug off":
		print("debug is off")
		DEBUG = 0
	else:
		return 0,cmd
	return 1,cmd



def main():
	global possible_options

	process_init_control_lib()

	argumentList = sys.argv[1:]
	if argumentList and len(argumentList):
		try:
			if "--help" in argumentList:
				sys.argv.insert(argumentList.index("--help")+1, '--verbose')
				sys.argv.insert(argumentList.index("--help")+1, '--')
			fire.Fire(ctrl_cmd)
		except:
			#traceback.print_exc()
			if "--help" in argumentList:
				pass
			else:
				print("Command failed/unsupported")
		process_deinit_control_lib()
		raise SystemExit()

	parse = argparse.ArgumentParser()
	args = parse.parse_args()

	our_history = FileHistory(HIST_FILE)
	session = PromptSession(history=our_history, auto_suggest=AutoSuggestFromHistory())
	try:
		while True:
			try :
				prompt1 = session.prompt(
					"hosted > ", completer=FuzzyCompleter(CustomCompleter()),
				)
			except KeyboardInterrupt:
				continue

			ret, prompt2 = filter_input(prompt1)
			if ret:
				continue

			google_fire(possible_options)

			try :
				fire_obj = fire.Fire(ctrl_cmd, command=prompt2)
			except:
				if DEBUG:
					traceback.print_exc()
	except SystemExit:
		exit_wrap()
	except KeyboardInterrupt:
		exit_wrap();
	except EOFError:
		exit_wrap()
	except :
		if DEBUG:
			traceback.print_exc()
			print("exception!!")


if __name__ == "__main__":
	main()
