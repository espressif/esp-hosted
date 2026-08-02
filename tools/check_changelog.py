#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
# check that the changelog file contains the changelog for the version
# in idf_component.yml
# exit with 0 if ok
# exit with 1 if fail
# to be run whenever idf_component.yml is updated
import argparse
import re
import sys
# paths to files to check
yml_file = "idf_component.yml"
changelog_file = "docs/changelog.md"
def get_idf_yml_version_as_string() -> str:
	# read the yml file
	file_info = open(yml_file, "r")
	info = file_info.read()
	file_info.close()
	# extract the version info
	ver = re.search("^version: \"([0-9.]+)\"", info, re.MULTILINE)
	# print("yml:", ver.group(1))
	return ver.group(1)
def changelog_has_version(ver_string: str, debug: bool = False) -> int:
	# iterate over the changelog file
	escaped_ver = re.escape(ver_string)
	# Match the colored version heading format:
	# # $${\color{COLOR} \text{VERSION}}$$ (with optional trailing text e.g. " - Some Title")
	pattern = r'^# \$\$\{\\color\{[a-z]+\} \\text\{' + escaped_ver + r'\}\}\$\$'
	if debug:
		print(f"[debug] Looking for version : {ver_string}")
		print(f"[debug] Using pattern       : {pattern}")
		print(f"[debug] Scanning            : {changelog_file}")
		print()
	with open(changelog_file, "r") as changelog:
		for lineno, line in enumerate(changelog, start=1):
			stripped = line.rstrip('\n')
			if re.match(pattern, stripped):
				if debug:
					print(f"[debug] MATCH at line {lineno}: {stripped!r}")
				return 0
			elif debug and stripped.startswith('# '):
				print(f"[debug] heading line {lineno:4d}: {stripped!r}")
	return 1
def check(debug: bool = False) -> int:
	yml_string = get_idf_yml_version_as_string()
	if debug:
		print(f"[debug] Version from {yml_file}: {yml_string}")
		print()
	result = changelog_has_version(yml_string, debug=debug)
	if result:
		print(f"Changelog for version {yml_string} not found in {changelog_file}")
		if not debug:
			print(f"Tip: re-run with --debug for more details")
		return 1
	return 0
if __name__ == '__main__':
	parser = argparse.ArgumentParser(
		description="Check that the changelog contains an entry for the version in idf_component.yml"
	)
	parser.add_argument(
		"--debug",
		action="store_true",
		help="Print the pattern used and all # headings found in the changelog to help diagnose failures"
	)
	args, _ = parser.parse_known_args()  # parse_known_args ignores filenames passed by pre-commit
	sys.exit(check(debug=args.debug))
