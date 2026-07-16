#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
# SPDX-FileCopyrightText: 2011-2019, Ulf Magnusson
# SPDX-License-Identifier: ISC
# This file is copied from kconfiglib project:
# https://github.com/ulfalizer/Kconfiglib/commit/061e71f7d78cb057762d88de088055361863deff
import errno
import importlib
import os
import re
import sys
from glob import iglob
from os.path import dirname
from os.path import exists
from os.path import expandvars
from os.path import islink
from os.path import join
from os.path import realpath
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Set
from typing import Tuple
from typing import Union

"""
Overview
========

Kconfiglib is a Python 3 library and part of esp-idf-kconfig for parsing and extracting information
from Kconfig (https://www.kernel.org/doc/Documentation/kbuild/kconfig-language.txt)
configuration system.

Intro to symbol values
======================

Kconfiglib has the same assignment semantics as the C implementation.

Any symbol can be assigned a value by the user (via Kconfig.load_config() or
Symbol.set_value()), but this user value is only respected if the symbol is
visible, which corresponds to it (currently) being visible in the menuconfig
interface.

For symbols with prompts, the visibility of the symbol is determined by the
condition on the prompt. Symbols without prompts are never visible, so setting
a user value on them is pointless. A warning will be printed by default if
Symbol.set_value() is called on a promptless symbol. Assignments to promptless
symbols are normal within a .config file, so no similar warning will be printed
by load_config().

Dependencies from parents and 'if'/'depends on' are propagated to properties,
including prompts, so these two configurations are logically equivalent:

(1)

  menu "menu"
      depends on A

  if B

  config FOO
      bool "foo" if D
      default y
      depends on C

  endif

  endmenu

(2)

  menu "menu"
      depends on A

  config FOO
      bool "foo" if A && B && C && D
      default y if A && B && C

  endmenu

In this example, A && B && C && D (the prompt condition) needs to be non-n for
FOO to be visible (assignable).

'default' properties are independent of the visibility, though a 'default' will
often get the same condition as the prompt due to dependency propagation.
'default' properties are used if the symbol is not visible or has no user
value.

Symbols with no user value (or that have a user value but are not visible) and
no (active) 'default' default to n for bool symbols and to the empty
string for other symbol types.

'select' works similarly to symbol visibility, but sets a lower bound on the
value of the symbol. The lower bound is determined by the value of the
select*ing* symbol. 'select' does not respect visibility, so non-visible
symbols can be forced to a particular (minimum) value by a select as well.

Conditions on 'default' and 'select' work in mostly intuitive ways. If the
condition is n, the 'default' or 'select' is disabled.

When writing a configuration with Kconfig.write_config(), only symbols that are
visible, have an (active) default, or are selected will get written out (note
that this includes all symbols that would accept user values). Kconfiglib
matches the .config format produced by the C implementations down to the
character. This eases testing.

For a visible bool symbol FOO with value n, this line is written to
.config:

    # CONFIG_FOO is not set

The point is to remember the user n selection (which might differ from the
default value the symbol would get), while at the same sticking to the rule
that undefined corresponds to n (.config uses Makefile format, making the line
above a comment). When the .config file is read back in, this line will be
treated the same as the following assignment:

    CONFIG_FOO=n

In Kconfiglib, the set of (currently) assignable values for a bool
symbol appear in Symbol.assignable. For other symbol types, just check if
sym.visibility is 2 (y) to see whether the user value will have an
effect.


Intro to the menu tree
======================

The menu structure, as seen in e.g. menuconfig, is represented by a tree of
MenuNode objects. The top node of the configuration corresponds to an implicit
top-level menu, the title of which is shown at the top in the standard
menuconfig interface. (The title is also available in Kconfig.mainmenu_text in
Kconfiglib.)

The top node is found in Kconfig.top_node. From there, you can visit child menu
nodes by following the 'list' pointer, and any following menu nodes by
following the 'next' pointer. Usually, a non-None 'list' pointer indicates a
menu or Choice, but menu nodes for symbols can sometimes have a non-None 'list'
pointer too due to submenus created implicitly from dependencies.

MenuNode.item is either a Symbol or a Choice object, or one of the constants
MENU and COMMENT. The prompt of the menu node can be found in MenuNode.prompt,
which also holds the title for menus and comments. For Symbol and Choice,
MenuNode.help holds the help text (if any, otherwise None).

Most symbols will only have a single menu node. A symbol defined in multiple
locations will have one menu node for each location. The list of menu nodes for
a Symbol or Choice can be found in the Symbol/Choice.nodes attribute.

Note that prompts and help texts for symbols and choices are stored in their
menu node(s) rather than in the Symbol or Choice objects themselves. This makes
it possible to define a symbol in multiple locations with a different prompt or
help text in each location. To get the help text or prompt for a symbol with a
single menu node, do sym.nodes[0].help and sym.nodes[0].prompt, respectively.
The prompt is a (text, condition) tuple, where condition determines the
visibility (see 'Intro to expressions' below).

This organization mirrors the C implementation. MenuNode is called
'struct menu' there, but I thought "menu" was a confusing name.

It is possible to give a Choice a name and define it in multiple locations,
hence why Choice.nodes is also a list.

As a convenience, the properties added at a particular definition location are
available on the MenuNode itself, in e.g. MenuNode.defaults. This is helpful
when generating documentation, so that symbols/choices defined in multiple
locations can be shown with the correct properties at each location.


Intro to expressions
====================

Expressions can be evaluated with the expr_value() function and printed with
the expr_str() function (these are used internally as well). Evaluating an
expression always yields a bool value, where n and y are represented as
0 and 2. Value of 1 is skipped for compatibility with the original Kconfiglib,
where there was a third state (m) for kernel modules, which is not needed nor supported
in esp-idf-kconfig.

The following table should help you figure out how expressions are represented.
A, B, C, ... are symbols (Symbol instances), NOT is the kconfiglib.NOT
constant, etc.

Expression            Representation
----------            --------------
A                     A
"A"                   A (constant symbol)
!A                    (NOT, A)
A && B                (AND, A, B)
A && B && C           (AND, A, (AND, B, C))
A || B                (OR, A, B)
A || (B && C && D)    (OR, A, (AND, B, (AND, C, D)))
A = B                 (EQUAL, A, B)
A != "foo"            (UNEQUAL, A, foo (constant symbol))
A && B = C && D       (AND, A, (AND, (EQUAL, B, C), D))
n                     Kconfig.n (constant symbol)
y                     Kconfig.y (constant symbol)
"y"                   Kconfig.y (constant symbol)

Strings like "foo" in 'default "foo"' or 'depends on SYM = "foo"' are
represented as constant symbols, so the only values that appear in expressions
are symbols***. This mirrors the C implementation.

***For choice symbols, the parent Choice will appear in expressions as well,
but it's usually invisible as the value interfaces of Symbol and Choice are
identical. This mirrors the C implementation and makes different choice modes
"just work".

Manual evaluation examples:

  - The value of A && B is min(A.bool_value, B.bool_value)

  - The value of A || B is max(A.bool_value, B.bool_value)

  - The value of !A is 2 - A.bool_value

  - The value of A = B is 2 (y) if A.str_value == B.str_value, and 0 (n)
    otherwise. Note that str_value is used here instead of bool_value.

    For constant (as well as undefined) symbols, str_value matches the name of
    the symbol. This mirrors the C implementation and explains why
    'depends on SYM = "foo"' above works as expected.

n/y are automatically converted to the corresponding constant symbols
"n"/"y" (Kconfig.n/y) during parsing.

Kconfig.const_syms is a dictionary like Kconfig.syms but for constant symbols.

If a condition is missing (e.g., <cond> when the 'if <cond>' is removed from
'default A if <cond>'), it is actually Kconfig.y. The standard __str__()
functions just avoid printing 'if y' conditions to give cleaner output.


Kconfig extensions
==================

Kconfiglib includes a couple of Kconfig extensions:

'source' with relative path
---------------------------

The 'rsource' statement sources Kconfig files with a path relative to directory
of the Kconfig file containing the 'rsource' statement, instead of relative to
the project root.

Consider following directory tree:

  Project
  +--Kconfig
  |
  +--src
     +--Kconfig
     |
     +--SubSystem1
        +--Kconfig
        |
        +--ModuleA
           +--Kconfig

In this example, assume that src/SubSystem1/Kconfig wants to source
src/SubSystem1/ModuleA/Kconfig.

With 'source', this statement would be used:

  source "src/SubSystem1/ModuleA/Kconfig"

With 'rsource', this turns into

  rsource "ModuleA/Kconfig"

If an absolute path is given to 'rsource', it acts the same as 'source'.

'rsource' can be used to create "position-independent" Kconfig trees that can
be moved around freely.


Globbing 'source'
-----------------

'source' and 'rsource' accept glob patterns, sourcing all matching Kconfig
files. They require at least one matching file, raising a KconfigError
otherwise.

For example, the following statement might source sub1/foofoofoo and
sub2/foobarfoo:

  source "sub[12]/foo*foo"

The glob patterns accepted are the same as for the standard glob.glob()
function.

Two additional statements are provided for cases where it's acceptable for a
pattern to match no files: 'osource' and 'orsource' (the o is for "optional").

For example, the following statements will be no-ops if neither "foo" nor any
files matching "bar*" exist:

  osource "foo"
  osource "bar*"

'orsource' does a relative optional source.

'source' and 'osource' are analogous to 'include' and '-include' in Make.


Extra optional warnings
-----------------------

Some optional warnings can be controlled via environment variables:

  - KCONFIG_WARN_UNDEF: If set to 'y', warnings will be generated for all
    references to undefined symbols within Kconfig files. The only gotcha is
    that all hex literals must be prefixed with "0x" or "0X", to make it
    possible to distinguish them from symbol references.

    Some projects (e.g. the Linux kernel) use multiple Kconfig trees with many
    shared Kconfig files, leading to some safe undefined symbol references.
    KCONFIG_WARN_UNDEF is useful in projects that only have a single Kconfig
    tree though.

    KCONFIG_STRICT is an older alias for this environment variable, supported
    for backwards compatibility.

  - KCONFIG_WARN_UNDEF_ASSIGN: If set to 'y', warnings will be generated for
    all assignments to undefined symbols within .config files. By default, no
    such warnings are generated.

    This warning can also be enabled/disabled via the Kconfig.warn_assign_undef
    variable.


Preprocessor user functions defined in Python
---------------------------------------------

Preprocessor functions can be defined in Python, which makes it simple to
integrate information from existing Python tools into Kconfig (e.g. to have
Kconfig symbols depend on hardware information stored in some other format).

Putting a Python module named kconfigfunctions(.py) anywhere in sys.path will
cause it to be imported by Kconfiglib (in Kconfig.__init__()). Note that
sys.path can be customized via PYTHONPATH, and includes the directory of the
module being run by default, as well as installation directories.

If the KCONFIG_FUNCTIONS environment variable is set, it gives a different
module name to use instead of 'kconfigfunctions'.

The imported module is expected to define a global dictionary named 'functions'
that maps function names to Python functions, as follows:

  def my_fn(kconf, name, arg_1, arg_2, ...):
      # kconf:
      #   Kconfig instance
      #
      # name:
      #   Name of the user-defined function ("my-fn"). Think argv[0].
      #
      # arg_1, arg_2, ...:
      #   Arguments passed to the function from Kconfig (strings)
      #
      # Returns a string to be substituted as the result of calling the
      # function
      ...

  def my_other_fn(kconf, name, arg_1, arg_2, ...):
      ...

  functions = {
      "my-fn":       (my_fn,       <min.args>, <max.args>/None),
      "my-other-fn": (my_other_fn, <min.args>, <max.args>/None),
      ...
  }

  ...

<min.args> and <max.args> are the minimum and maximum number of arguments
expected by the function (excluding the implicit 'name' argument). If
<max.args> is None, there is no upper limit to the number of arguments. Passing
an invalid number of arguments will generate a KconfigError exception.

Functions can access the current parsing location as kconf.filename/linenr.
Accessing other fields of the Kconfig object is not safe. See the warning
below.

Keep in mind that for a variable defined like 'foo = $(fn)', 'fn' will be
called only when 'foo' is expanded. If 'fn' uses the parsing location and the
intent is to use the location of the assignment, you want 'foo := $(fn)'
instead, which calls the function immediately.

Once defined, user functions can be called from Kconfig in the same way as
other preprocessor functions:

    config FOO
        ...
        depends on $(my-fn,arg1,arg2)

If my_fn() returns "n", this will result in

    config FOO
        ...
        depends on n

Warning
*******

User-defined preprocessor functions are called as they're encountered at parse
time, before all Kconfig files have been processed, and before the menu tree
has been finalized. There are no guarantees that accessing Kconfig symbols or
the menu tree via the 'kconf' parameter will work, and it could potentially
lead to a crash.

Preferably, user-defined functions should be stateless.
"""

# File layout:
#
# Public classes
# Public functions
# Internal functions
# Global constants

#
# Public classes
#


class Kconfig(object):
    """
    Represents a Kconfig configuration, e.g. for x86 or ARM. This is the set of
    symbols, choices, and menu nodes appearing in the configuration. Creating
    any number of Kconfig objects (including for different architectures) is
    safe. Kconfiglib doesn't keep any global state.

    The following attributes are available. They should be treated as
    read-only, and some are implemented through @property magic.
    """

    __slots__ = (
        "_encoding",
        "_functions",
        "_set_match",
        "_srctree_prefix",
        "_unset_match",
        "_warn_assign_no_prompt",
        "allowed_multi_def_choices",
        "allowed_multi_def_syms",
        "choices",
        "comments",
        "config_header",
        "config_prefix",
        "const_syms",
        "defconfig_list",
        "defined_syms",
        "env_vars",
        "header_header",
        "kconfig_filenames",
        "menus",
        "missing_syms",
        "n",
        "named_choices",
        "srctree",
        "syms",
        "top_node",
        "unique_choices",
        "unique_defined_syms",
        "variables",
        "info",
        "warn",
        "warn_assign_override",
        "warn_assign_redun",
        "warn_assign_undef",
        "warn_to_stderr",
        "warnings",
        "y",
        # Parsing-related
        "parser_version",
        "_parsing_kconfigs",
        "_readline",
        "root_file",
        "filename",
        "linenr",
        "_include_path",
        "_filestack",
        "_line",
        "_tokens",
        "_tokens_i",
        "_reuse_tokens",
    )
    warnings: List[str]
    syms: Dict[str, "Symbol"]
    const_syms: Dict[str, "Symbol"]
    defined_syms: List["Symbol"]
    missing_syms: List[Tuple[str, str]]
    named_choices: Dict[str, "Choice"]
    choices: List["Choice"]
    unique_choices: List["Choice"]
    menus: List["MenuNode"]
    comments: List["MenuNode"]
    variables: Dict[str, "Variable"]
    env_vars: set
    _filestack: List[str]
    allowed_multi_def_syms: Set[str]
    allowed_multi_def_choices: Set[str]

    #
    # Public interface
    #

    def __init__(
        self,
        filename="Kconfig",
        warn=True,
        info=True,
        warn_to_stderr=True,
        encoding="utf-8",
        suppress_traceback: bool = False,  # NOTE: deprecated (unused), preserved for compatibility
        parser_version=None,
    ):
        """
        Creates a new Kconfig object by parsing Kconfig files.
        Note that Kconfig files are not the same as .config files (which store
        configuration symbol values).

        See the module docstring for some environment variables that influence
        default warning settings (KCONFIG_WARN_UNDEF and
        KCONFIG_WARN_UNDEF_ASSIGN).

        Raises KconfigError on syntax/semantic errors, and OSError or (possibly
        a subclass of) IOError on IO errors ('errno', 'strerror', and
        'filename' are available). Note that IOError is an alias for OSError on
        Python 3, so it's enough to catch OSError there.

        filename (default: "Kconfig"):
          The Kconfig file to load. For the Linux kernel, you'll want "Kconfig"
          from the top-level directory, as environment variables will make sure
          the right Kconfig is included from there (arch/$SRCARCH/Kconfig as of
          writing).

          If $srctree is set, 'filename' will be looked up relative to it.
          $srctree is also used to look up source'd files within Kconfig files.

          If you are using Kconfiglib via 'make scriptconfig', the filename of
          the base base Kconfig file will be in sys.argv[1]. It's currently
          always "Kconfig" in practice.

        warn (default: True):
          True if warnings related to this configuration should be generated.
          This can be changed later by setting Kconfig.warn to True/False. It
          is provided as a constructor argument since warnings might be
          generated during parsing.

          See the other Kconfig.warn_* variables as well, which enable or
          suppress certain warnings when warnings are enabled.

          All generated warnings are added to the Kconfig.warnings list.

        warn_to_stderr (default: True):
          True if warnings should be printed to stderr in addition to being
          added to Kconfig.warnings.

          This can be changed later by setting Kconfig.warn_to_stderr to
          True/False.

        encoding (default: "utf-8"):
          The encoding to use when reading and writing files, and when decoding
          output from commands run via $(shell). If None, the encoding
          specified in the current locale will be used.

          The "utf-8" default avoids exceptions on systems that are configured
          to use the C locale, which implies an ASCII encoding.

          Related PEP: https://www.python.org/dev/peps/pep-0538/

        suppress_traceback (default: False):
          Helper for tools. When True, any EnvironmentError or KconfigError
          generated during parsing is caught, the exception message is printed
          to stderr together with the command name, and sys.exit(1) is called
          (which generates SystemExit).

          This hides the Python traceback for "expected" errors like syntax
          errors in Kconfig files.

          Other exceptions besides EnvironmentError and KconfigError are still
          propagated when suppress_traceback is True.
        """
        self._encoding = encoding

        self.parser_version = parser_version if parser_version else int(os.environ.get("KCONFIG_PARSER_VERSION", "1"))

        """
        allowed_multi_def_choices:
        allowed_multi_def_syms:

            Temporary solution how to hold a set of config/choice names that are allowed to be defined
            in multiple locations.
            Reason: In some cases, the same config/choice name is defined in multiple locations intentionally.
            See e.g. https://github.com/espressif/esp-idf/issues/15242

            Will be handled by Kconfig Report in the future.
        """
        self.allowed_multi_def_choices = set()
        self.allowed_multi_def_syms = set()

        """
        srctree:
            The value the $srctree environment variable had when the Kconfig instance
            was created, or the empty string if $srctree wasn't set. This gives nice
            behavior with os.path.join(), which treats "" as the current directory,
            without adding "./".

            Kconfig files are looked up relative to $srctree (unless absolute paths
            are used), and .config files are looked up relative to $srctree if they
            are not found in the current directory. This is used to support
            out-of-tree builds. The C tools use this environment variable in the same
            way.

            Changing $srctree after creating the Kconfig instance has no effect. Only
            the value when the configuration is loaded matters. This avoids surprises
            if multiple configurations are loaded with different values for $srctree.
        """
        self.srctree = os.getenv("srctree", "")

        # A prefix we can reliably strip from glob() results to get a filename
        # relative to $srctree. relpath() can cause issues for symlinks,
        # because it assumes symlink/../foo is the same as foo/.
        self._srctree_prefix = realpath(self.srctree) + os.sep

        """
        warn:
            Set this variable to True/False to enable/disable warnings.

            When 'warn' is False, the values of the other warning-related variables
            are ignored.

            This variable as well as the other warn* variables can be read to check
            the current warning settings.
        """
        self.warn = warn

        """
        info:
            Same as 'warn', but for informational messages.
        """
        self.info = info

        """
        warn_to_stderr:
            Set this variable to True/False to enable/disable warnings on stderr.
        """
        self.warn_to_stderr = warn_to_stderr

        """
        warn_assign_undef:
            Set this variable to True to generate warnings for assignments to
            undefined symbols in configuration files.

            This variable is False by default unless the KCONFIG_WARN_UNDEF_ASSIGN
            environment variable was set to 'y' when the Kconfig instance was
            created.
        """
        self.warn_assign_undef = os.getenv("KCONFIG_WARN_UNDEF_ASSIGN") == "y"

        """
        warn_assign_override:
            Set this variable to True to generate warnings for multiple assignments
            to the same symbol in configuration files, where the assignments set
            different values (e.g. CONFIG_FOO=n followed by CONFIG_FOO=y, where the
            last value would get used).

            This variable is True by default. Disabling it might be useful when
            merging configurations.
        """
        self.warn_assign_override = True

        """
        warn_assign_redun:
            Like warn_assign_override, but for multiple assignments setting a symbol
            to the same value.

            This variable is True by default. Disabling it might be useful when
            merging configurations.
        """
        self.warn_assign_redun = True
        self._warn_assign_no_prompt = True

        """
        warnings:
            A list of strings containing all warnings that have been generated, for
            cases where more flexibility is needed.

            See the 'warn_to_stderr' parameter to Kconfig.__init__() and the
            Kconfig.warn_to_stderr variable as well. Note that warnings still get
            added to Kconfig.warnings when 'warn_to_stderr' is True.

            Just as for warnings printed to stderr, only warnings that are enabled
            will get added to Kconfig.warnings. See the various Kconfig.warn*
            variables.
        """
        self.warnings = []

        """
        config_prefix:
            The value the CONFIG_ environment variable had when the Kconfig instance
            was created, or "CONFIG_" if CONFIG_ wasn't set. This is the prefix used
            (and expected) on symbol names in .config files and C headers. Used in
            the same way in the C tools.
        """
        self.config_prefix = os.getenv("CONFIG_", "CONFIG_")

        # Regular expressions for parsing .config files
        self._set_match = re.compile(self.config_prefix + r"([^=]+)=(.*)", re.ASCII).match
        self._unset_match = re.compile(rf"# {self.config_prefix}([^ ]+) is not set", re.ASCII).match

        """
        config_header:
            The value the KCONFIG_CONFIG_HEADER environment variable had when the
            Kconfig instance was created, or the empty string if
            KCONFIG_CONFIG_HEADER wasn't set. This string is inserted verbatim at the
            beginning of configuration files. See write_config().
        """
        self.config_header = os.getenv("KCONFIG_CONFIG_HEADER", "")

        """
        header_header:
            The value the KCONFIG_AUTOHEADER_HEADER environment variable had when the
            Kconfig instance was created, or the empty string if
            KCONFIG_AUTOHEADER_HEADER wasn't set. This string is inserted verbatim at
            the beginning of header files. See write_autoconf().
        """
        self.header_header = os.getenv("KCONFIG_AUTOHEADER_HEADER", "")

        """
        syms:
            A dictionary with all symbols in the configuration, indexed by name. Also
            includes all symbols that are referenced in expressions but never
            defined, except for constant (quoted) symbols.

            Undefined symbols can be recognized by Symbol.nodes being empty -- see
            the 'Intro to the menu tree' section in the module docstring.
        """
        self.syms = {}

        """
        const_syms:
            A dictionary like 'syms' for constant (quoted) symbols
        """
        self.const_syms = {}

        """
        defined_syms:
            A list with all defined symbols, in the same order as they appear in the
            Kconfig files. Symbols defined in multiple locations appear multiple
            times.

            Note: You probably want to use 'unique_defined_syms' instead. This
            attribute is mostly maintained for backwards compatibility.
        """
        self.defined_syms = []

        """
        missing_syms:
            A list with (name, value) tuples for all assignments to undefined symbols
            within the most recently loaded .config file(s). 'name' is the symbol
            name without the 'CONFIG_' prefix. 'value' is a string that gives the
            right-hand side of the assignment verbatim.

            See Kconfig.load_config() as well.
        """
        self.missing_syms = []

        """
        named_choices:
            A dictionary like 'syms' for named choices (choice FOO).
        """
        self.named_choices = {}

        """
        choices:
            A list with all choices, in the same order as they appear in the Kconfig
            files.

            Note: You probably want to use 'unique_choices' instead. This attribute
            is mostly maintained for backwards compatibility.
        """
        self.choices = []

        """
        menus:
            A list with all menus, in the same order as they appear in the Kconfig
            files.
        """
        self.menus = []

        """
        comments:
            A list with all comments, in the same order as they appear in the Kconfig
            files.
        """
        self.comments = []

        for ny in "n", "y":
            sym = Symbol(kconfig=self, name=ny, is_constant=True, init_rest=False)
            self.const_syms[ny] = sym

        self.n: Symbol = self.const_syms["n"]
        self.y: Symbol = self.const_syms["y"]

        for ny in "n", "y":
            sym = self.const_syms[ny]
            sym.init_rest()
            sym.orig_type = BOOL
            sym._cached_bool_val = STR_TO_BOOL[ny]

        # Maps preprocessor variables names to Variable instances
        """
        variables:
            A dictionary with all preprocessor variables, indexed by name. See the
            Variable class.
        """
        self.variables = {}

        # Predefined preprocessor functions, with min/max number of arguments
        self._functions = {
            "info": (_info_fn, 1, 1),
            "error-if": (_error_if_fn, 2, 2),
            "filename": (_filename_fn, 0, 0),
            "lineno": (_lineno_fn, 0, 0),
            "shell": (_shell_fn, 1, 1),
            "warning-if": (_warning_if_fn, 2, 2),
        }

        # Add any user-defined preprocessor functions
        try:
            self._functions.update(
                importlib.import_module(os.getenv("KCONFIG_FUNCTIONS", "kconfigfunctions")).functions
            )
        except ImportError:
            pass

        # This determines whether previously unseen symbols are registered.
        # They shouldn't be if we parse expressions after parsing, as part of
        # Kconfig.eval_string().
        self._parsing_kconfigs = True

        """
        defconfig_list:
            The Symbol instance for the 'option defconfig_list' symbol, or None if no
            defconfig_list symbol exists. The defconfig filename derived from this
            symbol can be found in Kconfig.defconfig_filename.
        """
        self.defconfig_list = None
        self._include_path = ()

        """
        top_node:
            The menu node (see the MenuNode class) of the implicit top-level menu.
            Acts as the root of the menu tree.
        """
        self.top_node = MenuNode(
            kconfig=self, item=MENU, is_menuconfig=True, prompt=("Main menu", self.y), filename=filename, linenr=1
        )

        # Parse the Kconfig files

        # Not used internally. Provided as a convenience.
        # TODO is the attribute below needed?
        """
        kconfig_filenames:
            A list with the filenames of all Kconfig files included in the
            configuration, relative to $srctree (or relative to the current directory
            if $srctree isn't set), except absolute paths (e.g.
            'source "/foo/Kconfig"') are kept as-is.

            The files are listed in the order they are source'd, starting with the
            top-level Kconfig file. If a file is source'd multiple times, it will
            appear multiple times. Use set() to get unique filenames.

            Note that Kconfig.sync_deps() already indirectly catches any file
            modifications that change configuration output.
        """
        self.kconfig_filenames = [filename]

        """
        env_vars:
            A set() with the names of all environment variables referenced in the
            Kconfig files.

            Only environment variables referenced with the preprocessor $(FOO) syntax
            will be registered. The older $FOO syntax is only supported for backwards
            compatibility.

            Also note that $(FOO) won't be registered unless the environment variable
            $FOO is actually set. If it isn't, $(FOO) is an expansion of an unset
            preprocessor variable (which gives the empty string).

            Another gotcha is that environment variables referenced in the values of
            recursively expanded preprocessor variables (those defined with =) will
            only be registered if the variable is actually used (expanded) somewhere.

            The note from the 'kconfig_filenames' documentation applies here too.
        """
        self.env_vars = set()

        # Keeps track of the location in the parent Kconfig files. Kconfig
        # files usually source other Kconfig files. See _enter_file().
        self._filestack = []

        """
        filename/linenr:
            The current parsing location, for use in Python preprocessor functions.
            See the module docstring.
        TODO: May be removed during removing preprocessor functions
        """
        self.filename = filename
        self.linenr = 0

        # Used to avoid retokenizing lines when we discover that they're not
        # part of the construct currently being parsed. This is kinda like an
        # unget operation.
        self._reuse_tokens = False
        self()

    def __call__(self):
        """
        In order to untagle the code, parsing of the Kconfig files is done in a separate function.
        """
        # Open the top-level Kconfig file. Store the readline() method directly
        # as a small optimization.
        self._readline = open(join(self.srctree, self.filename), "r", encoding=self._encoding).readline
        self.root_file = join(self.srctree, self.filename)

        try:
            # Parse the Kconfig files. Returns the last node, which we terminate with '.next = None'.
            if self.parser_version == 1:
                prev = self._parse_block(None, self.top_node, self.top_node)
                self.top_node.list = self.top_node.next
                prev.next = None
            else:
                prev = self._new_parse(None, self.top_node, self.top_node)
                prev.next = None
            self.top_node.next = None
        except UnicodeDecodeError as e:
            _decoding_error(e, self.filename)

        # Close the top-level Kconfig file. __self__ fetches the 'file' object
        # for the method.
        self._readline.__self__.close()

        self._parsing_kconfigs = False

        # Do various menu tree post-processing
        self._finalize_node(self.top_node, self.y)

        """
        unique_defined_syms:
            A list like 'defined_syms', but with duplicates removed. Just the first
            instance is kept for symbols defined in multiple locations. Kconfig order
            is preserved otherwise.

            Using this attribute instead of 'defined_syms' can save work, and
            automatically gives reasonable behavior when writing configuration output
            (symbols defined in multiple locations only generate output once, while
            still preserving Kconfig order for readability).
        """
        self.unique_defined_syms = _ordered_unique(self.defined_syms)

        """
        unique_choices:
            Analogous to 'unique_defined_syms', for choices. Named choices can have
            multiple definition locations.
        """
        self.unique_choices = _ordered_unique(self.choices)

        # Do sanity checks. Some of these depend on everything being finalized.
        self._check_sym_sanity()
        self._check_choice_sanity()

        self._check_multiple_definitions()
        # KCONFIG_STRICT is an older alias for KCONFIG_WARN_UNDEF, supported
        # for backwards compatibility
        if os.getenv("KCONFIG_WARN_UNDEF") == "y" or os.getenv("KCONFIG_STRICT") == "y":
            self._check_undef_syms()

        # Build Symbol._dependents for all symbols and choices
        self._build_dep()

        # Check for dependency loops
        check_dep_loop_sym = _check_dep_loop_sym  # Micro-optimization
        for sym in self.unique_defined_syms:
            check_dep_loop_sym(sym, False)

        # Add extra dependencies from choices to choice symbols that get
        # awkward during dependency loop detection
        self._add_choice_deps()

        return self

    @property
    def mainmenu_text(self):
        """
        mainmenu_text:
            The prompt (title) of the top menu (top_node). Defaults to "Main menu".
            Can be changed with the 'mainmenu' statement.
        """
        return self.top_node.prompt[0]

    @property
    def defconfig_filename(self):
        """
        The filename given by the defconfig_list symbol. This is taken from the
        first 'default' with a satisfied condition where the specified file
        exists (can be opened for reading). If a defconfig file foo/defconfig is
        not found and $srctree was set when the Kconfig was created,
        $srctree/foo/defconfig is looked up as well.

        'defconfig_filename' is None if either no defconfig_list symbol exists,
        or if the defconfig_list symbol has no 'default' with a satisfied
        condition that specifies a file that exists.

        Gotcha: scripts/kconfig/Makefile might pass --defconfig=<defconfig> to
        scripts/kconfig/conf when running e.g. 'make defconfig'. This option
        overrides the defconfig_list symbol, meaning defconfig_filename might not
        always match what 'make defconfig' would use.
        """
        if self.defconfig_list:
            for filename, cond in self.defconfig_list.defaults:
                if expr_value(cond):
                    try:
                        with self._open_config(filename.str_value) as f:
                            return f.name
                    except EnvironmentError:
                        continue

        return None

    def load_config(self, filename=None, replace=True, verbose=None):
        """
        Loads symbol values from a file in the .config format. Equivalent to
        calling Symbol.set_value() to set each of the values.

        "# CONFIG_FOO is not set" within a .config file sets the user value of
        FOO to n. The C tools work the same way.

        For each symbol, the Symbol._user_value attribute holds the value the
        symbol was assigned in the .config file (if any). The user value might
        differ from Symbol.str/bool_value if there are unsatisfied dependencies.

        Calling this function also updates the Kconfig.missing_syms attribute
        with a list of all assignments to undefined symbols within the
        configuration file. Kconfig.missing_syms is cleared if 'replace' is
        True, and appended to otherwise. See the documentation for
        Kconfig.missing_syms as well.

        See the Kconfig.__init__() docstring for raised exceptions
        (OSError/IOError). KconfigError is never raised here.

        filename (default: None):
          Path to load configuration from (a string). Respects $srctree if set
          (see the class documentation).

          If 'filename' is None (the default), the configuration file to load
          (if any) is calculated automatically, giving the behavior you'd
          usually want:

            1. If the KCONFIG_CONFIG environment variable is set, it gives the
               path to the configuration file to load. Otherwise, ".config" is
               used. See standard_config_filename().

            2. If the path from (1.) doesn't exist, the configuration file
               given by kconf.defconfig_filename is loaded instead, which is
               derived from the 'option defconfig_list' symbol.

            3. If (1.) and (2.) fail to find a configuration file to load, no
               configuration file is loaded, and symbols retain their current
               values (e.g., their default values). This is not an error.

           See the return value as well.

        replace (default: True):
          If True, all existing user values will be cleared before loading the
          .config. Pass False to merge configurations.

        verbose (default: None):
          Limited backwards compatibility to prevent crashes. A warning is
          printed if anything but None is passed.

          Prior to Kconfiglib 12.0.0, this option enabled printing of messages
          to stdout when 'filename' was None. A message is (always) returned
          now instead, which is more flexible.

          Will probably be removed in some future version.

        Returns a string with a message saying which file got loaded (or
        possibly that no file got loaded, when 'filename' is None). This is
        meant to reduce boilerplate in tools, which can do e.g.
        print(kconf.load_config()). The returned message distinguishes between
        loading (replace == True) and merging (replace == False).
        """
        if verbose is not None:
            _warn_verbose_deprecated("load_config")

        msg = None
        if filename is None:
            filename = standard_config_filename()
            if not exists(filename) and not exists(join(self.srctree, filename)):
                defconfig = self.defconfig_filename
                if defconfig is None:
                    return f"Using default symbol values (no '{filename}')"

                msg = f" default configuration '{defconfig}' (no '{filename}')"
                filename = defconfig

        if not msg:
            msg = f" configuration '{filename}'"

        # Disable the warning about assigning to symbols without prompts. This
        # is normal and expected within a .config file.
        self._warn_assign_no_prompt = False

        # This stub only exists to make sure _warn_assign_no_prompt gets re-enabled
        try:
            self._load_config(filename, replace)
        except UnicodeDecodeError as e:
            _decoding_error(e, filename)
        finally:
            self._warn_assign_no_prompt = True

        return ("Loaded" if replace else "Merged") + msg

    def _load_config(self, filename, replace):
        # CHOICE: (SYMBOL: VAL)*
        # When setting choice symbols one-by-one, we cannot correctly determine e.g. if the choice
        # will have any y-set symbols (e.g. first occurred choice symbol is set to n - will there be any y-set symbol?)
        # So we cache symbols with user-set value and set them additionally.
        choices_with_user_set_value: Dict[Choice, List[Tuple[Symbol, str]]] = dict()

        with self._open_config(filename) as f:
            if replace:
                self.missing_syms = []

                # If we're replacing the configuration, keep track of which
                # symbols and choices got set so that we can unset the rest
                # later. This avoids invalidating everything and is faster.
                # Another benefit is that invalidation must be rock solid for
                # it to work, making it a good test.

                for sym in self.unique_defined_syms:
                    sym._was_set = False

                for choice in self.unique_choices:
                    choice._was_set = False

            # Small optimizations
            set_match = self._set_match
            unset_match = self._unset_match
            get_sym = self.syms.get

            for linenr, line in enumerate(f, 1):
                # The C tools ignore trailing whitespace
                line = line.rstrip()

                match = set_match(line)
                if match:
                    name, val = match.groups()
                    sym = get_sym(name)
                    if not sym or not sym.nodes:
                        self._undef_assign(name, val, filename, linenr)
                        continue

                    if sym.orig_type == BOOL:
                        # The C implementation only checks the first character
                        # to the right of '=', for whatever reason
                        if not val.startswith(("y", "n")):
                            self._warn(
                                f"'{val}' is not a valid value for the {TYPE_TO_STR[sym.orig_type]} symbol "
                                f"{sym.name_and_loc}. Assignment ignored.",
                                filename,
                                linenr,
                            )
                            continue

                        val = val[0]

                        if sym.choice and val != "n":
                            # During .config/sdkconfig loading, we infer the mode of the
                            # choice from the kind of values that are assigned
                            # to the choice symbols

                            prev_mode = sym.choice._user_value
                            if prev_mode is not None and BOOL_TO_STR[prev_mode] != val:
                                self._warn(
                                    "both m and y assigned to symbols within the same choice",
                                    filename,
                                    linenr,
                                )

                            # Set the choice's mode
                            sym.choice.set_value(val)

                    elif sym.orig_type == STRING:
                        match = _conf_string_match(val)
                        if not match:
                            self._warn(
                                f"malformed string literal in assignment to {sym.name_and_loc}. Assignment ignored.",
                                filename,
                                linenr,
                            )
                            continue

                        val = unescape(match.group(1))

                else:
                    match = unset_match(line)
                    if not match:
                        # Print a warning for lines that match neither
                        # set_match() nor unset_match() and that are not blank
                        # lines or comments. 'line' has already been
                        # rstrip()'d, so blank lines show up as "" here.
                        if line and not line.lstrip().startswith("#"):
                            self._warn(
                                f"ignoring malformed line '{line}'",
                                filename,
                                linenr,
                            )

                        continue

                    name = match.group(1)
                    sym = get_sym(name)
                    if not sym or not sym.nodes:
                        self._undef_assign(name, "n", filename, linenr)
                        continue

                    if sym.orig_type != BOOL:
                        continue

                    val = "n"

                #############################################
                # Done parsing the assignment. Set the value.
                #############################################

                if sym.choice:
                    # NOTE: Order is important here; if multiple choice symbols are set to y at once,
                    #       the last one will be used.
                    if sym.choice not in choices_with_user_set_value:
                        choices_with_user_set_value[sym.choice] = []
                    choices_with_user_set_value[sym.choice].append((sym, val))
                    continue

                if sym._was_set:
                    self._assigned_twice(sym, val, filename, linenr)

                sym.set_value(val)

            #############################################
            # Done setting the values. Set special cases:
            # 1) Choice symbols
            #############################################

            for choice in choices_with_user_set_value:
                # if all symbols are set to n, we need to report that the current choice selection
                # is attempted to be set to n without setting another choice symbol to y
                if all(val == "n" for _, val in choices_with_user_set_value[choice]):
                    # Theoretically, can happen a situation where the choice selection is not set, but
                    # its symbols are still loaded from the .config/sdkconfig file.
                    choice_selection = choice.selection
                    if choice_selection is not None:
                        self._info(
                            msg=(
                                f"Trying to set symbol {choice_selection.name} to n, but it is currently selected "
                                f"by choice {choice.name}. For setting it to n, set another choice symbol to y instead."
                            ),
                        )
                # if there are multiple active selections, report that the last one will be used
                if len([val for _, val in choices_with_user_set_value[choice] if val == "y"]) > 1:
                    active_selections = [sym.name for sym, val in choices_with_user_set_value[choice] if val == "y"]
                    self._info(
                        msg=(
                            f"Choice {choice.name} has multiple active selections. "
                            f"The last one will be used: {active_selections[-1]} ."
                        ),
                    )

                for sym, val in choices_with_user_set_value[choice]:
                    sym.set_value(val)

        if replace:
            # If we're replacing the configuration, unset the symbols that
            # didn't get set

            for sym in self.unique_defined_syms:
                if not sym._was_set:
                    sym.unset_value()

            for choice in self.unique_choices:
                if not choice._was_set:
                    choice.unset_value()

    def _undef_assign(self, name, val, filename, linenr):
        # Called for assignments to undefined symbols during .config loading

        self.missing_syms.append((name, val))
        if self.warn_assign_undef:
            self._warn(
                f"attempt to assign the value '{val}' to the undefined symbol {name}",
                filename,
                linenr,
            )

    def _assigned_twice(self, sym, new_val, filename, linenr):
        # Called when a symbol is assigned more than once in a .config file

        # Use strings for bool user values in the warning
        if sym.orig_type == BOOL:
            user_val = BOOL_TO_STR[sym._user_value]
        else:
            user_val = sym._user_value

        msg = f'{sym.name_and_loc} set more than once. Old value "{user_val}", new value "{new_val}".'

        if user_val == new_val:
            if self.warn_assign_redun:
                self._warn(msg, filename, linenr)
        elif self.warn_assign_override:
            self._warn(msg, filename, linenr)

    def write_autoconf(self, filename=None, header=None):
        r"""
        Writes out symbol values as a C header file, matching the format used
        by include/generated/autoconf.h in the kernel.

        The ordering of the #defines matches the one generated by
        write_config(). The order in the C implementation depends on the hash
        table implementation as of writing, and so won't match.

        If 'filename' exists and its contents is identical to what would get
        written out, it is left untouched. This avoids updating file metadata
        like the modification time and possibly triggering redundant work in
        build tools.

        filename (default: None):
          Path to write header to.

          If None (the default), the path in the environment variable
          KCONFIG_AUTOHEADER is used if set, and "include/generated/autoconf.h"
          otherwise. This is compatible with the C tools.

        header (default: None):
          Text inserted verbatim at the beginning of the file. You would
          usually want it enclosed in '/* */' to make it a C comment, and
          include a trailing newline.

          If None (the default), the value of the environment variable
          KCONFIG_AUTOHEADER_HEADER had when the Kconfig instance was created
          will be used if it was set, and no header otherwise. See the
          Kconfig.header_header attribute.

        Returns a string with a message saying that the header got saved, or
        that there were no changes to it. This is meant to reduce boilerplate
        in tools, which can do e.g. print(kconf.write_autoconf()).
        """
        if filename is None:
            filename = os.getenv("KCONFIG_AUTOHEADER", "include/generated/autoconf.h")

        if self._write_if_changed(filename, self._autoconf_contents(header)):
            return f"Kconfig header saved to '{filename}'"
        return f"No change to Kconfig header in '{filename}'"

    def _autoconf_contents(self, header):
        # write_autoconf() helper. Returns the contents to write as a string,
        # with 'header' or KCONFIG_AUTOHEADER_HEADER at the beginning.

        if header is None:
            header = self.header_header

        chunks = [header]  # "".join()ed later
        add = chunks.append

        for sym in self.unique_defined_syms:
            # _write_to_conf is determined when the value is calculated. This
            # is a hidden function call due to property magic.
            #
            # Note: In client code, you can check if sym.config_string is empty
            # instead, to avoid accessing the internal _write_to_conf variable
            # (though it's likely to keep working).
            val = sym.str_value
            if not sym._write_to_conf:
                continue

            if sym.orig_type == BOOL and val == "y":
                add(f"#define {self.config_prefix}{sym.name} 1\n")

            elif sym.orig_type == STRING:
                add(f'#define {self.config_prefix}{sym.name} "{escape(val)}"\n')

            elif sym.orig_type in _INT_HEX:
                if sym.orig_type == HEX and not val.startswith(("0x", "0X")):
                    val = "0x" + val
                add(f"#define {self.config_prefix}{sym.name} {val}\n")

        return "".join(chunks)

    def write_config(self, filename=None, header=None, save_old=True, verbose=None):
        r"""
        Writes out symbol values in the .config format. The format matches the
        C implementation, including ordering.

        Symbols appear in the same order in generated .config files as they do
        in the Kconfig files. For symbols defined in multiple locations, a
        single assignment is written out corresponding to the first location
        where the symbol is defined.

        See the 'Intro to symbol values' section in the module docstring to
        understand which symbols get written out.

        If 'filename' exists and its contents is identical to what would get
        written out, it is left untouched. This avoids updating file metadata
        like the modification time and possibly triggering redundant work in
        build tools.

        See the Kconfig.__init__() docstring for raised exceptions
        (OSError/IOError). KconfigError is never raised here.

        filename (default: None):
          Path to write configuration to (a string).

          If None (the default), the path in the environment variable
          KCONFIG_CONFIG is used if set, and ".config" otherwise. See
          standard_config_filename().

        header (default: None):
          Text inserted verbatim at the beginning of the file. You would
          usually want each line to start with '#' to make it a comment, and
          include a trailing newline.

          if None (the default), the value of the environment variable
          KCONFIG_CONFIG_HEADER had when the Kconfig instance was created will
          be used if it was set, and no header otherwise. See the
          Kconfig.config_header attribute.

        save_old (default: True):
          If True and <filename> already exists, a copy of it will be saved to
          <filename>.old in the same directory before the new configuration is
          written.

          Errors are silently ignored if <filename>.old cannot be written (e.g.
          due to being a directory, or <filename> being something like
          /dev/null).

        verbose (default: None):
          Limited backwards compatibility to prevent crashes. A warning is
          printed if anything but None is passed.

          Prior to Kconfiglib 12.0.0, this option enabled printing of messages
          to stdout when 'filename' was None. A message is (always) returned
          now instead, which is more flexible.

          Will probably be removed in some future version.

        Returns a string with a message saying which file got saved. This is
        meant to reduce boilerplate in tools, which can do e.g.
        print(kconf.write_config()).
        """
        if verbose is not None:
            _warn_verbose_deprecated("write_config")

        if filename is None:
            filename = standard_config_filename()

        contents = self._config_contents(header)
        if self._contents_eq(filename, contents):
            return f"No change to configuration in '{filename}'"

        if save_old:
            _save_old(filename)

        with open(filename, "w", encoding=self._encoding) as f:
            f.write(contents)

        return f"Configuration saved to '{filename}'"

    def return_config(self):
        return self._config_contents(None)

    def _config_contents(self, header):
        # write_config() helper. Returns the contents to write as a string,
        # with 'header' or KCONFIG_CONFIG_HEADER at the beginning.
        #
        # More memory friendly would be to 'yield' the strings and
        # "".join(_config_contents()), but it was a bit slower on my system.

        # node_iter() was used here before commit 3aea9f7 ("Add '# end of
        # <menu>' after menus in .config"). Those comments get tricky to
        # implement with it.

        for sym in self.unique_defined_syms:
            sym._visited = False

        if header is None:
            header = self.config_header

        chunks = [header]  # "".join()ed later
        add = chunks.append

        # Did we just print an '# end of ...' comment?
        after_end_comment = False

        node = self.top_node
        while 1:
            # Jump to the next node with an iterative tree walk
            if node.list:
                node = node.list
            elif node.next:
                node = node.next
            else:
                while node.parent:
                    node = node.parent

                    # Add a comment when leaving visible menus
                    if (
                        node.item == MENU
                        and expr_value(node.dep)
                        and expr_value(node.visibility)
                        and node is not self.top_node
                    ):
                        add(f"# end of {node.prompt[0]}\n")
                        after_end_comment = True

                    if node.next:
                        node = node.next
                        break
                else:
                    # No more nodes
                    return "".join(chunks)

            # Generate configuration output for the node

            item = node.item

            if item.__class__ is Symbol:
                if item._visited:
                    continue
                item._visited = True

                conf_string = item.config_string
                if not conf_string:
                    continue

                if after_end_comment:
                    # Add a blank line before the first symbol printed after an
                    # '# end of ...' comment
                    after_end_comment = False
                    add("\n")
                add(conf_string)

            elif expr_value(node.dep) and ((item == MENU and expr_value(node.visibility)) or item == COMMENT):
                add(f"\n#\n# {node.prompt[0]}\n#\n")
                after_end_comment = False

    def write_min_config(self, filename, header=None, labels=False, normalize_unset=False):
        """
        Writes out a "minimal" configuration file, omitting symbols whose value
        matches their default value. The format matches the one produced by
        'make savedefconfig'.

        The resulting configuration file is incomplete, but a complete
        configuration can be derived from it by loading it. Minimal
        configuration files can serve as a more manageable configuration format
        compared to a "full" .config file, especially when configurations files
        are merged or edited by hand.

        See the Kconfig.__init__() docstring for raised exceptions
        (OSError/IOError). KconfigError is never raised here.

        filename:
          Path to write minimal configuration to.

        header (default: None):
          Text inserted verbatim at the beginning of the file. You would
          usually want each line to start with '#' to make it a comment, and
          include a final terminating newline.

          if None (the default), the value of the environment variable
          KCONFIG_CONFIG_HEADER had when the Kconfig instance was created will
          be used if it was set, and no header otherwise. See the
          Kconfig.config_header attribute.

        labels (default: False):
          When True, emit menu section labels (``# Menu`` /
          ``# end of Menu``) around groups of non-default symbols, using
          the same MenuNode tree walk as write_config().

        normalize_unset (default: False):
          When True, rewrite ``# CONFIG_FOO is not set`` lines to
          ``CONFIG_FOO=n`` in order to help users understand the generated file.

        Returns a string with a message saying the minimal configuration got
        saved, or that there were no changes to it. This is meant to reduce
        boilerplate in tools, which can do e.g.
        print(kconf.write_min_config()).
        """
        contents = self._min_config_contents(header, labels=labels)

        if normalize_unset:
            unset_match = re.compile(r"# {}([^ ]+) is not set".format(self.config_prefix)).match
            lines = contents.splitlines()
            for idx, line in enumerate(lines):
                match = unset_match(line)
                if match:
                    lines[idx] = f"{self.config_prefix}{match.group(1)}=n"
            if lines:
                lines[-1] += "\n"
            contents = "\n".join(lines)

        if self._write_if_changed(filename, contents):
            return f"Minimal configuration saved to '{filename}'"
        return f"No change to minimal configuration in '{filename}'"

    def _min_config_contents(self, header, labels=False):
        """
        write_min_config() helper. Returns the contents to write as a string,
        with 'header' or KCONFIG_CONFIG_HEADER at the beginning.

        When labels=True, uses the same MenuNode tree walk as
        _config_contents() to emit menu section labels around groups of
        non-default symbols (only for sections that actually contain at
        least one such symbol).
        """

        if header is None:
            header = self.config_header

        chunks = [header]  # "".join()ed later
        add = chunks.append

        if labels:
            return self._min_config_contents_with_labels(chunks, add)

        for sym in self.unique_defined_syms:
            # Skip symbols that cannot be changed. Only check
            # non-choice symbols, as selects don't affect choice
            # symbols.
            if not sym.choice and sym.visibility <= expr_value(sym.rev_dep):
                continue

            # Skip symbols whose value matches their default
            if sym.str_value == sym._str_default():
                continue

            if (
                sym.choice
                and sym.choice._selection_from_defaults() is sym
                and sym.orig_type == BOOL
                and sym.bool_value == 2
            ):
                continue

            add(sym.config_string)

        return "".join(chunks)

    def _is_min_config_sym(self, sym):
        """
        Return True if 'sym' should be included in the minimal config
        (i.e. its value differs from the default). Shared by
        _min_config_contents() and _min_config_contents_with_labels().
        """

        if not sym.choice and sym.visibility <= expr_value(sym.rev_dep):
            return False
        if sym.str_value == sym._str_default():
            return False
        if (
            sym.choice
            and sym.choice._selection_from_defaults() is sym
            and sym.orig_type == BOOL
            and sym.bool_value == 2
        ):
            return False
        return True

    def _min_config_contents_with_labels(self, chunks, add):
        """
        Tree-walk variant of _min_config_contents that emits menu section
        labels (same format as _config_contents) around groups of
        non-default symbols.

        Uses a deferred label stack: menu labels are only emitted when
        the first minimal symbol inside that section is encountered.
        `# end of ...` markers are emitted only for sections that had
        content.
        """

        for sym in self.unique_defined_syms:
            sym._visited = False

        # Each entry: [prompt_text, emitted: bool]
        label_stack = []
        after_end_comment = False

        node = self.top_node
        while 1:
            if node.list:
                node = node.list
            elif node.next:
                node = node.next
            else:
                while node.parent:
                    node = node.parent

                    if (
                        node.item == MENU
                        and expr_value(node.dep)
                        and expr_value(node.visibility)
                        and node is not self.top_node
                    ):
                        if label_stack:
                            prompt, emitted = label_stack.pop()
                            if emitted:
                                add(f"# end of {prompt}\n")
                                after_end_comment = True

                    if node.next:
                        node = node.next
                        break
                else:
                    return "".join(chunks)

            item = node.item

            if item.__class__ is Symbol:
                if item._visited:
                    continue
                item._visited = True

                if not self._is_min_config_sym(item):
                    continue

                conf_string = item.config_string
                if not conf_string:
                    continue

                # Emit deferred labels for all ancestor menus not yet printed
                for entry in label_stack:
                    if not entry[1]:
                        entry[1] = True
                        add(f"\n#\n# {entry[0]}\n#\n")
                        after_end_comment = False

                if after_end_comment:
                    after_end_comment = False
                    add("\n")
                add(conf_string)

            elif item == MENU and expr_value(node.dep) and expr_value(node.visibility) and node is not self.top_node:
                label_stack.append([node.prompt[0], False])

    def node_iter(self, unique_syms=False):
        """
        Returns a generator for iterating through all MenuNode's in the Kconfig
        tree. The iteration is done in Kconfig definition order (each node is
        visited before its children, and the children of a node are visited
        before the next node).

        The Kconfig.top_node menu node is skipped. It contains an implicit menu
        that holds the top-level items.

        As an example, the following code will produce a list equal to
        Kconfig.defined_syms:

          defined_syms = [node.item for node in kconf.node_iter()
                          if isinstance(node.item, Symbol)]

        unique_syms (default: False):
          If True, only the first MenuNode will be included for symbols defined
          in multiple locations.

          Using kconf.node_iter(True) in the example above would give a list
          equal to unique_defined_syms.
        """
        if unique_syms:
            for sym in self.unique_defined_syms:
                sym._visited = False

        node = self.top_node
        while 1:
            # Jump to the next node with an iterative tree walk
            if node.list:
                node = node.list
            elif node.next:
                node = node.next
            else:
                while node.parent:
                    node = node.parent
                    if node.next:
                        node = node.next
                        break
                else:
                    # No more nodes
                    return

            if unique_syms and node.item.__class__ is Symbol:
                if node.item._visited:
                    continue
                node.item._visited = True

            yield node

    def eval_string(self, s):
        """
        Returns the bool value of the expression 's', represented as 0 and 2 for n, and y.
        Raises KconfigError on syntax errors. Warns if undefined symbols are referenced.

        As an example, if FOO and BAR are bool symbols at least one of
        which has the value y, then eval_string("y && (FOO || BAR)") returns
        2 (y).

        To get the string value of non-bool symbols, use
        Symbol.str_value. eval_string() always returns a bool value, and
        all non-bool symbols have the bool value 0 (n).

        The expression parsing is consistent with how parsing works for
        conditional ('if ...') expressions in the configuration, and matches
        the C implementation.
        """
        # The parser is optimized to be fast when parsing Kconfig files (where
        # an expression can never appear at the beginning of a line). We have
        # to monkey-patch things a bit here to reuse it.

        self.filename = None

        self._tokens = self._tokenize("if " + s)
        # Strip "if " to avoid giving confusing error messages
        self._line = s
        self._tokens_i = 1  # Skip the 'if' token

        return expr_value(self._expect_expr_and_eol())

    def __repr__(self):
        """
        Returns a string with information about the Kconfig object when it is
        evaluated on e.g. the interactive Python prompt.
        """

        def status(flag):
            return "enabled" if flag else "disabled"

        return "<{}>".format(
            ", ".join(
                (
                    f"configuration with {len(self.syms)} symbols",
                    f'main menu prompt "{self.mainmenu_text}"',
                    "srctree is current directory" if not self.srctree else f'srctree "{self.srctree}"',
                    f'config symbol prefix "{self.config_prefix}"',
                    f"warnings {status(self.warn)}",
                    f"printing of warnings to stderr {status(self.warn_to_stderr)}",
                    f"undef. symbol assignment warnings {status(self.warn_assign_undef)}",
                    f"overriding symbol assignment warnings {status(self.warn_assign_override)}",
                    f"redundant symbol assignment warnings {status(self.warn_assign_redun)}",
                )
            )
        )

    #
    # Private methods
    #

    #
    # File reading
    #

    def _open_config(self, filename):
        # Opens a .config file. First tries to open 'filename', then
        # '$srctree/filename' if $srctree was set when the configuration was
        # loaded.

        try:
            return open(filename, "r", encoding=self._encoding)
        except OSError:
            # This will try opening the same file twice if $srctree is unset,
            # but it's not a big deal
            try:
                return open(join(self.srctree, filename), "r", encoding=self._encoding)
            except OSError as e:
                env_var_value = f"set to '{self.srctree}'" if self.srctree else "unset or blank"
                raise _KconfigIOError(
                    e,
                    f"Could not open '{filename}' ({errno.errorcode[e.errno]}: {e.strerror}). Check that the $srctree "
                    f"environment variable ({env_var_value}) is set correctly.",
                )

    def _enter_file(self, filename):
        # Jumps to the beginning of a sourced Kconfig file, saving the previous
        # position and file object.
        #
        # filename:
        #   Absolute path to file

        # Path relative to $srctree, stored in e.g. self.filename (which makes
        # it indirectly show up in MenuNode.filename). Equals 'filename' for
        # absolute paths passed to 'source'.
        if filename.startswith(self._srctree_prefix):
            # Relative path (or a redundant absolute path to within $srctree,
            # but it's probably fine to reduce those too)
            rel_filename = filename[len(self._srctree_prefix) :]
        else:
            # Absolute path
            rel_filename = filename

        self.kconfig_filenames.append(rel_filename)

        # The parent Kconfig files are represented as a list of
        # (<include path>, <Python 'file' object for Kconfig file>) tuples.
        #
        # <include path> is immutable and holds a *tuple* of
        # (<filename>, <linenr>) tuples, giving the locations of the 'source'
        # statements in the parent Kconfig files. The current include path is
        # also available in Kconfig._include_path.
        #
        # The point of this redundant setup is to allow Kconfig._include_path
        # to be assigned directly to MenuNode.include_path without having to
        # copy it, sharing it wherever possible.

        # Save include path and 'file' object (via its 'readline' function)
        # before entering the file
        self._filestack.append((self._include_path, self._readline))

        # _include_path is a tuple, so this rebinds the variable instead of
        # doing in-place modification
        self._include_path += ((self.filename, self.linenr),)

        # Check for recursive 'source'
        for name, _ in self._include_path:
            if name == rel_filename:
                raise KconfigError(
                    "\n{}:{}: recursive 'source' of '{}' detected. Check that "
                    "environment variables are set correctly.\n"
                    "Include path:\n{}".format(
                        self.filename,
                        self.linenr,
                        rel_filename,
                        "\n".join(f"{name}:{linenr}" for name, linenr in self._include_path),
                    )
                )

        try:
            self._readline = open(filename, "r", encoding=self._encoding).readline
        except EnvironmentError as e:
            # We already know that the file exists
            raise _KconfigIOError(
                e,
                f"{self.filename}:{self.linenr}: Could not open '{filename}' (in '{self._line.strip()}') "
                f"({errno.errorcode[e.errno]}: {e.strerror})",
            )

        self.filename = rel_filename
        self.linenr = 0

    def _leave_file(self):
        # Returns from a Kconfig file to the file that sourced it. See
        # _enter_file().

        # Restore location from parent Kconfig file
        self.filename, self.linenr = self._include_path[-1]
        # Restore include path and 'file' object
        self._readline.__self__.close()  # __self__ fetches the 'file' object
        self._include_path, self._readline = self._filestack.pop()

    def _tokenize_line(self):
        # Fetches and tokenizes the next line from the current Kconfig file.
        # Returns False at EOF and True otherwise.

        # We might already have tokens from parsing a line and discovering that
        # it's part of a different construct
        if self._reuse_tokens:
            self._reuse_tokens = False
            # self._tokens_i is known to be 1 here, because _parse_props()
            # leaves it like that when it can't recognize a line (or parses a
            # help text)
            return True

        # readline() returns '' over and over at EOF, which we rely on for help
        # texts at the end of files (see _line_after_help())
        line = self._readline()
        if not line:
            return False
        self.linenr += 1

        # Handle line joining
        while line.endswith("\\\n"):
            line = line[:-2] + self._readline()
            self.linenr += 1

        self._tokens = self._tokenize(line)
        # Initialize to 1 instead of 0 to factor out code from _parse_block()
        # and _parse_props(). They immediately fetch self._tokens[0].
        self._tokens_i = 1

        return True

    def _line_after_help(self, line):
        # Tokenizes a line after a help text. This case is special in that the
        # line has already been fetched (to discover that it isn't part of the
        # help text).
        #
        # An earlier version used a _saved_line variable instead that was
        # checked in _next_line(). This special-casing gets rid of it and makes
        # _reuse_tokens alone sufficient to handle unget.

        # Handle line joining
        while line.endswith("\\\n"):
            line = line[:-2] + self._readline()
            self.linenr += 1

        self._tokens = self._tokenize(line)
        self._reuse_tokens = True

    def _write_if_changed(self, filename, contents):
        # Writes 'contents' into 'filename', but only if it differs from the
        # current contents of the file.
        #
        # Another variant would be write a temporary file on the same
        # filesystem, compare the files, and rename() the temporary file if it
        # differs, but it breaks stuff like write_config("/dev/null"), which is
        # used out there to force evaluation-related warnings to be generated.
        # This simple version is pretty failsafe and portable.
        #
        # Returns True if the file has changed and is updated, and False
        # otherwise.

        if self._contents_eq(filename, contents):
            return False
        with open(filename, "w", encoding=self._encoding) as f:
            f.write(contents)
        return True

    def _contents_eq(self, filename, contents):
        # Returns True if the contents of 'filename' is 'contents' (a string),
        # and False otherwise (including if 'filename' can't be opened/read)

        try:
            with open(filename, "r", encoding=self._encoding) as f:
                # Robust re. things like encoding and line endings (mmap()
                # trickery isn't)
                return f.read(len(contents) + 1) == contents
        except EnvironmentError:
            # If the error here would prevent writing the file as well, we'll
            # notice it later
            return False

    #
    # Tokenization
    #

    def _lookup_sym(self, name):
        # Fetches the symbol 'name' from the symbol table, creating and
        # registering it if it does not exist. If '_parsing_kconfigs' is False,
        # it means we're in eval_string(), and new symbols won't be registered.

        if name in self.syms:
            return self.syms[name]

        sym = Symbol(kconfig=self, name=name, is_constant=False)

        if self._parsing_kconfigs:
            self.syms[name] = sym
        else:
            self._warn(f"no symbol {name} in configuration")

        return sym

    def _lookup_const_sym(self, name: str) -> "Symbol":
        # Like _lookup_sym(), for constant (quoted) symbols

        if name in self.const_syms:
            return self.const_syms[name]  # type: ignore

        sym = Symbol(kconfig=self, name=name, is_constant=True)

        if self._parsing_kconfigs:
            self.const_syms[name] = sym

        return sym

    def check_pragmas(self, line: str) -> None:
        if _KCONFIG_IGNORE_PRAGMA in line:
            match = _kconfig_ignore_match(line)
            if match:
                sym_choice_name = match.groups()[1]
                if sym_choice_name:
                    if match.group("type") in (_MULTIPLE_DEFINITION_LONG, _MULTIPLE_DEFINITION_SHORT):
                        if match.group("option") == "config":
                            self.allowed_multi_def_syms.add(sym_choice_name)
                        elif match.group("option") == "choice":
                            self.allowed_multi_def_choices.add(sym_choice_name)

    def _tokenize(self, s):
        # Parses 's', returning a None-terminated list of tokens. Registers any
        # new symbols encountered with _lookup(_const)_sym().
        #
        # Tries to be reasonably speedy by processing chunks of text via
        # regexes and string operations where possible. This is the biggest
        # hotspot during parsing.
        #
        # It might be possible to rewrite this to 'yield' tokens instead,
        # working across multiple lines. Lookback and compatibility with old
        # janky versions of the C tools complicate things though.

        self._line = s  # Used for error reporting
        self.check_pragmas(s)

        # Initial token on the line
        match = _command_match(s)
        if not match:
            if s.isspace() or s.lstrip().startswith("#"):
                return (None,)
            self._parse_error("unknown token at start of line")

        # Tricky implementation detail: While parsing a token, 'token' refers
        # to the previous token. See _STRING_LEX for why this is needed.
        token = _get_keyword(match.group(1))
        if not token:
            # Backwards compatibility with old versions of the C tools, which
            # (accidentally) accepted stuff like "--help--" and "-help---".
            # This was fixed in the C tools by commit c2264564 ("kconfig: warn
            # of unhandled characters in Kconfig commands"), committed in July
            # 2015, but it seems people still run Kconfiglib on older kernels.
            if s.strip(" \t\n-") == "help":
                return (_T_HELP, None)

            # If the first token is not a keyword (and not a weird help token),
            # we have a preprocessor variable assignment (or a bare macro on a
            # line)
            self._parse_assignment(s)
            return (None,)

        tokens = [token]
        # The current index in the string being tokenized
        i = match.end()

        # Main tokenization loop (for tokens past the first one)
        while i < len(s):
            # Test for an identifier/keyword first. This is the most common
            # case.
            match = _id_keyword_match(s, i)
            if match:
                # We have an identifier or keyword

                # Check what it is. lookup_sym() will take care of allocating
                # new symbols for us the first time we see them. Note that
                # 'token' still refers to the previous token.

                name = match.group(1)
                keyword = _get_keyword(name)
                if keyword:
                    # It's a keyword
                    token = keyword
                    # Jump past it
                    i = match.end()

                elif token not in _STRING_LEX:
                    # It's a non-const symbol, except we translate n, and y
                    # into the corresponding constant symbols, like the C
                    # implementation

                    if "$" in name:
                        # Macro expansion within symbol name
                        name, s, i = self._expand_name(s, i)
                    else:
                        i = match.end()

                    token = self.const_syms[name] if name in STR_TO_BOOL else self._lookup_sym(name)

                else:
                    # It's a case of missing quotes. For example, the
                    # following is accepted:
                    #
                    #   menu unquoted_title
                    #
                    #   config A
                    #       bool unquoted_prompt
                    #
                    #   endmenu
                    #
                    # Named choices ('choice FOO') also end up here.

                    if token is not _T_CHOICE:
                        self._warn(
                            f"style: quotes recommended around '{name}' in '{self._line.strip()}'",
                            self.filename,
                            self.linenr,
                        )

                    token = name
                    i = match.end()

            else:
                # Neither a keyword nor a non-const symbol

                # We always strip whitespace after tokens, so it is safe to
                # assume that s[i] is the start of a token here.
                c = s[i]

                if c in "\"'":
                    if "$" not in s and "\\" not in s:
                        # Fast path for lines without $ and \. Find the
                        # matching quote.
                        end_i = s.find(c, i + 1) + 1
                        if not end_i:
                            self._parse_error("unterminated string")
                        val = s[i + 1 : end_i - 1]
                        i = end_i
                    else:
                        # Slow path
                        s, end_i = self._expand_str(s, i)

                        # os.path.expandvars() and the $UNAME_RELEASE replace()
                        # is a backwards compatibility hack, which should be
                        # reasonably safe as expandvars() leaves references to
                        # undefined env. vars. as is.
                        #
                        # The preprocessor functionality changed how
                        # environment variables are referenced, to $(FOO).
                        val = expandvars(s[i + 1 : end_i - 1].replace("$UNAME_RELEASE", _UNAME_RELEASE))

                        i = end_i

                    # This is the only place where we don't survive with a
                    # single token of lookback: 'option env="FOO"' does not
                    # refer to a constant symbol named "FOO".
                    token = val if token in _STRING_LEX or tokens[0] is _T_OPTION else self._lookup_const_sym(val)

                elif s.startswith("&&", i):
                    token = _T_AND
                    i += 2

                elif s.startswith("||", i):
                    token = _T_OR
                    i += 2

                elif c == "=":
                    token = _T_EQUAL
                    i += 1

                elif s.startswith("!=", i):
                    token = _T_UNEQUAL
                    i += 2

                elif c == "!":
                    token = _T_NOT
                    i += 1

                elif c == "(":
                    token = _T_OPEN_PAREN
                    i += 1

                elif c == ")":
                    token = _T_CLOSE_PAREN
                    i += 1

                elif c == "#":
                    break

                # Very rare

                elif s.startswith("<=", i):
                    token = _T_LESS_EQUAL
                    i += 2

                elif c == "<":
                    token = _T_LESS
                    i += 1

                elif s.startswith(">=", i):
                    token = _T_GREATER_EQUAL
                    i += 2

                elif c == ">":
                    token = _T_GREATER
                    i += 1

                else:
                    self._parse_error("unknown tokens in line")

                # Skip trailing whitespace
                while i < len(s) and s[i].isspace():
                    i += 1

            # Add the token
            tokens.append(token)

        # None-terminating the token list makes token fetching simpler/faster
        tokens.append(None)

        return tokens

    # End of _tokenize()

    # Helpers for syntax checking and token fetching. See the
    # 'Intro to expressions' section for what a constant symbol is.
    #
    # More of these could be added, but the single-use cases are inlined as an
    # optimization.

    def _expect_sym(self):
        token = self._tokens[self._tokens_i]
        self._tokens_i += 1

        if token.__class__ is not Symbol:
            self._parse_error("expected symbol")

        return token

    def _expect_nonconst_sym(self):
        # Used for 'select' and 'imply' only. We know the token indices.

        token = self._tokens[1]
        self._tokens_i = 2

        if token.__class__ is not Symbol or token.is_constant:
            self._parse_error("expected nonconstant symbol")

        return token

    def _expect_str_and_eol(self):
        token = self._tokens[self._tokens_i]
        self._tokens_i += 1

        if token.__class__ is not str:
            self._parse_error("expected string")

        if self._tokens[self._tokens_i] is not None:
            self._trailing_tokens_error()

        return token

    def _expect_expr_and_eol(self):
        expr = self._parse_expr()

        if self._tokens[self._tokens_i] is not None:
            self._trailing_tokens_error()

        return expr

    def _check_token(self, token):
        # If the next token is 'token', removes it and returns True

        if self._tokens[self._tokens_i] is token:
            self._tokens_i += 1
            return True
        return False

    #
    # Preprocessor logic
    #

    def _parse_assignment(self, s):
        # Parses a preprocessor variable assignment, registering the variable
        # if it doesn't already exist. Also takes care of bare macros on lines
        # (which are allowed, and can be useful for their side effects).

        # Expand any macros in the left-hand side of the assignment (the
        # variable name)
        s = s.lstrip()
        i = 0
        while 1:
            i = _assignment_lhs_fragment_match(s, i).end()
            if s.startswith("$(", i):
                s, i = self._expand_macro(s, i, ())
            else:
                break

        if s.isspace():
            # We also accept a bare macro on a line (e.g.
            # $(warning-if,$(foo),ops)), provided it expands to a blank string
            return

        # Assigned variable
        name = s[:i]

        # Extract assignment operator (=, :=, or +=) and value
        rhs_match = _assignment_rhs_match(s, i)
        if not rhs_match:
            self._parse_error("syntax error")

        op, val = rhs_match.groups()

        if name in self.variables:
            # Already seen variable
            var = self.variables[name]
        else:
            # New variable
            var = Variable()
            var.kconfig = self
            var.name = name
            var._n_expansions = 0
            self.variables[name] = var

            # += acts like = on undefined variables (defines a recursive
            # variable)
            if op == "+=":
                op = "="

        if op == "=":
            var.is_recursive = True
            var.value = val
        elif op == ":=":
            var.is_recursive = False
            var.value = self._expand_whole(val, ())
        else:  # op == "+="
            # += does immediate expansion if the variable was last set
            # with :=
            var.value += " " + (val if var.is_recursive else self._expand_whole(val, ()))

    def _expand_whole(self, s, args):
        # Expands preprocessor macros in all of 's'. Used whenever we don't
        # have to worry about delimiters. See _expand_macro() re. the 'args'
        # parameter.
        #
        # Returns the expanded string.

        i = 0
        while 1:
            i = s.find("$(", i)
            if i == -1:
                break
            s, i = self._expand_macro(s, i, args)
        return s

    def _expand_name(self, s, i):
        # Expands a symbol name starting at index 'i' in 's'.
        #
        # Returns the expanded name, the expanded 's' (including the part
        # before the name), and the index of the first character in the next
        # token after the name.

        s, end_i = self._expand_name_iter(s, i)
        name = s[i:end_i]
        # isspace() is False for empty strings
        if not name.strip():
            # Avoid creating a Kconfig symbol with a blank name. It's almost
            # guaranteed to be an error.
            self._parse_error("macro expanded to blank string")

        # Skip trailing whitespace
        while end_i < len(s) and s[end_i].isspace():
            end_i += 1

        return name, s, end_i

    def _expand_name_iter(self, s, i):
        # Expands a symbol name starting at index 'i' in 's'.
        #
        # Returns the expanded 's' (including the part before the name) and the
        # index of the first character after the expanded name in 's'.

        while 1:
            match = _name_special_search(s, i)

            if match.group() != "$(":
                return (s, match.start())
            s, i = self._expand_macro(s, match.start(), ())

    def _expand_str(self, s, i):
        # Expands a quoted string starting at index 'i' in 's'. Handles both
        # backslash escapes and macro expansion.
        #
        # Returns the expanded 's' (including the part before the string) and
        # the index of the first character after the expanded string in 's'.

        quote = s[i]
        i += 1  # Skip over initial "/'
        while 1:
            match = _string_special_search(s, i)
            if not match:
                self._parse_error("unterminated string")

            if match.group() == quote:
                # Found the end of the string
                return (s, match.end())

            elif match.group() == "\\":
                # Replace '\x' with 'x'. 'i' ends up pointing to the character
                # after 'x', which allows macros to be canceled with '\$(foo)'.
                i = match.end()
                s = s[: match.start()] + s[i:]

            elif match.group() == "$(":
                # A macro call within the string
                s, i = self._expand_macro(s, match.start(), ())

            else:
                # A ' quote within " quotes or vice versa
                i += 1

    def _expand_macro(self, s, i, args):
        # Expands a macro starting at index 'i' in 's'. If this macro resulted
        # from the expansion of another macro, 'args' holds the arguments
        # passed to that macro.
        #
        # Returns the expanded 's' (including the part before the macro) and
        # the index of the first character after the expanded macro in 's'.

        res = s[:i]
        i += 2  # Skip over "$("

        arg_start = i  # Start of current macro argument
        new_args = []  # Arguments of this macro call
        nesting = 0  # Current parentheses nesting level

        while 1:
            match = _macro_special_search(s, i)
            if not match:
                self._parse_error("missing end parenthesis in macro expansion")

            if match.group() == "(":
                nesting += 1
                i = match.end()

            elif match.group() == ")":
                if nesting:
                    nesting -= 1
                    i = match.end()
                    continue

                # Found the end of the macro

                new_args.append(s[arg_start : match.start()])

                # $(1) is replaced by the first argument to the function, etc.,
                # provided at least that many arguments were passed

                try:
                    # Does the macro look like an integer, with a corresponding
                    # argument? If so, expand it to the value of the argument.
                    res += args[int(new_args[0])]
                except (ValueError, IndexError):
                    # Regular variables are just functions without arguments,
                    # and also go through the function value path
                    res += self._fn_val(new_args)

                return (res + s[match.end() :], len(res))

            elif match.group() == ",":
                i = match.end()
                if nesting:
                    continue

                # Found the end of a macro argument
                new_args.append(s[arg_start : match.start()])
                arg_start = i

            else:  # match.group() == "$("
                # A nested macro call within the macro
                s, i = self._expand_macro(s, match.start(), args)

    def _fn_val(self, args):
        # Returns the result of calling the function args[0] with the arguments
        # args[1..len(args)-1]. Plain variables are treated as functions
        # without arguments.

        fn = args[0]

        if fn in self.variables:
            var = self.variables[fn]

            if len(args) == 1:
                # Plain variable
                if var._n_expansions:
                    self._parse_error(f"Preprocessor variable {var.name} recursively references itself")
            elif var._n_expansions > 100:
                # Allow functions to call themselves, but guess that functions
                # that are overly recursive are stuck
                self._parse_error(f"Preprocessor function {var.name} seems stuck in infinite recursion")

            var._n_expansions += 1
            res = self._expand_whole(self.variables[fn].value, args)
            var._n_expansions -= 1
            return res

        if fn in self._functions:
            # Built-in or user-defined function

            py_fn, min_arg, max_arg = self._functions[fn]

            if len(args) - 1 < min_arg or (max_arg is not None and len(args) - 1 > max_arg):
                if min_arg == max_arg:
                    expected_args = min_arg
                elif max_arg is None:
                    expected_args = f"{min_arg} or more"
                else:
                    expected_args = f"{min_arg}-{max_arg}"

                raise KconfigError(
                    f"{self.filename}:{self.linenr}: bad number of arguments in call to {fn}, "
                    f"expected {expected_args}, got {len(args) - 1}"
                )

            return py_fn(self, *args)

        # Environment variables are tried last
        if fn in os.environ:
            self.env_vars.add(fn)
            return os.environ[fn]

        return ""

    #
    # Parsing
    #

    def _make_and(self, e1, e2):
        # Constructs an AND (&&) expression. Performs trivial simplification.

        if e1 is self.y:
            return e2

        if e2 is self.y:
            return e1

        if e1 is self.n or e2 is self.n:
            return self.n

        return (AND, e1, e2)

    def _make_or(self, e1, e2):
        # Constructs an OR (||) expression. Performs trivial simplification.

        if e1 is self.n:
            return e2

        if e2 is self.n:
            return e1

        if e1 is self.y or e2 is self.y:
            return self.y

        return (OR, e1, e2)

    def _new_parse(self, end_token, parent, last_node):
        """
        Temporary function to parse with new Parser

        end_token: deprecated, unused
        parent: parent node - to which node to add the top node parsed here
                e.g.: parent = top_node -> to this node, we add (expected) MainMenu from file
        last_node: node that was parsed as a last one before calling this function
                   will be deprecated probably as the call logic is different from the original


        returns node that was parsed as a last one in this function.
                probably will be deprecated as the call logic is different from the original.
        """
        from kconfiglib.kconfig_parser import Parser

        parser = Parser(kconfig=self)

        parser.parse_all()

        last = self.top_node
        return last

    def _parse_block(self, end_token, parent, prev):
        # Parses a block, which is the contents of either a file or an if,
        # menu, or choice statement.
        #
        # end_token:
        #   The token that ends the block, e.g. _T_ENDIF ("endif") for ifs.
        #   None for files.
        #
        # parent:
        #   The parent menu node, corresponding to a menu, Choice, or 'if'.
        #   'if's are flattened after parsing.
        #
        # prev:
        #   The previous menu node. New nodes will be added after this one (by
        #   modifying 'next' pointers).
        #
        #   'prev' is reused to parse a list of child menu nodes (for a menu or
        #   Choice): After parsing the children, the 'next' pointer is assigned
        #   to the 'list' pointer to "tilt up" the children above the node.
        #
        # Returns the final menu node in the block (or 'prev' if the block is
        # empty). This allows chaining.

        while self._tokenize_line():
            t0 = self._tokens[0]

            if t0 == _T_CONFIG or t0 == _T_MENUCONFIG:
                # The tokenizer allocates Symbol objects for us
                sym = self._tokens[1]
                if sym.__class__ is not Symbol or sym.is_constant:
                    self._parse_error("missing or bad symbol name")

                if self._tokens[2] is not None:
                    self._trailing_tokens_error()

                self.defined_syms.append(sym)

                node = MenuNode(
                    kconfig=self,
                    item=sym,
                    is_menuconfig=t0 == _T_MENUCONFIG,
                    parent=parent,
                    filename=self.filename,
                    linenr=self.linenr,
                )
                node.include_path = self._include_path

                sym.nodes.append(node)

                self._parse_props(node)
                if node.is_menuconfig and not node.prompt:
                    self._warn(f"the menuconfig symbol {sym.name_and_loc} has no prompt")

                # Equivalent to
                #
                #   prev.next = node
                #   prev = node
                #
                # due to tricky Python semantics. The order matters.
                prev.next = prev = node

            elif t0 is None:
                # Blank line
                continue

            elif t0 in _SOURCE_TOKENS:
                pattern = self._expect_str_and_eol()
                if t0 in _REL_SOURCE_TOKENS:
                    # Relative source
                    pattern = join(dirname(self.filename), pattern)

                # - glob() doesn't support globbing relative to a directory, so
                #   we need to prepend $srctree to 'pattern'. Use join()
                #   instead of '+' so that an absolute path in 'pattern' is
                #   preserved.
                #
                # - Sort the glob results to ensure a consistent ordering of
                #   Kconfig symbols, which indirectly ensures a consistent
                #   ordering in e.g. .config files
                filenames = sorted(iglob(join(self._srctree_prefix, pattern)))

                if not filenames and t0 in _OBL_SOURCE_TOKENS:
                    raise KconfigError(
                        "{}:{}: '{}' not found (in '{}'). Check that "
                        "environment variables are set correctly (e.g. "
                        "$srctree, which is {}). Also note that unset "
                        "environment variables expand to the empty string.".format(
                            self.filename,
                            self.linenr,
                            pattern,
                            self._line.strip(),
                            f"set to '{self.srctree}'" if self.srctree else "unset or blank",
                        )
                    )

                for filename in filenames:
                    self._enter_file(filename)
                    prev = self._parse_block(None, parent, prev)
                    self._leave_file()

            elif t0 is end_token:
                # Reached the end of the block. Terminate the final node and
                # return it.

                if self._tokens[1] is not None:
                    self._trailing_tokens_error()

                prev.next = None
                return prev

            elif t0 == _T_IF:
                node = MenuNode(kconfig=self, parent=parent, dep=self._expect_expr_and_eol())

                self._parse_block(_T_ENDIF, node, node)
                node.list = node.next

                prev.next = prev = node

            elif t0 == _T_MENU:
                node = MenuNode(
                    kconfig=self,
                    item=t0,
                    is_menuconfig=True,
                    parent=parent,
                    prompt=(self._expect_str_and_eol(), self.y),
                    visibility=self.y,
                    filename=self.filename,
                    linenr=self.linenr,
                )
                node.include_path = self._include_path

                self.menus.append(node)

                self._parse_props(node)
                self._parse_block(_T_ENDMENU, node, node)
                node.list = node.next

                prev.next = prev = node

            elif t0 == _T_COMMENT:
                node = MenuNode(
                    kconfig=self,
                    item=t0,
                    is_menuconfig=False,
                    parent=parent,
                    prompt=(self._expect_str_and_eol(), self.y),
                    filename=self.filename,
                    linenr=self.linenr,
                )
                node.list = None
                node.include_path = self._include_path

                self.comments.append(node)

                self._parse_props(node)

                prev.next = node
                prev = node

            elif t0 == _T_CHOICE:
                if self._tokens[1] is None:
                    choice = Choice(kconfig=self, direct_dep=self.n)
                else:
                    # Named choice
                    name = self._expect_str_and_eol()
                    choice = self.named_choices.get(name)
                    if not choice:
                        choice = Choice(kconfig=self, name=name, direct_dep=self.n)
                        self.named_choices[name] = choice
                self.choices.append(choice)

                node = MenuNode(
                    kconfig=self,
                    item=choice,
                    is_menuconfig=True,
                    parent=parent,
                    filename=self.filename,
                    linenr=self.linenr,
                )
                choice.nodes.append(node)

                self._parse_props(node)
                self._parse_block(_T_ENDCHOICE, node, node)
                node.list = node.next

                prev.next = prev = node

            elif t0 == _T_MAINMENU:
                self.top_node.prompt = (self._expect_str_and_eol(), self.y)

            else:
                # A valid endchoice/endif/endmenu is caught by the 'end_token'
                # check above
                self._parse_error(
                    "no corresponding 'choice'"
                    if t0 == _T_ENDCHOICE
                    else "no corresponding 'if'"
                    if t0 == _T_ENDIF
                    else "no corresponding 'menu'"
                    if t0 == _T_ENDMENU
                    else "unrecognized construct"
                )

        # End of file reached. Return the last node.

        if end_token:
            raise KconfigError(
                "error: expected "
                f"{'endchoice' if end_token == _T_ENDCHOICE else 'endif' if end_token == _T_ENDIF else 'endmenu'} "
                f"at end of {self.filename}"
            )

        return prev

    def _parse_cond(self):
        # Parses an optional 'if <expr>' construct and returns the parsed
        # <expr>, or self.y if the next token is not _T_IF

        expr = self._parse_expr() if self._check_token(_T_IF) else self.y

        if self._tokens[self._tokens_i] is not None:
            self._trailing_tokens_error()

        return expr

    def _parse_props(self, node):
        # Parses and adds properties to the MenuNode 'node' (type, 'prompt',
        # 'default's, etc.) Properties are later copied up to symbols and
        # choices in a separate pass after parsing, in e.g.
        # _add_props_to_sym().
        #
        # An older version of this code added properties directly to symbols
        # and choices instead of to their menu nodes (and handled dependency
        # propagation simultaneously), but that loses information on where a
        # property is added when a symbol or choice is defined in multiple
        # locations. Some Kconfig configuration systems rely heavily on such
        # symbols, and better docs can be generated by keeping track of where
        # properties are added.
        #
        # node:
        #   The menu node we're parsing properties on

        # Dependencies from 'depends on'. Will get propagated to the properties
        # below.
        node.dep = self.y

        while self._tokenize_line():
            t0 = self._tokens[0]

            if t0 in _TYPE_TOKENS:
                # Relies on '_T_BOOL is BOOL', etc., to save a conversion
                self._set_type(node.item, t0)
                if self._tokens[1] is not None:
                    self._parse_prompt(node)

            elif t0 == _T_DEPENDS:
                if not self._check_token(_T_ON):
                    self._parse_error("expected 'on' after 'depends'")

                node.dep = self._make_and(node.dep, self._expect_expr_and_eol())

            elif t0 == _T_HELP:
                self._parse_help(node)

            elif t0 == _T_SELECT:
                if node.item.__class__ is not Symbol:
                    self._parse_error("only symbols can select")

                node.selects.append((self._expect_nonconst_sym(), self._parse_cond()))

            elif t0 is None:
                # Blank line
                continue

            elif t0 == _T_DEFAULT:
                node.defaults.append((self._parse_expr(), self._parse_cond()))

            elif t0 == _T_PROMPT:
                self._parse_prompt(node)

            elif t0 == _T_RANGE:
                node.ranges.append((self._expect_sym(), self._expect_sym(), self._parse_cond()))

            elif t0 == _T_IMPLY:
                if node.item.__class__ is not Symbol:
                    self._parse_error("only symbols can imply")

                node.implies.append((self._expect_nonconst_sym(), self._parse_cond()))

            elif t0 == _T_VISIBLE:
                if not self._check_token(_T_IF):
                    self._parse_error("expected 'if' after 'visible'")

                if node.item and node.item.__class__ is Symbol:
                    self._warn(
                        f'config {node.item.name} {_locs(node.item)} has a "visible if" option, '
                        "which is not supported for configs"
                    )

                elif node.item and node.item.__class__ is Choice:
                    self._warn(
                        f'choice {node.item.name} {_locs(node.item)} has a "visible if" option, '
                        "which is not supported for choices"
                    )
                else:
                    node.visibility = self._make_and(node.visibility, self._expect_expr_and_eol())

            elif t0 == _T_OPTION:
                if self._check_token(_T_ENV):
                    if not self._check_token(_T_EQUAL):
                        self._parse_error("expected '=' after 'env'")

                    env_var = self._expect_str_and_eol()
                    node.item.env_var = env_var

                    if env_var in os.environ:
                        node.defaults.append((self._lookup_const_sym(os.environ[env_var]), self.y))
                    else:
                        self._warn(
                            f"{node.item.name} has 'option env=\"{env_var}\"', "
                            f"but the environment variable {env_var} is not set",
                            self.filename,
                            self.linenr,
                        )

                    if env_var != node.item.name:
                        self._warn(
                            "Kconfiglib expands environment variables "
                            "in strings directly, meaning you do not "
                            "need 'option env=...' \"bounce\" symbols. "
                            "For compatibility with the C tools, "
                            f"rename {node.item.name} to {env_var} (so that the symbol name "
                            "matches the environment variable name).",
                            self.filename,
                            self.linenr,
                        )

                elif self._check_token(_T_DEFCONFIG_LIST):
                    if not self.defconfig_list:
                        self.defconfig_list = node.item
                    else:
                        self._warn(
                            "'option defconfig_list' set on multiple "
                            f"symbols ({self.defconfig_list.name} and {node.item.name}). "
                            f"Only {self.defconfig_list.name} will be used.",
                            self.filename,
                            self.linenr,
                        )

                elif self._check_token(_T_ALLNOCONFIG_Y):
                    if node.item.__class__ is not Symbol:
                        self._parse_error("the 'allnoconfig_y' option is only valid for symbols")

                    node.item.is_allnoconfig_y = True

                else:
                    self._parse_error("unrecognized option")

            elif t0 == _T_OPTIONAL:
                if node.item.__class__ is not Choice:
                    self._parse_error('"optional" is only valid for choices')

            else:
                # Reuse the tokens for the non-property line later
                self._reuse_tokens = True
                return

    def _set_type(self, symbol_or_choice, new_type):
        # Sets the type of 'sc' (symbol or choice) to 'new_type'

        # UNKNOWN is falsy
        if symbol_or_choice.orig_type and symbol_or_choice.orig_type != new_type:
            self._warn(
                f"{symbol_or_choice.name_and_loc} defined with multiple types, {TYPE_TO_STR[new_type]} will be used"
            )

        symbol_or_choice.orig_type = new_type

    def _parse_prompt(self, node):
        # 'prompt' properties override each other within a single definition of
        # a symbol, but additional prompts can be added by defining the symbol
        # multiple times

        if node.prompt:
            self._warn(node.item.name_and_loc + " defined with multiple prompts in single location")

        prompt = self._tokens[1]
        self._tokens_i = 2

        if prompt.__class__ is not str:
            self._parse_error("expected prompt string")

        if prompt != prompt.strip():
            self._warn(node.item.name_and_loc + " has leading or trailing whitespace in its prompt")

            # This avoid issues for e.g. reStructuredText documentation, where
            # '*prompt *' is invalid
            prompt = prompt.strip()

        node.prompt = (prompt, self._parse_cond())

    def _parse_help(self, node):
        if node.help is not None:
            self._warn(
                f"{node.item.name_and_loc} defined with more than one help text -- only the last one will be used"
            )

        # Micro-optimization. This code is pretty hot.
        readline = self._readline

        # Find first non-blank (not all-space) line and get its
        # indentation

        while 1:
            line = readline()
            self.linenr += 1
            if not line:
                self._empty_help(node, line)
                return
            if not line.isspace():
                break

        len_ = len  # Micro-optimization

        # Use a separate 'expline' variable here and below to avoid stomping on
        # any tabs people might've put deliberately into the first line after
        # the help text
        expline = line.expandtabs()
        indent = len_(expline) - len_(expline.lstrip())
        if not indent:
            self._empty_help(node, line)
            return

        # The help text goes on till the first non-blank line with less indent
        # than the first line

        # Add the first line
        lines = [expline[indent:]]
        add_line = lines.append  # Micro-optimization

        while 1:
            line = readline()
            if line.isspace():
                # No need to preserve the exact whitespace in these
                add_line("\n")
            elif not line:
                # End of file
                break
            else:
                expline = line.expandtabs()
                if len_(expline) - len_(expline.lstrip()) < indent:
                    break
                add_line(expline[indent:])

        self.linenr += len_(lines)
        node.help = "".join(lines).rstrip()
        if line:
            self._line_after_help(line)

    def _empty_help(self, node, line):
        self._warn(node.item.name_and_loc + " has 'help' but empty help text")
        node.help = ""
        if line:
            self._line_after_help(line)

    def _parse_expr(self):
        # Parses an expression from the tokens in Kconfig._tokens using a
        # simple top-down approach. See the module docstring for the expression
        # format.

        # Grammar:
        #
        #   expr:     and_expr ['||' expr]
        #   and_expr: factor ['&&' and_expr]
        #   factor:   <symbol> ['='/'!='/'<'/... <symbol>]
        #             '!' factor
        #             '(' expr ')'
        #
        # It helps to think of the 'expr: and_expr' case as a single-operand OR
        # (no ||), and of the 'and_expr: factor' case as a single-operand AND
        # (no &&). Parsing code is always a bit tricky.

        # Mind dump: parse_factor() and two nested loops for OR and AND would
        # work as well. The straightforward implementation there gives a
        # (op, (op, (op, A, B), C), D) parse for A op B op C op D. Representing
        # expressions as (op, [list of operands]) instead goes nicely with that
        # version, but is wasteful for short expressions and complicates
        # expression evaluation and other code that works on expressions (more
        # complicated code likely offsets any performance gain from less
        # recursion too). If we also try to optimize the list representation by
        # merging lists when possible (e.g. when ANDing two AND expressions),
        # we end up allocating a ton of lists instead of reusing expressions,
        # which is bad.

        and_expr = self._parse_and_expr()

        # Return 'and_expr' directly if we have a "single-operand" OR.
        # Otherwise, parse the expression on the right and make an OR node.
        # This turns A || B || C || D into (OR, A, (OR, B, (OR, C, D))).
        return and_expr if not self._check_token(_T_OR) else (OR, and_expr, self._parse_expr())

    def _parse_and_expr(self):
        factor = self._parse_factor()

        # Return 'factor' directly if we have a "single-operand" AND.
        # Otherwise, parse the right operand and make an AND node. This turns
        # A && B && C && D into (AND, A, (AND, B, (AND, C, D))).
        return factor if not self._check_token(_T_AND) else (AND, factor, self._parse_and_expr())

    def _parse_factor(self):
        token = self._tokens[self._tokens_i]
        self._tokens_i += 1

        if token.__class__ is Symbol:
            # Plain symbol or relation

            if self._tokens[self._tokens_i] not in _RELATIONS:
                # Plain symbol
                return token

            # Relation
            #
            # _T_EQUAL, _T_UNEQUAL, etc., deliberately have the same values as
            # EQUAL, UNEQUAL, etc., so we can just use the token directly
            self._tokens_i += 1
            return (self._tokens[self._tokens_i - 1], token, self._expect_sym())

        if token == _T_NOT:
            # token == _T_NOT == NOT
            return (token, self._parse_factor())

        if token == _T_OPEN_PAREN:
            expr_parse = self._parse_expr()
            if self._check_token(_T_CLOSE_PAREN):
                return expr_parse

        self._parse_error("malformed expression")

    #
    # Caching and invalidation
    #

    def _build_dep(self):
        # Populates the Symbol/Choice._dependents sets, which contain all other
        # items (symbols and choices) that immediately depend on the item in
        # the sense that changing the value of the item might affect the value
        # of the dependent items. This is used for caching/invalidation.
        #
        # The calculated sets might be larger than necessary as we don't do any
        # complex analysis of the expressions.

        depend_on = _depend_on  # Micro-optimization

        # Only calculate _dependents for defined symbols. Constant and
        # undefined symbols could theoretically be selected/implied, but it
        # wouldn't change their value, so it's not a true dependency.
        for sym in self.unique_defined_syms:
            # Symbols depend on the following:

            # The prompt conditions
            for node in sym.nodes:
                if node.prompt:
                    depend_on(sym, node.prompt[1])

            # The default values and their conditions
            for value, cond in sym.defaults:
                depend_on(sym, value)
                depend_on(sym, cond)

            # The reverse and weak reverse dependencies
            depend_on(sym, sym.rev_dep)
            depend_on(sym, sym.weak_rev_dep)

            # The ranges along with their conditions
            for low, high, cond in sym.ranges:
                depend_on(sym, low)
                depend_on(sym, high)
                depend_on(sym, cond)

            # The direct dependencies. This is usually redundant, as the direct
            # dependencies get propagated to properties, but it's needed to get
            # invalidation solid for 'imply', which only checks the direct
            # dependencies (even if there are no properties to propagate it
            # to).
            depend_on(sym, sym.direct_dep)

            # In addition to the above, choice symbols depend on the choice
            # they're in, but that's handled automatically since the Choice is
            # propagated to the conditions of the properties before
            # _build_dep() runs.

        for choice in self.unique_choices:
            # Choices depend on the following:

            # The prompt conditions
            for node in choice.nodes:
                if node.prompt:
                    depend_on(choice, node.prompt[1])

            # The default symbol conditions
            for _, cond in choice.defaults:
                depend_on(choice, cond)

    def _add_choice_deps(self):
        # Choices also depend on the choice symbols themselves, because the
        # y-mode selection of the choice might change if a choice symbol's
        # visibility changes.
        #
        # We add these dependencies separately after dependency loop detection.
        # The invalidation algorithm can handle the resulting
        # <choice symbol> <-> <choice> dependency loops, but they make loop
        # detection awkward.

        for choice in self.unique_choices:
            for sym in choice.syms:
                sym._dependents.add(choice)

    def _invalidate_all(self):
        # Undefined symbols never change value and don't need to be
        # invalidated, so we can just iterate over defined symbols.
        # Invalidating constant symbols would break things horribly.
        for sym in self.unique_defined_syms:
            sym._invalidate()

        for choice in self.unique_choices:
            choice._invalidate()

    #
    # Post-parsing menu tree processing, including dependency propagation and
    # implicit submenu creation
    #

    def _finalize_node(self, node, visible_if):
        # Finalizes a menu node and its children:
        #
        #  - Copies properties from menu nodes up to their contained
        #    symbols/choices
        #
        #  - Propagates dependencies from parent to child nodes
        #
        #  - Creates implicit menus
        #
        #  - Removes 'if' nodes
        #
        #  - Sets 'choice' types and registers choice symbols
        #
        # menu_finalize() in the C implementation is similar.
        #
        # node:
        #   The menu node to finalize. This node and its children will have
        #   been finalized when the function returns, and any implicit menus
        #   will have been created.
        #
        # visible_if:
        #   Dependencies from 'visible if' on parent menus. These are added to
        #   the prompts of symbols and choices.

        if node.item.__class__ is Symbol:
            # Copy defaults, ranges, selects, and implies to the Symbol
            self._add_props_to_sym(node)

            # Find any items that should go in an implicit menu rooted at the
            # symbol
            cur = node
            while cur.next and _auto_menu_dep(node, cur.next):
                # This makes implicit submenu creation work recursively, with
                # implicit menus inside implicit menus
                self._finalize_node(cur.next, visible_if)
                cur = cur.next
                cur.parent = node

            if cur is not node:
                # Found symbols that should go in an implicit submenu. Tilt
                # them up above us.
                node.list = node.next
                node.next = cur.next
                cur.next = None

        elif node.list:
            # The menu node is a choice, menu, or if. Finalize each child node.

            if node.item == MENU:
                visible_if = self._make_and(visible_if, node.visibility)

            # Propagate the menu node's dependencies to each child menu node.
            #
            # This needs to go before the recursive _finalize_node() call so
            # that implicit submenu creation can look ahead at dependencies.
            self._propagate_deps(node, visible_if)

            # Finalize the children
            cur = node.list
            while cur:
                self._finalize_node(cur, visible_if)
                cur = cur.next

        if node.list:
            # node's children have been individually finalized. Do final steps
            # to finalize this "level" in the menu tree.
            _flatten(node.list)
            _remove_ifs(node)

        # Empty choices (node.list None) are possible, so this needs to go
        # outside
        if node.item.__class__ is Choice:
            # Add the node's non-node-specific properties to the choice, like
            # _add_props_to_sym() does
            choice = node.item
            choice.direct_dep = self._make_or(choice.direct_dep, node.dep)
            choice.defaults += node.defaults

            _finalize_choice(node)

    def _propagate_deps(self, node, visible_if):
        # Propagates 'node's dependencies to its child menu nodes

        # If the parent node holds a Choice, we use the Choice itself as the
        # parent dependency. This makes sense as the value (mode) of the choice
        # limits the visibility of the contained choice symbols. The C
        # implementation works the same way.
        #
        # Due to the similar interface, Choice works as a drop-in replacement
        # for Symbol here.
        basedep = node.item if node.item.__class__ is Choice else node.dep

        cur = node.list
        while cur:
            dep = cur.dep = self._make_and(cur.dep, basedep)

            if cur.item.__class__ in _SYMBOL_CHOICE:
                # Propagate 'visible if' and dependencies to the prompt
                if cur.prompt:
                    cur.prompt = (
                        cur.prompt[0],
                        self._make_and(cur.prompt[1], self._make_and(visible_if, dep)),
                    )

                # Propagate dependencies to defaults
                if cur.defaults:
                    cur.defaults = [(default, self._make_and(cond, dep)) for default, cond in cur.defaults]

                # Propagate dependencies to ranges
                if cur.ranges:
                    cur.ranges = [(low, high, self._make_and(cond, dep)) for low, high, cond in cur.ranges]

                # Propagate dependencies to selects
                if cur.selects:
                    cur.selects = [(target, self._make_and(cond, dep)) for target, cond in cur.selects]

                # Propagate dependencies to implies
                if cur.implies:
                    cur.implies = [(target, self._make_and(cond, dep)) for target, cond in cur.implies]

            elif cur.prompt:  # Not a symbol/choice
                # Propagate dependencies to the prompt. 'visible if' is only
                # propagated to symbols/choices.
                cur.prompt = (cur.prompt[0], self._make_and(cur.prompt[1], dep))

            cur = cur.next

    def _add_props_to_sym(self, node):
        # Copies properties from the menu node 'node' up to its contained
        # symbol, and adds (weak) reverse dependencies to selected/implied
        # symbols.
        #
        # This can't be rolled into _propagate_deps(), because that function
        # traverses the menu tree roughly breadth-first, meaning properties on
        # symbols defined in multiple locations could end up in the wrong
        # order.

        sym = node.item

        sym.direct_dep = self._make_or(sym.direct_dep, node.dep)

        sym.defaults += node.defaults
        sym.ranges += node.ranges
        sym.selects += node.selects
        sym.implies += node.implies

        # Modify the reverse dependencies of the selected symbol
        for target, cond in node.selects:
            target.rev_dep = self._make_or(target.rev_dep, self._make_and(sym, cond))

        # Modify the weak reverse dependencies of the implied
        # symbol
        for target, cond in node.implies:
            target.weak_rev_dep = self._make_or(target.weak_rev_dep, self._make_and(sym, cond))

    #
    # Misc.
    #

    def _check_multiple_definitions(self):
        """
        Checks for multiple definitions of symbols and choices. If such a symbol or choice is found,
        warning is generated.
        NOTE: One Kconfig file can be sourced in multiple files, this case will manifest as several
        nodes, but with the same filename and line number. This situation is not ideal, but allowable.
        """
        for sym in self.unique_defined_syms:
            if len(sym.nodes) > 1:
                occurrences = set(f"    {os.path.abspath(node.filename)}:{node.linenr}" for node in sym.nodes)
                if len(occurrences) > 1 and sym.name not in self.allowed_multi_def_syms:
                    occurrences = "\n".join(occurrences)
                    self._info(
                        f"INFO: Symbol {sym.name} defined in multiple locations (see below). "
                        f"Please check if this is a correct behavior or a random name match:\n{occurrences}"
                    )

        for choice in self.unique_choices:
            if len(choice.nodes) > 1 and choice.name not in self.allowed_multi_def_choices:
                occurrences = set(f"    {os.path.abspath(node.filename)}:{node.linenr}" for node in choice.nodes)
                if len(occurrences) > 1:
                    occurrences = "\n".join(occurrences)
                    self._info(
                        f"INFO: Choice {choice.name} defined in multiple locations (see below)."
                        f" Please check if this is a correct behavior or a random name match:\n{occurrences}"
                    )

    def _check_sym_sanity(self):
        # Checks various symbol properties that are handiest to check after
        # parsing. Only generates errors and warnings.

        def num_ok(sym, type_):
            # Returns True if the (possibly constant) symbol 'sym' is valid as a value
            # for a symbol of type type_ (INT or HEX)

            # 'not sym.nodes' implies a constant or undefined symbol, e.g. a plain
            # "123"
            if not sym.nodes:
                return _is_base_n(sym.name, _TYPE_TO_BASE[type_])

            return sym.orig_type == type_

        for sym in self.unique_defined_syms:
            if sym.orig_type == BOOL:
                # A helper function could be factored out here, but keep it
                # speedy/straightforward

                for target_sym, _ in sym.selects:
                    if target_sym.orig_type not in _BOOL_UNKNOWN:
                        self._warn(
                            f"{sym.name_and_loc} selects the {TYPE_TO_STR[target_sym.orig_type]} symbol "
                            f"{target_sym.name_and_loc}, which is not bool"
                        )

                for target_sym, _ in sym.implies:
                    if target_sym.orig_type not in _BOOL_UNKNOWN:
                        self._warn(
                            f"{sym.name_and_loc} implies the {TYPE_TO_STR[target_sym.orig_type]} symbol "
                            f"{target_sym.name_and_loc}, which is not bool"
                        )

            elif sym.orig_type:  # STRING/INT/HEX
                for default, _ in sym.defaults:
                    if default.__class__ is not Symbol:
                        raise KconfigError(
                            f"the {TYPE_TO_STR[sym.orig_type]} symbol {sym.name_and_loc} has a malformed default "
                            f"{expr_str(default)} -- expected a single symbol"
                        )

                    if sym.orig_type == STRING:
                        if not default.is_constant and not default.nodes and not default.name.isupper():
                            # 'default foo' on a string symbol could be either a symbol
                            # reference or someone leaving out the quotes. Guess that
                            # the quotes were left out if 'foo' isn't all-uppercase
                            # (and no symbol named 'foo' exists).
                            self._warn(
                                f"style: quotes recommended around default value for string symbol {sym.name_and_loc}"
                            )

                    elif not num_ok(default, sym.orig_type):  # INT/HEX
                        self._warn(
                            f"the {TYPE_TO_STR[sym.orig_type]} symbol {sym.name_and_loc} has "
                            f"a non-{TYPE_TO_STR[sym.orig_type]} default {default.name_and_loc}"
                        )

                if sym.selects or sym.implies:
                    self._warn(f"the {TYPE_TO_STR[sym.orig_type]} symbol {sym.name_and_loc} has selects or implies")

            else:  # UNKNOWN
                self._warn(f"{sym.name_and_loc} defined without a type")

            if sym.ranges:
                if sym.orig_type not in _INT_HEX:
                    self._warn(
                        f"the {TYPE_TO_STR[sym.orig_type]} symbol {sym.name_and_loc} has ranges, but is not int or hex"
                    )
                else:
                    for low, high, _ in sym.ranges:
                        if not num_ok(low, sym.orig_type) or not num_ok(high, sym.orig_type):
                            self._warn(
                                f"the {TYPE_TO_STR[sym.orig_type]} symbol {sym.name_and_loc} has "
                                f"a non-{TYPE_TO_STR[sym.orig_type]} range [{low.name_and_loc}, {high.name_and_loc}]"
                            )

    def _check_choice_sanity(self):
        # Checks various choice properties that are handiest to check after
        # parsing. Only generates errors and warnings.

        def warn_select_imply(sym, expr, expr_type):
            msg = (
                f"the choice symbol {sym.name_and_loc} is {expr_type} by the following symbols, "
                "but select/imply has no effect on choice symbols"
            )

            # si = select/imply
            for si in split_expr(expr, OR):
                msg += "\n - " + split_expr(si, AND)[0].name_and_loc

            self._warn(msg)

        for choice in self.unique_choices:
            if choice.orig_type != BOOL:
                self._warn(f"{choice.name_and_loc} defined with type {TYPE_TO_STR[choice.orig_type]}")

            for node in choice.nodes:
                if node.prompt:
                    break
            else:
                self._warn(choice.name_and_loc + " defined without a prompt")

            for default, _ in choice.defaults:
                if default.__class__ is not Symbol:
                    raise KconfigError(f"{choice.name_and_loc} has a malformed default {expr_str(default)}")

                if default.choice is not choice:
                    self._warn(
                        f"the default selection {default.name_and_loc} of {choice.name_and_loc} "
                        "is not contained in the choice"
                    )

            for sym in choice.syms:
                if sym.defaults:
                    self._warn(
                        f"default on the choice symbol {sym.name_and_loc} will have "
                        "no effect, as defaults do not affect choice "
                        "symbols"
                    )

                if sym.rev_dep is not sym.kconfig.n:
                    warn_select_imply(sym, sym.rev_dep, "selected")

                if sym.weak_rev_dep is not sym.kconfig.n:
                    warn_select_imply(sym, sym.weak_rev_dep, "implied")

                for node in sym.nodes:
                    if node.parent.item is choice:
                        if not node.prompt:
                            self._warn(f"the choice symbol {sym.name_and_loc} has no prompt")

                    elif node.prompt:
                        self._warn(f"the choice symbol {sym.name_and_loc} is defined with a prompt outside the choice")

    def _parse_error(self, msg):
        raise KconfigError(
            "{}error: couldn't parse '{}': {}".format(
                "" if self.filename is None else f"{self.filename}:{self.linenr}: ",
                self._line.strip(),
                msg,
            )
        )

    def _trailing_tokens_error(self):
        self._parse_error("extra tokens at end of line")

    def _check_undef_syms(self):
        # Prints warnings for all references to undefined symbols within the
        # Kconfig files

        def is_num(s):
            # Returns True if the string 's' looks like a number.
            #
            # Internally, all operands in Kconfig are symbols, only undefined symbols
            # (which numbers usually are) get their name as their value.
            #
            # Only hex numbers that start with 0x/0X are classified as numbers.
            # Otherwise, symbols whose names happen to contain only the letters A-F
            # would trigger false positives.

            try:
                int(s)
            except ValueError:
                if not s.startswith(("0x", "0X")):
                    return False

                try:
                    int(s, 16)
                except ValueError:
                    return False

            return True

        for sym in self.syms.values():
            # - sym.nodes empty means the symbol is undefined (has no
            #   definition locations)
            #
            # - Due to Kconfig internals, numbers show up as undefined Kconfig
            #   symbols, but shouldn't be flagged
            if not sym.nodes and not is_num(sym.name):
                msg = f"undefined symbol {sym.name}:"
                for node in self.node_iter():
                    if sym in node.referenced:
                        msg += f"\n\n- Referenced at {node.filename}:{node.linenr}:\n\n{node}"
                self._warn(msg)

    def _warn(self, msg, filename=None, linenr=None):
        # For printing general warnings

        if not self.warn:
            return

        msg = "warning: " + msg
        if filename is not None:
            msg = f"{filename}:{linenr}: {msg}"

        self.warnings.append(msg)
        if self.warn_to_stderr:
            sys.stderr.write(msg + "\n")

    def _info(self, msg):
        if not self.info:
            return

        sys.stderr.write(f"info: {msg}\n")


class Symbol:
    """
    Represents a configuration symbol:

      (menu)config FOO
          ...

    Note: Prompts, help texts, and locations are stored in the Symbol's
    MenuNode(s) rather than in the Symbol itself. Check the MenuNode class and
    the Symbol.nodes attribute. This organization matches the C tools.
    """

    __slots__ = (
        "_cached_assignable",
        "_cached_bool_val",
        "_cached_str_val",
        "_cached_vis",
        "_dependents",
        "_old_val",
        "_visited",
        "_was_set",
        "_write_to_conf",
        "choice",
        "defaults",
        "direct_dep",
        "implies",
        "is_allnoconfig_y",
        "is_constant",
        "kconfig",
        "name",
        "nodes",
        "orig_type",
        "ranges",
        "rev_dep",
        "selects",
        "_user_value",
        "weak_rev_dep",
        "env_var",
    )
    name: str
    item: Union["Symbol", "Choice", int]
    parent: Optional["MenuNode"]
    next: Optional["MenuNode"]
    prompt: Optional[Tuple]
    dep: Optional[Union[Tuple, "Symbol"]]
    kconfig: "Kconfig"
    is_menuconfig: bool
    help: Optional[str]
    is_constant: bool
    env_var: Optional[str]
    ranges: List[Tuple]
    _cached_bool_val: Optional[int]
    choice: Optional["Choice"]

    #
    # Public interface
    #
    def __init__(self, kconfig: Kconfig, name: str, is_constant: bool = False, init_rest=True):
        """
        Symbol constructor -- not intended to be called directly by Kconfiglib
        clients.
        """

        self.kconfig = kconfig
        self.name = name

        """
        is_constant:
            True if the symbol is a constant (quoted) symbol.
        """
        self.is_constant = is_constant

        if init_rest:
            self.init_rest()

    def init_rest(self):
        """
        Because kconfig.y and kconfig.n are symbols as well, we can't initialize many of the attributes
        in the constructor.
        This method is called after the constructor to initialize the rest of the attributes.
        """

        """
        direct_dep:
            The direct ('depends on') dependencies for the symbol, or self.kconfig.y
            if there are no direct dependencies.

            This attribute includes any dependencies from surrounding menus and ifs.
            Those get propagated to the direct dependencies, and the resulting direct
            dependencies in turn get propagated to the conditions of all properties.

            If the symbol is defined in multiple locations, the dependencies from the
            different locations get ORed together.
        """
        self.direct_dep = self.kconfig.n

        """
        rev_dep:
            Reverse dependency expression from other symbols selecting this symbol.
            Multiple selections get ORed together. A condition on a select is ANDed
            with the selecting symbol.

            For example, if A has 'select FOO' and B has 'select FOO if C', then
            FOO's rev_dep will be (OR, A, (AND, B, C)).
        """
        self.rev_dep = self.kconfig.n

        """
        weak_rev_dep:
            Like rev_dep, but for imply.
        """
        self.weak_rev_dep = self.kconfig.n

        """
        orig_type:
            The type as given in the Kconfig file, without any changes applied. Used
            when printing the symbol.
        """
        self.orig_type = UNKNOWN

        """
        env_var:
            If the Symbol has an 'option env="FOO"' option, this contains the name
            ("FOO") of the environment variable. None for symbols without no
            'option env'.

            'option env="FOO"' acts like a 'default' property whose value is the
            value of $FOO.

            NOTE: Symbols with 'option env' are never written out to .config files, even if
            they are visible. env_var corresponds to a flag called SYMBOL_AUTO in the
            C implementation.
        """
        self.env_var = None

        """
        nodes:
            A list of MenuNodes for this symbol. Will contain a single MenuNode for
            most symbols. Undefined and constant symbols have an empty nodes list.
            Symbols defined in multiple locations get one node for each location.
        """
        self.nodes = []

        """
        defaults:
            List of (default, cond) tuples for the symbol's 'default' properties. For
            example, 'default A && B if C || D' is represented as
            ((AND, A, B), (OR, C, D)). If no condition was given, 'cond' is
            self.kconfig.y.

            Note that 'depends on' and parent dependencies are propagated to
            'default' conditions.
        """
        self.defaults = []

        """
        selects:
            List of (symbol, cond) tuples for the symbol's 'select' properties. For
            example, 'select A if B && C' is represented as (A, (AND, B, C)). If no
            condition was given, 'cond' is self.kconfig.y.

            Note that 'depends on' and parent dependencies are propagated to 'select'
            conditions.
        """
        self.selects = []

        """
        implies:
            Same as 'selects', but for 'imply' properties.
        """
        self.implies = []

        """
        ranges:
            List of (low, high, cond) tuples for the symbol's 'range' properties. For
            example, 'range 0 10 if A' is represented as (0, 10, A). If there is no
            condition, 'cond' is self.kconfig.y.

            Note that 'depends on' and parent dependencies are propagated to 'range'
            conditions.

            Gotcha: 0 and 10 above will be represented as (undefined) Symbols rather
            than plain integers. Undefined symbols get their name as their string
            value, so this works out. The C tools work the same way.
        """
        self.ranges = []

        """
        choice:
            Holds the parent Choice for choice symbols, and None for non-choice
            symbols. Doubles as a flag for whether a symbol is a choice symbol.
        """
        self.choice = None

        """
        _user_value:
            The user value of the symbol. None if no user value has been assigned
            (via Kconfig.load_config() or Symbol.set_value()).

            Holds 0 or 2 for bool symbols, and a string for the other
            symbol types.

            WARNING: Do not assign directly to this. It will break things. Use
            Symbol.set_value().
        """
        self._user_value = None

        # Internal attributes
        self._cached_str_val = None
        self._cached_bool_val = None
        self._cached_vis = None
        self._cached_assignable = None
        # - _visited is used during tree iteration and dep. loop detection
        self._visited = UNKNOWN

        # _write_to_conf is calculated along with the value. If True, the
        # Symbol gets a .config entry.
        self._write_to_conf = False

        """
        is_allnoconfig_y:
        True if the symbol has 'option allnoconfig_y' set on it. This has no
        effect internally (except when printing symbols), but can be checked by
        scripts.
        TODO: Do we use this?
        """
        self.is_allnoconfig_y = False
        self._was_set = False

        # See Kconfig._build_dep()
        self._dependents = set()

    @property
    def type(self):
        """
        The type of the symbol. One of BOOL, STRING, INT, HEX, UNKNOWN.
        UNKNOWN is for undefined symbols, (non-special) constant symbols, and
        symbols defined without a type.
        """
        return self.orig_type

    @property
    def str_value(self):
        """
        The value of the symbol as a string. Gives the value for string/int/hex
        symbols. For bool symbols, gives "n" or "y".

        This is the symbol value that's used in relational expressions
        (A = B, A != B, etc.)

        Gotcha: For int/hex symbols, the exact format of the value is often
        preserved (e.g. when writing a .config file), hence why you can't get it
        directly as an int. Do int(int_sym.str_value) or
        int(hex_sym.str_value, 16) to get the integer value.
        """
        if self._cached_str_val is not None:
            return self._cached_str_val

        if self.orig_type == BOOL:
            # Also calculates the visibility, so invalidation safe
            self._cached_str_val = BOOL_TO_STR[self.bool_value]
            return self._cached_str_val

        # As a quirk of Kconfig, undefined symbols get their name as their
        # string value. This is why things like "FOO = bar" work for seeing if
        # FOO has the value "bar".
        if not self.orig_type:  # UNKNOWN
            self._cached_str_val = self.name
            return self.name

        val = ""
        # Warning: TODO See Symbol._rec_invalidate(), and note that this is a hidden
        # function call (property magic)
        vis = self.visibility

        self._write_to_conf = vis != 0

        if self.orig_type in _INT_HEX:
            # The C implementation checks the user value against the range in a
            # separate code path (post-processing after loading a .config).
            # Checking all values here instead makes more sense for us. It
            # requires that we check for a range first.

            base = _TYPE_TO_BASE[self.orig_type]

            # Check if a range is in effect
            for low_expr, high_expr, cond in self.ranges:
                if expr_value(cond):
                    has_active_range = True

                    # The zeros are from the C implementation running strtoll()
                    # on empty strings
                    low = int(low_expr.str_value, base) if _is_base_n(low_expr.str_value, base) else 0
                    high = int(high_expr.str_value, base) if _is_base_n(high_expr.str_value, base) else 0

                    break
            else:
                has_active_range = False

            # Defaults are used if the symbol is invisible, lacks a user value,
            # or has an out-of-range user value
            use_defaults = True

            if vis and self._user_value:
                user_val = int(self._user_value, base)
                if has_active_range and not low <= user_val <= high:
                    num2str = str if base == 10 else hex
                    self.kconfig._warn(
                        f"user value {num2str(user_val)} on the {TYPE_TO_STR[self.orig_type]} symbol "
                        f"{self.name_and_loc} ignored due to being outside the active range "
                        f"([{num2str(low)}, {num2str(high)}]) -- falling back on defaults"
                    )
                else:
                    # If the user value is well-formed and satisfies range
                    # constraints, it is stored in exactly the same form as
                    # specified in the assignment (with or without "0x", etc.)
                    val = self._user_value
                    use_defaults = False

            if use_defaults:
                # No user value or invalid user value. Look at defaults.

                # Used to implement the warning below
                has_default = False

                for sym, cond in self.defaults:
                    if expr_value(cond):
                        has_default = self._write_to_conf = True

                        val = sym.str_value

                        if _is_base_n(val, base):
                            val_num = int(val, base)
                        else:
                            val_num = 0  # strtoll() on empty string

                        break
                else:
                    val_num = 0  # strtoll() on empty string

                # This clamping procedure runs even if there's no default
                if has_active_range:
                    clamp = None
                    if val_num < low:
                        clamp = low
                    elif val_num > high:
                        clamp = high

                    if clamp is not None:
                        # The value is rewritten to a standard form if it is
                        # clamped
                        val = str(clamp) if self.orig_type == INT else hex(clamp)

                        if has_default:
                            num2str = str if base == 10 else hex
                            self.kconfig._warn(
                                f"default value {val_num} on {self.name_and_loc} clamped to {num2str(clamp)} due to "
                                f"being outside the active range ([{num2str(low)}, {num2str(high)}])"
                            )

        elif self.orig_type == STRING:
            if vis and self._user_value is not None:
                # If the symbol is visible and has a user value, use that
                val = self._user_value
            else:
                # Otherwise, look at defaults
                for sym, cond in self.defaults:
                    if expr_value(cond):
                        val = sym.str_value
                        self._write_to_conf = True
                        break

        # env_var corresponds to SYMBOL_AUTO in the C implementation, and is
        # also set on the defconfig_list symbol there. Test for the
        # defconfig_list symbol explicitly instead here, to avoid a nonsensical
        # env_var setting and the defconfig_list symbol being printed
        # incorrectly. This code is pretty cold anyway.
        if self.env_var is not None or self is self.kconfig.defconfig_list:
            self._write_to_conf = False

        self._cached_str_val = val
        return val

    @property
    def bool_value(self) -> int:
        """
        See the class documentation.
        """
        if self._cached_bool_val is not None:
            return self._cached_bool_val

        if self.orig_type != BOOL:
            # always n for non-bool symbols
            if self.orig_type:  # != UNKNOWN
                # Would take some work to give the location here
                self.kconfig._warn(
                    f"The {TYPE_TO_STR[self.orig_type]} symbol {self.name_and_loc} is being evaluated in a logical "
                    "context somewhere. It will always evaluate to n."
                )

            self._cached_bool_val = 0
            return 0

        # Warning: See Symbol._rec_invalidate(), and note that this is a hidden
        # function call (property magic)
        vis = self.visibility
        self._write_to_conf = vis != 0

        val = 0

        if not self.choice:
            # Non-choice symbol

            if vis and self._user_value is not None:
                # If the symbol is visible and has a user value, use that
                val = min(self._user_value, vis)

            else:
                # Otherwise, look at defaults and weak reverse dependencies
                # (implies)

                for default, cond in self.defaults:
                    dep_val = expr_value(cond)
                    if dep_val:
                        val = min(expr_value(default), dep_val)
                        if val:
                            self._write_to_conf = True
                        break

                # Weak reverse dependencies are only considered if our
                # direct dependencies are met
                dep_val = expr_value(self.weak_rev_dep)
                if dep_val and expr_value(self.direct_dep):
                    val = max(dep_val, val)
                    self._write_to_conf = True

            # Reverse (select-related) dependencies take precedence
            dep_val = expr_value(self.rev_dep)
            if dep_val:
                if expr_value(self.direct_dep) < dep_val:
                    self._warn_select_unsatisfied_deps()

                val = max(dep_val, val)
                self._write_to_conf = True

            if val == 1:
                raise ValueError(
                    f"Value of symbol {self.name} of type {TYPE_TO_STR[self.type]} is 1,"
                    " which is not a valid bool value in kconfiglib (choose 0 for n or 2 for y)."
                )

        elif vis == 2:
            # Visible choice symbol in y-mode choice. The choice mode limits
            # the visibility of choice symbols, so it's sufficient to just
            # check the visibility of the choice symbols themselves.
            val = 2 if self.choice.selection is self else 0

        elif vis and self._user_value:
            val = 2

        self._cached_bool_val = val
        return val

    @property
    def assignable(self):
        """
        A tuple containing the bool user values that can currently be
        assigned to the symbol (that would be respected), ordered from lowest (0,
        representing n) to highest (2, representing y). This corresponds to the
        selections available in the menuconfig interface. The set of assignable
        values is calculated from the symbol's visibility and selects/implies.

        Returns the empty set for non-bool symbols and for symbols with
        visibility n. The other possible values are (0, 2) and (2,). A (2,) result means
        the symbol is visible but "locked" to y through a select, perhaps in combination with the
        visibility. menuconfig represents this as and -*-.

        For string/hex/int symbols, check if Symbol.visibility is non-0 (non-n)
        instead to determine if the value can be changed.

        Some handy 'assignable' idioms:

            # Is 'sym' an assignable (visible) bool symbol?
            if sym.assignable:
                # What's the highest value it can be assigned? [-1] in Python
                # gives the last element.
                sym_high = sym.assignable[-1]

                # The lowest?
                sym_low = sym.assignable[0]

                # Can the symbol be set to y?
                if sym.assignable[-1] == 2:
                    ...

        """
        if self._cached_assignable is None:
            self._cached_assignable = self._assignable()
        return self._cached_assignable

    @property
    def visibility(self):
        """
        The visibility of the symbol. One of 0, 2, representing n, y. See
        the module documentation for an overview of symbol values and visibility.
        """
        if self._cached_vis is None:
            self._cached_vis = _visibility(self)
        return self._cached_vis

    @property
    def config_string(self):
        """
        The .config assignment string that would get written out for the symbol
        by Kconfig.write_config(). Returns the empty string if no .config
        assignment would get written out.

        In general, visible symbols, symbols with (active) defaults, and selected
        symbols get written out. This includes all non-n-valued bool
        symbols, and all visible string/int/hex symbols.

        Symbols with the (no longer needed) 'option env=...' option generate no
        configuration output, and neither does the special
        'option defconfig_list' symbol.

        Tip: This field is useful when generating custom configuration output,
        even for non-.config-like formats. To write just the symbols that would
        get written out to .config files, do this:

            if sym.config_string:
                *Write symbol, e.g. by looking sym.str_value*

        This is a superset of the symbols written out by write_autoconf().
        That function skips all n-valued symbols.

        There usually won't be any great harm in just writing all symbols either,
        though you might get some special symbols and possibly some "redundant"
        n-valued symbol entries in there.
        """
        # _write_to_conf is determined when the value is calculated. This is a
        # hidden function call due to property magic.
        val = self.str_value
        if not self._write_to_conf:
            return ""

        if self.orig_type == BOOL:
            return (
                f"{self.kconfig.config_prefix}{self.name}={val}\n"
                if val != "n"
                else f"# {self.kconfig.config_prefix}{self.name} is not set\n"
            )

        if self.orig_type in _INT_HEX:
            return f"{self.kconfig.config_prefix}{self.name}={val}\n"

        # sym.orig_type == STRING
        return f'{self.kconfig.config_prefix}{self.name}="{escape(val)}"\n'

    @property
    def name_and_loc(self):
        """
        Holds a string like

        "MY_SYMBOL (defined at foo/Kconfig:12, bar/Kconfig:14)"

        , giving the name of the symbol and its definition location(s).
        If the symbol is undefined, the location is given as "(undefined)".
        """
        return self.name + " " + _locs(self)

    def set_value(self, value):
        """
        Sets the user value of the symbol.

        Equal in effect to assigning the value to the symbol within a .config
        file. For bool symbols, use the 'assignable' attribute to
        check which values can currently be assigned. Setting values outside
        'assignable' will cause Symbol._user_value to differ from
        Symbol.str/bool_value (be truncated down or up).

        Setting a choice symbol to 2 (y) sets Choice._user_selection to the
        choice symbol in addition to setting Symbol._user_value.
        Choice._user_selection is considered when the choice is in y mode (the
        "normal" mode).

        Other symbols that depend (possibly indirectly) on this symbol are
        automatically recalculated to reflect the assigned value.

        value:
          The user value to give to the symbol. For bool symbols,
          n/y can be specified either as 0/2 (the usual format for
          values in Kconfiglib) or as one of the strings "n", "y". For
          other symbol types, pass a string.

          Note that the value for an int/hex symbol is passed as a string, e.g.
          "123" or "0x0123". The format of this string is preserved in the
          output.

          Values that are invalid for the type (such as "foo" for a
          BOOL or "0x123" for an INT) are ignored and won't be stored in
          Symbol._user_value. Kconfiglib will print a warning by default for
          invalid assignments, and set_value() will return False.

        Returns True if the value is valid for the type of the symbol, and
        False otherwise. This only looks at the form of the value. For BOOL
        symbols, check the Symbol.assignable attribute to see what values are
        currently in range and would actually be reflected in the value
        of the symbol. For other symbol types, check whether the visibility is non-n.
        """
        if self.orig_type == BOOL and value in STR_TO_BOOL:
            value = STR_TO_BOOL[value]

        # If the new user value matches the old, nothing changes, and we can
        # avoid invalidating cached values.
        #
        # This optimization is skipped for choice symbols: Setting a choice
        # symbol's user value to y might change the state of the choice, so it
        # wouldn't be safe (symbol user values always match the values set in a
        # .config file or via set_value(), and are never implicitly updated).
        if value == self._user_value and not self.choice:
            self._was_set = True
            return True

        # Check if the value is valid for our type
        if not (
            self.orig_type == BOOL
            and value in (2, 0)
            or value.__class__ is str
            and (
                self.orig_type == STRING
                or self.orig_type == INT
                and _is_base_n(value, 10)
                or self.orig_type == HEX
                and _is_base_n(value, 16)
                and int(value, 16) >= 0
            )
        ):
            # Display bool values as n, y in the warning
            self.kconfig._warn(
                "the value {} is invalid for {}, which has type {} -- assignment ignored".format(
                    BOOL_TO_STR[value] if value in BOOL_TO_STR else f"'{value}'",
                    self.name_and_loc,
                    TYPE_TO_STR[self.orig_type],
                )
            )

            return False

        self._user_value = value
        self._was_set = True

        if self.choice and value == 2:
            # Setting a choice symbol to y makes it the user selection of the
            # choice. Like for symbol user values, the user selection is not
            # guaranteed to match the actual selection of the choice, as
            # dependencies come into play.
            self.choice._user_selection = self
            self.choice._was_set = True
            self.choice._rec_invalidate()

        else:
            self._rec_invalidate_if_has_prompt()

        return True

    def unset_value(self):
        """
        Removes any user value from the symbol, as if the symbol had never
        gotten a user value via Kconfig.load_config() or Symbol.set_value().
        """
        if self._user_value is not None:
            self._user_value = None
            self._rec_invalidate_if_has_prompt()

    @property
    def referenced(self):
        """
        A set() with all symbols and choices referenced in the properties and
        property conditions of the symbol.

        Also includes dependencies from surrounding menus and ifs, because those
        get propagated to the symbol (see the 'Intro to symbol values' section in
        the module docstring).

        Choices appear in the dependencies of choice symbols.

        For the following definitions, only B and not C appears in A's
        'referenced'. To get transitive references, you'll have to recursively
        expand 'references' until no new items appear.

            config A
                    bool
                    depends on B

            config B
                    bool
                    depends on C

            config C
                    bool

        See the Symbol.direct_dep attribute if you're only interested in the
        direct dependencies of the symbol (its 'depends on'). You can extract the
        symbols in it with the global expr_items() function.
        """
        return {item for node in self.nodes for item in node.referenced}

    @property
    def orig_defaults(self):
        """
        Returns a list of all orig_defaults from all menunodes this symbol is part of.
        See Menunode orig_* for more information.
        """
        return [d for node in self.nodes for d in node.orig_defaults]

    @property
    def orig_selects(self):
        """
        Returns a list of all orig_selects from all menunodes this symbol is part of.
        See Menunode orig_* for more information.
        """
        return [s for node in self.nodes for s in node.orig_selects]

    @property
    def orig_implies(self):
        """
        Returns a list of all orig_implies from all menunodes this symbol is part of.
        See Menunode orig_* for more information.
        """
        return [i for node in self.nodes for i in node.orig_implies]

    @property
    def orig_ranges(self):
        """
        Returns a list of all orig_ranges from all menunodes this symbol is part of.
        See Menunode orig_* for more information.
        """
        return [r for node in self.nodes for r in node.orig_ranges]

    def __repr__(self):
        """
        Returns a string with information about the symbol (including its name,
        value, visibility, and location(s)) when it is evaluated on e.g. the
        interactive Python prompt.
        """
        fields = ["symbol " + self.name, TYPE_TO_STR[self.type]]
        add = fields.append

        for node in self.nodes:
            if node.prompt:
                add(f'"{node.prompt[0]}"')

        # Only add quotes for non-bool symbols
        add("value " + (self.str_value if self.orig_type == BOOL else f'"{self.str_value}"'))

        if not self.is_constant:
            # These aren't helpful to show for constant symbols

            if self._user_value is not None:
                # Only add quotes for non-bool symbols
                add(
                    "user value "
                    + (BOOL_TO_STR[self._user_value] if self.orig_type == BOOL else f'"{self._user_value}"')
                )

            add("visibility " + BOOL_TO_STR[self.visibility])

            if self.choice:
                add("choice symbol")

            if self.is_allnoconfig_y:
                add("allnoconfig_y")

            if self is self.kconfig.defconfig_list:
                add("is the defconfig_list symbol")

            if self.env_var is not None:
                add("from environment variable " + self.env_var)

            add("direct deps " + BOOL_TO_STR[expr_value(self.direct_dep)])

        if self.nodes:
            for node in self.nodes:
                add(f"{node.filename}:{node.linenr}")
        else:
            add("constant" if self.is_constant else "undefined")

        return f"<{', '.join(fields)}>"

    def __str__(self):
        """
        Returns a string representation of the symbol when it is printed.
        Matches the Kconfig format, with any parent dependencies propagated to
        the 'depends on' condition.

        The string is constructed by joining the strings returned by
        MenuNode.__str__() for each of the symbol's menu nodes, so symbols
        defined in multiple locations will return a string with all
        definitions.

        The returned string does not end in a newline. An empty string is
        returned for undefined and constant symbols.
        """
        return self.custom_str(standard_sc_expr_str)

    def custom_str(self, sc_expr_str_fn):
        """
        Works like Symbol.__str__(), but allows a custom format to be used for
        all symbol/choice references. See expr_str().
        """
        return "\n\n".join(node.custom_str(sc_expr_str_fn) for node in self.nodes)

    #
    # Private methods
    #

    def _assignable(self):
        # Worker function for the 'assignable' attribute

        if self.orig_type != BOOL:
            return ()

        # Warning: See Symbol._rec_invalidate(), and note that this is a hidden
        # function call (property magic)
        vis = self.visibility
        if not vis:
            return ()

        rev_dep_val = expr_value(self.rev_dep)

        if vis == 2:
            if self.choice:
                return (2,)

            if not rev_dep_val:
                return (0, 2)

            if rev_dep_val == 2:
                return (2,)

            return (2,)

        if not rev_dep_val:
            return () if expr_value(self.weak_rev_dep) != 2 else (0, 2)

        if rev_dep_val == 2:
            return (2,)

        return (2,)

    def _invalidate(self):
        # Marks the symbol as needing to be recalculated

        self._cached_str_val = self._cached_bool_val = self._cached_vis = self._cached_assignable = None

    def _rec_invalidate(self):
        # Invalidates the symbol and all items that (possibly) depend on it

        self._invalidate()

        for item in self._dependents:
            # _cached_vis doubles as a flag that tells us whether 'item'
            # has cached values, because it's calculated as a side effect
            # of calculating all other (non-constant) cached values.
            #
            # If item._cached_vis is None, it means there can't be cached
            # values on other items that depend on 'item', because if there
            # were, some value on 'item' would have been calculated and
            # item._cached_vis set as a side effect. It's therefore safe to
            # stop the invalidation at symbols with _cached_vis None.
            #
            # This approach massively speeds up scripts that set a lot of
            # values, vs simply invalidating all possibly dependent symbols
            # (even when you already have a list of all the dependent
            # symbols, because some symbols get huge dependency trees).
            #
            # This gracefully handles dependency loops too, which is nice
            # for choices, where the choice depends on the choice symbols
            # and vice versa.
            if item._cached_vis is not None:
                item._rec_invalidate()

    def _rec_invalidate_if_has_prompt(self):
        # Invalidates the symbol and its dependent symbols, but only if the
        # symbol has a prompt. User values never have an effect on promptless
        # symbols, so we skip invalidation for them as an optimization.
        #
        # This also prevents constant (quoted) symbols from being invalidated
        # if set_value() is called on them, which would make them lose their
        # value and break things.
        #
        # Prints a warning if the symbol has no prompt. In some contexts (e.g.
        # when loading a .config files) assignments to promptless symbols are
        # normal and expected, so the warning can be disabled.

        for node in self.nodes:
            if node.prompt:
                self._rec_invalidate()
                return

        if self.kconfig._warn_assign_no_prompt:
            self.kconfig._warn(self.name_and_loc + " has no prompt, meaning user values have no effect on it")

    def _str_default(self):
        # write_min_config() helper function. Returns the value the symbol
        # would get from defaults if it didn't have a user value. Uses exactly
        # the same algorithm as the C implementation (though a bit cleaned up),
        # for compatibility.

        if self.orig_type == BOOL:
            val = 0

            # Defaults, selects, and implies do not affect choice symbols
            if not self.choice:
                for default, cond in self.defaults:
                    cond_val = expr_value(cond)
                    if cond_val:
                        val = min(expr_value(default), cond_val)
                        break

                val = max(expr_value(self.rev_dep), expr_value(self.weak_rev_dep), val)

            return BOOL_TO_STR[val]

        if self.orig_type:  # STRING/INT/HEX
            for default, cond in self.defaults:
                if expr_value(cond):
                    return default.str_value

        return ""

    def _warn_select_unsatisfied_deps(self):
        # Helper for printing an informative warning when a symbol with
        # unsatisfied direct dependencies (dependencies from 'depends on', ifs,
        # and menus) is selected by some other symbol.

        msg = (
            f"{self.name_and_loc} has direct dependencies {expr_str(self.direct_dep)} with value "
            f"{BOOL_TO_STR[expr_value(self.direct_dep)]}, but is currently being "
            f"{BOOL_TO_STR[expr_value(self.rev_dep)]}-selected by the following symbols:"
        )

        # The reverse dependencies from each select are ORed together
        for select in split_expr(self.rev_dep, OR):
            if expr_value(select) <= expr_value(self.direct_dep):
                # Only include selects that exceed the direct dependencies
                continue

            # - 'select A if B' turns into A && B
            # - 'select A' just turns into A
            #
            # In both cases, we can split on AND and pick the first operand
            selecting_sym = split_expr(select, AND)[0]

            msg += (
                f"\n - {selecting_sym.name_and_loc}, with value {selecting_sym.str_value}, direct dependencies "
                f"{expr_str(selecting_sym.direct_dep)} (value: {BOOL_TO_STR[expr_value(selecting_sym.direct_dep)]})"
            )

            if select.__class__ is tuple:
                msg += f", and select condition {expr_str(select[2])} (value: {BOOL_TO_STR[expr_value(select[2])]})"

        self.kconfig._warn(msg)


class Choice:
    """
    Represents a choice statement:

      choice
          ...
      endchoice

    Note: Prompts, help texts, and locations are stored in the Choice's
    MenuNode(s) rather than in the Choice itself. Check the MenuNode class and
    the Choice.nodes attribute. This organization matches the C tools.
    """

    __slots__ = (
        "_cached_assignable",
        "_cached_selection",
        "_cached_vis",
        "_dependents",
        "_visited",
        "_was_set",
        "defaults",
        "direct_dep",
        "is_constant",
        "kconfig",
        "name",
        "nodes",
        "orig_type",
        "syms",
        "_user_selection",
        "_user_value",
    )
    kconfig: Kconfig
    name: Optional[str]
    nodes: List["MenuNode"]
    syms: List[Symbol]
    defaults: List[Tuple[Any, Any]]  # Tuple[condition, expression], both can be pretty complicated

    def __init__(self, kconfig: Kconfig, name: Optional[str] = None, direct_dep: Optional[Symbol] = None):
        self.kconfig = kconfig
        self.name = name

        """
        Same as Symbol.direct_dep, but for the choice itself.
        """
        self.direct_dep = direct_dep

        """
        orig_type:
            The type as given in the Kconfig file, without any changes applied. Used
            when printing the choice.
        """
        self.orig_type = UNKNOWN

        """
        nodes:
            A list of MenuNodes for this choice. In practice, the list will probably
            always contain a single MenuNode, but it is possible to give a choice a
            name and define it in multiple locations.
        """
        self.nodes = []

        """
        syms:
            List of symbols contained in the choice.

            Obscure gotcha: If a symbol depends on the previous symbol within a
            choice so that an implicit menu is created, it won't be a choice symbol,
            and won't be included in 'syms'.
        """
        self.syms = []

        """
        defaults:
            List of (symbol, cond) tuples for the choice's 'defaults' properties. For
            example, 'default A if B && C' is represented as (A, (AND, B, C)). If
            there is no condition, 'cond' is self.kconfig.y.

            Note that 'depends on' and parent dependencies are propagated to
            'default' conditions.
        """
        self.defaults = []

        """
        _user_value:
            The value (mode) selected by the user through Choice.set_value(). Either
            0 or 2, or None if the user hasn't selected a mode. See
            Symbol._user_value.

            WARNING: Do not assign directly to this. It will break things. Use
            Choice.set_value() instead.
        """
        self._user_value = None

        """
        user_selection:
            The symbol selected by the user (by setting it to y). Ignored if the
            choice is not in y mode, but still remembered so that the choice "snaps
            back" to the user selection if the mode is changed back to y. This might
            differ from 'selection' due to unsatisfied dependencies.

            WARNING: Do not assign directly to this. It will break things. Call
            sym.set_value(2) on the choice symbol to be selected instead.
        """
        self._user_selection = None

        # Internal attributes
        self._visited = UNKNOWN
        self._cached_vis = None
        self._cached_assignable = None
        self._cached_selection = _NO_CACHED_SELECTION

        # is_constant is checked by _depend_on(). Just set it to avoid having
        # to special-case choices.
        self.is_constant = False

        # See Kconfig._build_dep()
        self._dependents = set()  # type: ignore

    @property
    def type(self):
        """
        The type of the choice. One of BOOL, UNKNOWN. UNKNOWN is for
        choices defined without a type where none of the contained symbols have a
        type either (otherwise the choice inherits the type of the first symbol
        defined with a type).
        """
        return self.orig_type

    @property
    def str_value(self):
        """
        Like choice.bool_value, but gives the value as one of the strings
        "n", or "y"
        """
        return BOOL_TO_STR[self.bool_value]

    @property
    def bool_value(self):
        """
        The bool value (mode) of the choice. A choice can be in one of two
        modes:

            0 (n) - The choice is disabled and no symbols can be selected. For
                    visible choices, this mode is only possible for choices with
                    the 'optional' flag set (see kconfig-language.txt).

            2 (y) - One symbol will be y, the rest n.

        The visibility of the choice is an upper bound on the mode, and the mode in
        turn is an upper bound on the visibility of the choice symbols.

        To change the mode, use Choice.set_value().

        Implementation note:
            The C tools internally represent choices as a type of symbol, with
            special-casing in many code paths. This is why there is a lot of
            similarity to Symbol.

            Symbols within choices get the choice propagated as a dependency to
            their properties. This turns the mode of the choice into an upper bound
            on e.g. the visibility of choice symbols, and explains the gotcha
            related to printing choice symbols mentioned in the module docstring.

            Kconfiglib uses a separate Choice class only because it makes the code
            and interface less confusing (especially in a user-facing interface).
            Corresponding attributes have the same name in the Symbol and Choice
            classes, for consistency and compatibility.
        """

        val = 2

        if self._user_value is not None:
            val = max(val, self._user_value)

        # Warning: See Symbol._rec_invalidate(), and note that this is a hidden
        # function call (property magic)
        val = min(val, self.visibility)

        return val

    @property
    def assignable(self):
        """
        A tuple containing the bool user values that can currently be
        assigned to the choice (that would be respected), ordered from lowest (0,
        representing n) to highest (2, representing y). This corresponds to the
        selections available in the menuconfig interface. The set of assignable
        values is calculated from the choice's visibility and selects/implies.

        Returns the empty set for non-bool choice and for choice with
        visibility n. The other possible values are (0, 2) and (2,). A (2,) result means
        the choice is visible but "locked" to y through a select, perhaps in combination with the
        visibility. menuconfig represents this as -*-.
        """
        if self._cached_assignable is None:
            self._cached_assignable = self._assignable()
        return self._cached_assignable

    @property
    def visibility(self):
        """
        The visibility of the choice. One of 0, 2, representing n, y. See
        the module documentation for an overview of symbol values and visibility.
        """
        if self._cached_vis is None:
            self._cached_vis = _visibility(self)
        return self._cached_vis

    @property
    def name_and_loc(self):
        """
        Holds a string like

            "<choice MY_CHOICE> (defined at foo/Kconfig:12)"

        , giving the name of the choice and its definition location(s). If the
        choice has no name (isn't defined with 'choice MY_CHOICE'), then it will
        be shown as "<choice>" before the list of locations (always a single one
        in that case)..
        """
        # Reuse the expression format, which is '<choice (name, if any)>'.
        return standard_sc_expr_str(self) + " " + _locs(self)

    @property
    def selection(self):
        """
        The Symbol instance of the currently selected symbol. None if the Choice
        is not in y mode or has no selected symbol (due to unsatisfied
        dependencies on choice symbols).

        WARNING: Do not assign directly to this. It will break things. Call
        sym.set_value(2) on the choice symbol you want to select instead.
        """
        if self._cached_selection is _NO_CACHED_SELECTION:
            self._cached_selection = self._selection()
        return self._cached_selection

    def set_value(self, value):
        """
        Sets the user value (mode) of the choice. Like for Symbol.set_value(),
        the visibility might truncate the value. Choices can never be in n mode,
        but 0/"n" is still accepted since it's not a malformed value (though it
        will have no effect).

        Returns True if the value is valid for the type of the choice, and
        False otherwise. This only looks at the form of the value. Check the
        Choice.assignable attribute to see what values are currently in range
        and would actually be reflected in the mode of the choice.
        """
        if value in STR_TO_BOOL:
            value = STR_TO_BOOL[value]

        if value == self._user_value:
            # We know the value must be valid if it was successfully set
            # previously
            self._was_set = True
            return True

        if not (self.orig_type == BOOL and value in (2, 0)):
            # Display bool values as n and y in the warning
            self.kconfig._warn(
                "the value {} is invalid for {}, which has type {} -- assignment ignored".format(
                    BOOL_TO_STR[value] if value in BOOL_TO_STR else f"'{value}'",
                    self.name_and_loc,
                    TYPE_TO_STR[self.orig_type],
                )
            )

            return False

        self._user_value = value
        self._was_set = True
        self._rec_invalidate()

        return True

    def unset_value(self):
        """
        Resets the user value (mode) and user selection of the Choice, as if
        the user had never touched the mode or any of the choice symbols.
        """
        if self._user_value is not None or self._user_selection:
            self._user_value = self._user_selection = None
            self._rec_invalidate()

    @property
    def referenced(self):
        """
        See the class documentation.
        """
        return {item for node in self.nodes for item in node.referenced}

    @property
    def orig_defaults(self):
        """
        See the corresponding attribute on the MenuNode class.
        """
        return [d for node in self.nodes for d in node.orig_defaults]

    def __repr__(self):
        """
        Returns a string with information about the choice when it is evaluated
        on e.g. the interactive Python prompt.
        """
        fields = [
            "choice " + self.name if self.name else "choice",
            TYPE_TO_STR[self.type],
        ]
        add = fields.append

        for node in self.nodes:
            if node.prompt:
                add(f'"{node.prompt[0]}"')

        add("mode " + self.str_value)

        if self._user_value is not None:
            add(f"user mode {BOOL_TO_STR[self._user_value]}")

        if self.selection:
            add(f"{self.selection.name} selected")

        if self._user_selection:
            user_sel_str = f"{self._user_selection.name} selected by user"

            if self.selection is not self._user_selection:
                user_sel_str += " (overridden)"

            add(user_sel_str)

        add("visibility " + BOOL_TO_STR[self.visibility])

        for node in self.nodes:
            add(f"{node.filename}:{node.linenr}")

        return f"<{', '.join(fields)}>"

    def __str__(self):
        """
        Returns a string representation of the choice when it is printed.
        Matches the Kconfig format (though without the contained choice
        symbols), with any parent dependencies propagated to the 'depends on'
        condition.

        The returned string does not end in a newline.

        See Symbol.__str__() as well.
        """
        return self.custom_str(standard_sc_expr_str)

    def custom_str(self, sc_expr_str_fn):
        """
        Works like Choice.__str__(), but allows a custom format to be used for
        all symbol/choice references. See expr_str().
        """
        return "\n\n".join(node.custom_str(sc_expr_str_fn) for node in self.nodes)

    def _assignable(self):
        # Worker function for the 'assignable' attribute

        # Warning: See Symbol._rec_invalidate(), and note that this is a hidden
        # function call (property magic)
        vis = self.visibility

        if not vis:
            return ()

        if vis == 2:
            return (2,)

        return ()

    def _selection(self):
        # Worker function for the 'selection' attribute

        # Warning: See Symbol._rec_invalidate(), and note that this is a hidden
        # function call (property magic)
        if self.bool_value != 2:
            # Not in y mode, so no selection
            return None

        # Use the user selection if it's visible
        if self._user_selection and self._user_selection.visibility:
            return self._user_selection

        # Otherwise, check if we have a default
        return self._selection_from_defaults()

    def _selection_from_defaults(self):
        # Check if we have a default
        for sym, cond in self.defaults:
            # The default symbol must be visible too
            if expr_value(cond) and sym.visibility:
                return sym

        # Otherwise, pick the first visible symbol, if any
        for sym in self.syms:
            if sym.visibility:
                return sym

        # Couldn't find a selection
        return None

    def _invalidate(self):
        self._cached_vis = self._cached_assignable = None
        self._cached_selection = _NO_CACHED_SELECTION

    def _rec_invalidate(self):
        # See Symbol._rec_invalidate()

        self._invalidate()

        for item in self._dependents:
            if item._cached_vis is not None:
                item._rec_invalidate()


class MenuNode:
    """
    Represents a menu node in the configuration. This corresponds to an entry
    in e.g. the 'make menuconfig' interface, though non-visible choices, menus,
    and comments also get menu nodes. If a symbol or choice is defined in
    multiple locations, it gets one menu node for each location.

    The top-level menu node, corresponding to the implicit top-level menu, is
    available in Kconfig.top_node.

    The menu nodes for a Symbol or Choice can be found in the
    Symbol/Choice.nodes attribute. Menus and comments are represented as plain
    menu nodes, with their text stored in the prompt attribute (prompt[0]).
    This mirrors the C implementation.

    The following attributes are available on MenuNode instances. They should
    be viewed as read-only.
    """

    __slots__ = (
        "dep",
        "filename",
        "help",
        "include_path",  # path which current file was sourced from
        "is_menuconfig",
        "item",
        "kconfig",
        "linenr",
        "list",
        "next",
        "parent",
        "prompt",
        "visibility",
        # Properties
        "defaults",
        "selects",
        "implies",
        "ranges",
    )
    item: Optional[Union[Symbol, Choice, int]]
    is_menuconfig: bool
    dep: Optional[Union[Tuple, "Symbol"]]
    kconfig: "Kconfig"
    help: Optional[str]
    next: Optional["MenuNode"]
    parent: Optional["MenuNode"]
    prompt: Optional[Tuple]
    defaults: List[Tuple[Any, Any]]  # Tuple[condition, expression], both can be pretty complicated
    selects: List[Tuple]
    implies: List[Tuple]
    ranges: List[Tuple]
    list: Optional["MenuNode"]

    def __init__(
        self,
        kconfig: Kconfig,
        item: Optional[Union[Symbol, Choice, int]] = None,
        is_menuconfig: bool = False,
        filename: Optional[str] = None,
        linenr: Optional[int] = None,
        dep: Optional[Union[Tuple, "Symbol"]] = None,
        visibility: Optional[Union[Tuple, "Symbol"]] = None,
        parent: Optional["MenuNode"] = None,
        help: Optional[str] = None,
        prompt: Optional[Tuple] = None,
    ):
        # Properties defined on this particular menu node. A local 'depends on'
        # only applies to these, in case a symbol is defined in multiple
        # locations.
        self.kconfig = kconfig

        """
        item:
            Either a Symbol, a Choice, or one of the constants MENU and COMMENT.
            Menus and comments are represented as plain menu nodes. Ifs are collapsed
            (matching the C implementation) and do not appear in the final menu tree.
        """
        self.item = item

        """
        is_menuconfig:
            Set to True if the children of the menu node should be displayed in a
            separate menu. This is the case for the following items:

                - Menus (node.item == MENU)

                - Choices

                - Symbols defined with the 'menuconfig' keyword. The children come from
                implicitly created submenus, and should be displayed in a separate
                menu rather than being indented.

            'is_menuconfig' is just a hint on how to display the menu node. It's
            ignored internally by Kconfiglib, except when printing symbols.
        """
        self.is_menuconfig = is_menuconfig

        """
        filename/linenr:
            The location where the menu node appears. The filename is relative to
            $srctree (or to the current directory if $srctree isn't set), except
            absolute paths are used for paths outside $srctree.
        """
        self.filename = filename
        self.linenr = linenr

        """
        dep:
            The direct ('depends on') dependencies for the menu node, or
            self.kconfig.y if there are no direct dependencies.

            This attribute includes any dependencies from surrounding menus and ifs.
            Those get propagated to the direct dependencies, and the resulting direct
            dependencies in turn get propagated to the conditions of all properties.

            If a symbol or choice is defined in multiple locations, only the
            properties defined at a particular location get the corresponding
            MenuNode.dep dependencies propagated to them.
        """
        self.dep = dep or kconfig.y

        """
        visibility:
            The 'visible if' dependencies for the menu node (which must represent a
            menu), or self.kconfig.y if there are no 'visible if' dependencies.
            'visible if' dependencies are recursively propagated to the prompts of
            symbols and choices within the menu.
        """
        self.visibility = visibility or kconfig.y

        """
        include_path:
            A tuple of (filename, linenr) tuples, giving the locations of the
            'source' statements via which the Kconfig file containing this menu node
            was included. The first element is the location of the 'source' statement
            in the top-level Kconfig file passed to Kconfig.__init__(), etc.

            Note that the Kconfig file of the menu node itself isn't included. Check
            'filename' and 'linenr' for that.
            TODO: Is it used anywhere both in old and new code?
        """
        self.include_path = kconfig._include_path

        # Menu tree related properties
        """
        list:
            The first child menu node. None if there are no children.

            Choices and menus naturally have children, but Symbols can also have
            children because of menus created automatically from dependencies (see
            kconfig-language.txt). The children of a symbol are the menu nodes
        """
        self.list = None

        """
        next:
            The following menu node. None if there is no following node.
        """
        self.next = None

        """
        parent:
            The parent menu node. None if there is no parent.
        """
        self.parent = parent

        # Properties of the item that needs to be stored in MenuNode.
        # e.g. Symbol can be defined in two different locations with two different help texts
        # We want to have track of both help texts together with the location -> this is better obtained
        # from MenuNode than item/Symbol.

        """
        prompt:
            A (string, cond) tuple with the prompt for the menu node and its
            conditional expression (which is self.kconfig.y if there is no
            condition). None if there is no prompt.

            For symbols and choices, the prompt is stored in the MenuNode rather than
            the Symbol or Choice instance. For menus and comments, the prompt holds
            the text.
        """
        self.prompt = prompt

        """
        help:
            The help text for the menu node for Symbols and Choices. None if there is
            no help text. Always stored in the node rather than the Symbol or Choice.
            It is possible to have a separate help text at each location if a symbol
            is defined in multiple locations.

            Trailing whitespace (including a final newline) is stripped from the help
            text. This was not the case before Kconfiglib 10.21.0, where the format
            was undocumented.
        """
        self.help = help

        """
        defaults:
            The 'default' properties for this particular menu node. See
            symbol.defaults.

            When evaluating defaults, you should use Symbol/Choice.defaults instead,
            as it include properties from all menu nodes (a symbol/choice can have
            multiple definition locations/menu nodes). MenuNode.defaults is meant for
            documentation generation.
        """
        self.defaults = []

        """
        selects:
            Like MenuNode.defaults, for selects.
        """
        self.selects = []

        """
        implies:
            Like MenuNode.defaults, for implies.
        """
        self.implies = []

        """
        ranges:
            Like MenuNode.defaults, for ranges.
        """
        self.ranges = []

    @property
    def orig_prompt(self):
        """
        orig_prompt:
          It works the like the corresponding attribute without orig_*, but omits
          any dependencies propagated from 'depends on' and surrounding 'if's (the
          direct dependencies, stored in MenuNode.dep).

          One use for this is generating less cluttered documentation, by only
          showing the direct dependencies in one place.
        """
        if not self.prompt:
            return None
        return (self.prompt[0], self._strip_dep(self.prompt[1]))

    @property
    def orig_defaults(self):
        """
        orig_defaults:
          It works the like the corresponding attribute without orig_*, but omits
          any dependencies propagated from 'depends on' and surrounding 'if's (the
          direct dependencies, stored in MenuNode.dep).

          One use for this is generating less cluttered documentation, by only
          showing the direct dependencies in one place.
        """
        return [(default, self._strip_dep(cond)) for default, cond in self.defaults]

    @property
    def orig_selects(self):
        """
        orig_selects:
          It works the like the corresponding attribute without orig_*, but omits
          any dependencies propagated from 'depends on' and surrounding 'if's (the
          direct dependencies, stored in MenuNode.dep).

          One use for this is generating less cluttered documentation, by only
          showing the direct dependencies in one place.
        """
        return [(select, self._strip_dep(cond)) for select, cond in self.selects]

    @property
    def orig_implies(self):
        """
        orig_implies:
          It works the like the corresponding attribute without orig_*, but omits
          any dependencies propagated from 'depends on' and surrounding 'if's (the
          direct dependencies, stored in MenuNode.dep).

          One use for this is generating less cluttered documentation, by only
          showing the direct dependencies in one place.
        """
        return [(imply, self._strip_dep(cond)) for imply, cond in self.implies]

    @property
    def orig_ranges(self):
        """
        orig_ranges:
          It works the like the corresponding attribute without orig_*, but omits
          any dependencies propagated from 'depends on' and surrounding 'if's (the
          direct dependencies, stored in MenuNode.dep).

          One use for this is generating less cluttered documentation, by only
          showing the direct dependencies in one place.
        """
        return [(low, high, self._strip_dep(cond)) for low, high, cond in self.ranges]

    @property
    def referenced(self):
        """
        A set() with all symbols and choices referenced in the properties and
        property conditions of the menu node.

        Also includes dependencies inherited from surrounding menus and ifs.
        Choices appear in the dependencies of choice symbols.
        """
        # self.dep is included to catch dependencies from a lone 'depends on'
        # when there are no properties to propagate it to
        res = expr_items(self.dep)

        if self.prompt:
            res |= expr_items(self.prompt[1])

        if self.item is MENU:
            res |= expr_items(self.visibility)

        for value, cond in self.defaults:
            res |= expr_items(value)
            res |= expr_items(cond)

        for value, cond in self.selects:
            res.add(value)
            res |= expr_items(cond)

        for value, cond in self.implies:
            res.add(value)
            res |= expr_items(cond)

        for low, high, cond in self.ranges:
            res.add(low)
            res.add(high)
            res |= expr_items(cond)

        return res

    def __repr__(self):
        """
        Returns a string with information about the menu node when it is
        evaluated on e.g. the interactive Python prompt.
        """
        fields = []
        add = fields.append

        if self.item.__class__ is Symbol:
            add("menu node for symbol " + self.item.name)

        elif self.item.__class__ is Choice:
            s = "menu node for choice"
            if self.item.name is not None:
                s += " " + self.item.name
            add(s)

        elif self.item == MENU:
            add("menu node for menu")

        else:  # self.item is COMMENT
            add("menu node for comment")

        if self.prompt:
            add(f'prompt "{self.prompt[0]}" (visibility {BOOL_TO_STR[expr_value(self.prompt[1])]})')

        if self.item.__class__ is Symbol and self.is_menuconfig:
            add("is menuconfig")

        add("deps " + BOOL_TO_STR[expr_value(self.dep)])

        if self.item == MENU:
            add("'visible if' deps " + BOOL_TO_STR[expr_value(self.visibility)])

        if self.item.__class__ in _SYMBOL_CHOICE and self.help is not None:
            add("has help")

        if self.list:
            add("has child")

        if self.next:
            add("has next")

        add(f"{self.filename}:{self.linenr}")

        return f"<{', '.join(fields)}>"

    def __str__(self):
        """
        Returns a string representation of the menu node. Matches the Kconfig
        format, with any parent dependencies propagated to the 'depends on'
        condition.

        The output could (almost) be fed back into a Kconfig parser to redefine
        the object associated with the menu node. See the module documentation
        for a gotcha related to choice symbols.

        For symbols and choices with multiple menu nodes (multiple definition
        locations), properties that aren't associated with a particular menu
        node are shown on all menu nodes ('option env=...', 'optional' for
        choices, etc.).

        The returned string does not end in a newline.
        """
        return self.custom_str(standard_sc_expr_str)

    def custom_str(self, sc_expr_str_fn):
        """
        Works like MenuNode.__str__(), but allows a custom format to be used
        for all symbol/choice references. See expr_str().
        """
        return (
            self._menu_comment_node_str(sc_expr_str_fn)
            if self.item in _MENU_COMMENT
            else self._sym_choice_node_str(sc_expr_str_fn)
        )

    def _menu_comment_node_str(self, sc_expr_str_fn):
        s = f'{"menu" if self.item == MENU else "comment"} "{self.prompt[0]}"'

        if self.dep is not self.kconfig.y:
            s += f"\n\tdepends on {expr_str(self.dep, sc_expr_str_fn)}"

        if self.item == MENU and self.visibility is not self.kconfig.y:
            s += f"\n\tvisible if {expr_str(self.visibility, sc_expr_str_fn)}"

        return s

    def _sym_choice_node_str(self, sc_expr_str_fn):
        def indent_add(s):
            lines.append("\t" + s)

        def indent_add_cond(s, cond):
            if cond is not self.kconfig.y:
                s += " if " + expr_str(cond, sc_expr_str_fn)
            indent_add(s)

        sc = self.item

        if sc.__class__ is Symbol:
            lines = [("menuconfig " if self.is_menuconfig else "config ") + sc.name]
        else:
            lines = ["choice " + sc.name if sc.name else "choice"]

        if sc.orig_type and not self.prompt:  # sc.orig_type != UNKNOWN
            # If there's a prompt, we'll use the '<type> "prompt"' shorthand
            # instead
            indent_add(TYPE_TO_STR[sc.orig_type])

        if self.prompt:
            if sc.orig_type:
                prefix = TYPE_TO_STR[sc.orig_type]
            else:
                # Symbol defined without a type (which generates a warning)
                prefix = "prompt"

            indent_add_cond(prefix + f' "{escape(self.prompt[0])}"', self.orig_prompt[1])

        if sc.__class__ is Symbol:
            if sc.is_allnoconfig_y:
                indent_add("option allnoconfig_y")

            if sc is sc.kconfig.defconfig_list:
                indent_add("option defconfig_list")

            if sc.env_var is not None:
                indent_add(f'option env="{sc.env_var}"')

            for low, high, cond in self.orig_ranges:
                indent_add_cond(
                    f"range {sc_expr_str_fn(low)} {sc_expr_str_fn(high)}",
                    cond,
                )

        for default, cond in self.orig_defaults:
            indent_add_cond("default " + expr_str(default, sc_expr_str_fn), cond)

        if sc.__class__ is Symbol:
            for select, cond in self.orig_selects:
                indent_add_cond("select " + sc_expr_str_fn(select), cond)

            for imply, cond in self.orig_implies:
                indent_add_cond("imply " + sc_expr_str_fn(imply), cond)

        if self.dep is not sc.kconfig.y:
            indent_add("depends on " + expr_str(self.dep, sc_expr_str_fn))

        if self.help is not None:
            indent_add("help")
            for line in self.help.splitlines():
                indent_add("  " + line)

        return "\n".join(lines)

    def _strip_dep(self, expr):
        # Helper function for removing MenuNode.dep from 'expr'. Uses two
        # pieces of internal knowledge: (1) Expressions are reused rather than
        # copied, and (2) the direct dependencies always appear at the end.

        # ... if dep -> ... if y
        if self.dep is expr:
            return self.kconfig.y

        # (AND, X, dep) -> X
        if expr.__class__ is tuple and expr[0] == AND and expr[2] is self.dep:
            return expr[1]

        return expr


class Variable(object):
    """
    # TODO/NOTE Variable/preprocessor logic can be removed, thus this class was not refactored
    Represents a preprocessor variable/function.

    The following attributes are available:

    name:
      The name of the variable.

    value:
      The unexpanded value of the variable.

    expanded_value:
      The expanded value of the variable. For simple variables (those defined
      with :=), this will equal 'value'. Accessing this property will raise a
      KconfigError if the expansion seems to be stuck in a loop.

      Accessing this field is the same as calling expanded_value_w_args() with
      no arguments. I hadn't considered function arguments when adding it. It
      is retained for backwards compatibility though.

    is_recursive:
      True if the variable is recursive (defined with =).
    """

    __slots__ = (
        "_n_expansions",
        "is_recursive",
        "kconfig",
        "name",
        "value",
    )

    @property
    def expanded_value(self):
        """
        See the class documentation.
        """
        return self.expanded_value_w_args()

    def expanded_value_w_args(self, *args):
        """
        Returns the expanded value of the variable/function. Any arguments
        passed will be substituted for $(1), $(2), etc.

        Raises a KconfigError if the expansion seems to be stuck in a loop.
        """
        return self.kconfig._fn_val((self.name,) + args)

    def __repr__(self):
        return f"<variable {self.name}, {'recursive' if self.is_recursive else 'immediate'}, value '{self.value}'>"


class KconfigError(Exception):
    """
    Exception raised for Kconfig-related errors.

    KconfigError and KconfigSyntaxError are the same class. The
    KconfigSyntaxError alias is only maintained for backwards compatibility.
    """


KconfigSyntaxError = KconfigError  # Backwards compatibility


class InternalError(Exception):
    "Never raised. Kept around for backwards compatibility."


# Workaround:
#
# If 'errno' and 'strerror' are set on IOError, then __str__() always returns
# "[Errno <errno>] <strerror>", ignoring any custom message passed to the
# constructor. By defining our own subclass, we can use a custom message while
# also providing 'errno', 'strerror', and 'filename' to scripts.
class _KconfigIOError(IOError):
    def __init__(self, ioerror, msg):
        self.msg = msg
        super(_KconfigIOError, self).__init__(ioerror.errno, ioerror.strerror, ioerror.filename)

    def __str__(self):
        return self.msg


#
# Public functions
#


def expr_value(expr: Union[Symbol, Choice, Tuple]) -> int:
    """
    Evaluates the expression 'expr' to a bool value. Returns 0 (n), or 2 (y).

    'expr' must be an already-parsed expression from a Symbol, Choice, or
    MenuNode property. To evaluate an expression represented as a string, use
    Kconfig.eval_string().

    Passing subexpressions of expressions to this function works as expected.
    """

    if not isinstance(expr, tuple):
        return expr.bool_value

    if expr[0] == AND:
        val_left_side = expr_value(expr[1])  # expr = (AND, left, right)
        # Short-circuit the n case as an optimization (~5% faster
        # allnoconfig.py and allyesconfig.py, as of writing)
        return 0 if not val_left_side else min(val_left_side, expr_value(expr[2]))

    if expr[0] == OR:
        val_left_side = expr_value(expr[1])  # expr = (OR, left, right)
        # Short-circuit the y case as an optimization
        return 2 if val_left_side == 2 else max(val_left_side, expr_value(expr[2]))

    if expr[0] == NOT:
        return 2 - expr_value(expr[1])

    # Relation
    #
    # Implements <, <=, >, >= comparisons as well. These were added to
    # kconfig in 31847b67 (kconfig: allow use of relations other than
    # (in)equality).
    v1: Union[Symbol, Choice]
    v2: Union[Symbol, Choice]
    rel, v1, v2 = expr

    # If both operands are strings...
    if v1.orig_type == STRING and v2.orig_type == STRING:
        # ...then compare them lexicographically
        comp = _strcmp(v1.str_value, v2.str_value)
    else:
        # Otherwise, try to compare them as numbers
        try:
            comp = _sym_to_num(v1) - _sym_to_num(v2)
        except ValueError:
            # Fall back on a lexicographic comparison if the operands don't
            # parse as numbers
            comp = _strcmp(v1.str_value, v2.str_value)

    return 2 * (  # type: ignore
        comp == 0
        if rel == EQUAL
        else comp != 0
        if rel == UNEQUAL
        else comp < 0
        if rel == LESS
        else comp <= 0
        if rel == LESS_EQUAL
        else comp > 0
        if rel == GREATER
        else comp >= 0
    )


def standard_sc_expr_str(sc):
    """
    Standard symbol/choice printing function. Uses plain Kconfig syntax, and
    displays choices as <choice> (or <choice NAME>, for named choices).

    See expr_str().
    """
    if sc.__class__ is Symbol:
        if sc.is_constant and sc.name not in STR_TO_BOOL:
            return f'"{escape(sc.name)}"'
        return sc.name

    return f"<choice {sc.name}>" if sc.name else "<choice>"


def _parenthesize(expr, type_, sc_expr_str_fn):
    # expr_str() helper. Adds parentheses around expressions of type 'type_'.

    if expr.__class__ is tuple and expr[0] is type_:
        return f"({expr_str(expr, sc_expr_str_fn)})"
    return expr_str(expr, sc_expr_str_fn)


def expr_str(expr, sc_expr_str_fn=standard_sc_expr_str):
    """
    Returns the string representation of the expression 'expr', as in a Kconfig
    file.

    Passing subexpressions of expressions to this function works as expected.

    sc_expr_str_fn (default: standard_sc_expr_str):
      This function is called for every symbol/choice (hence "sc") appearing in
      the expression, with the symbol/choice as the argument. It is expected to
      return a string to be used for the symbol/choice.

      This can be used e.g. to turn symbols/choices into links when generating
      documentation, or for printing the value of each symbol/choice after it.

      Note that quoted values are represented as constants symbols
      (Symbol.is_constant == True).
    """
    if expr.__class__ is not tuple:
        return sc_expr_str_fn(expr)

    if expr[0] == AND:
        return f"{_parenthesize(expr[1], OR, sc_expr_str_fn)} && {_parenthesize(expr[2], OR, sc_expr_str_fn)}"

    if expr[0] == OR:
        # This turns A && B || C && D into "(A && B) || (C && D)", which is
        # redundant, but more readable
        return f"{_parenthesize(expr[1], AND, sc_expr_str_fn)} || {_parenthesize(expr[2], AND, sc_expr_str_fn)}"

    if expr[0] == NOT:
        if expr[1].__class__ is tuple:
            return f"!({expr_str(expr[1], sc_expr_str_fn)})"
        return "!" + sc_expr_str_fn(expr[1])  # Symbol

    # Relation
    #
    # Relation operands are always symbols (quoted strings are constant
    # symbols)
    return f"{sc_expr_str_fn(expr[1])} {REL_TO_STR[expr[0]]} {sc_expr_str_fn(expr[2])}"


def expr_items(expr):
    """
    Returns a set() of all items (symbols and choices) that appear in the
    expression 'expr'.

    Passing subexpressions of expressions to this function works as expected.
    """
    res = set()

    def rec(subexpr):
        if subexpr.__class__ is tuple:
            # AND, OR, NOT, or relation

            rec(subexpr[1])

            # NOTs only have a single operand
            if subexpr[0] != NOT:
                rec(subexpr[2])

        else:
            # Symbol or choice
            res.add(subexpr)

    rec(expr)
    return res


def split_expr(expr, op):
    """
    Returns a list containing the top-level AND or OR operands in the
    expression 'expr', in the same (left-to-right) order as they appear in
    the expression.

    This can be handy e.g. for splitting (weak) reverse dependencies
    from 'select' and 'imply' into individual selects/implies.

    op:
      Either AND to get AND operands, or OR to get OR operands.

      (Having this as an operand might be more future-safe than having two
      hardcoded functions.)


    Pseudo-code examples:

      split_expr( A                    , OR  )  ->  [A]
      split_expr( A && B               , OR  )  ->  [A && B]
      split_expr( A || B               , OR  )  ->  [A, B]
      split_expr( A || B               , AND )  ->  [A || B]
      split_expr( A || B || (C && D)   , OR  )  ->  [A, B, C && D]

      # Second || is not at the top level
      split_expr( A || (B && (C || D)) , OR )  ->  [A, B && (C || D)]

      # Parentheses don't matter as long as we stay at the top level (don't
      # encounter any non-'op' nodes)
      split_expr( (A || B) || C        , OR )  ->  [A, B, C]
      split_expr( A || (B || C)        , OR )  ->  [A, B, C]
    """
    res = []

    def rec(subexpr):
        if subexpr.__class__ is tuple and subexpr[0] is op:
            rec(subexpr[1])
            rec(subexpr[2])
        else:
            res.append(subexpr)

    rec(expr)
    return res


def escape(s):
    r"""
    Escapes the string 's' in the same fashion as is done for display in
    Kconfig format and when writing strings to a .config file. " and \ are
    replaced by \" and \\, respectively.
    """
    # \ must be escaped before " to avoid double escaping
    return s.replace("\\", r"\\").replace('"', r"\"")


def unescape(s):
    r"""
    Unescapes the string 's'. \ followed by any character is replaced with just
    that character. Used internally when reading .config files.
    """
    return _unescape_sub(r"\1", s)


# unescape() helper
_unescape_sub = re.compile(r"\\(.)").sub


def standard_kconfig(description=None):
    """
    Argument parsing helper for tools that take a single optional Kconfig file
    argument (default: Kconfig). Returns the Kconfig instance for the parsed
    configuration. Uses argparse internally.

    Exits with sys.exit() (which raises SystemExit) on errors.

    description (default: None):
      The 'description' passed to argparse.ArgumentParser().
      argparse.RawDescriptionHelpFormatter is used, so formatting is preserved.
    """
    import argparse

    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=description)

    parser.add_argument(
        "kconfig",
        metavar="KCONFIG",
        default="Kconfig",
        nargs="?",
        help="Top-level Kconfig file (default: Kconfig)",
    )

    parsed_args = parser.parse_args()

    # Temporary parser version selection via envvar. After the refactor,
    # there would be a dedicated option for this.
    try:
        parser_version = int(os.environ.get("KCONFIG_PARSER_VERSION", "1"))
        kconfig = Kconfig(parsed_args.kconfig, suppress_traceback=True, parser_version=parser_version)
    except (EnvironmentError, KconfigError) as e:
        cmd = sys.argv[0]  # Empty string if missing
        if cmd:
            cmd += ": "
        # Some long exception messages have extra newlines for better
        # formatting when reported as an unhandled exception. Strip
        # them here.
        sys.exit(cmd + str(e).strip())

    return kconfig


def standard_config_filename():
    """
    Helper for tools. Returns the value of KCONFIG_CONFIG (which specifies the
    .config file to load/save) if it is set, and ".config" otherwise.

    Calling load_config() with filename=None might give the behavior you want,
    without having to use this function.
    """
    return os.getenv("KCONFIG_CONFIG", ".config")


#
# Internal functions
#


def _visibility(sc: Union[Symbol, Choice]) -> int:
    # Symbols and Choices have a "visibility" that acts as an upper bound on
    # the values a user can set for them, corresponding to the visibility in
    # e.g. 'make menuconfig'. This function calculates the visibility for the
    # Symbol or Choice 'sc' -- the logic is nearly identical.

    vis = 0  # "n" visibility

    for node in sc.nodes:
        if node.prompt:
            vis = max(vis, expr_value(node.prompt[1]))

    return vis


def _depend_on(sc, expr):
    # Adds 'sc' (symbol or choice) as a "dependee" to all symbols in 'expr'.
    # Constant symbols in 'expr' are skipped as they can never change value
    # anyway.

    if expr.__class__ is tuple:
        # AND, OR, NOT, or relation

        _depend_on(sc, expr[1])

        # NOTs only have a single operand
        if expr[0] != NOT:
            _depend_on(sc, expr[2])
    elif not expr.is_constant:
        # Non-constant symbol, or choice
        expr._dependents.add(sc)


def _ordered_unique(lst):
    # Returns 'lst' with any duplicates removed, preserving order. This hacky
    # version seems to be a common idiom. It relies on short-circuit evaluation
    # and set.add() returning None, which is falsy.

    seen = set()
    seen_add = seen.add
    return [x for x in lst if x not in seen and not seen_add(x)]


def _is_base_n(s, n):
    try:
        int(s, n)
        return True
    except ValueError:
        return False


def _strcmp(s1, s2):
    # TODO: is this C-like logic needed?
    # strcmp()-alike that returns -1, 0, or 1

    return (s1 > s2) - (s1 < s2)


def _sym_to_num(sym):
    # expr_value() helper for converting a symbol to a number. Raises
    # ValueError for symbols that can't be converted.

    # For BOOL n/y count as 0/2.
    return sym.bool_value if sym.orig_type == BOOL else int(sym.str_value, _TYPE_TO_BASE[sym.orig_type])


def _save_old(path):
    # See write_config()

    def copy(src, dst):
        # Import as needed, to save some startup time
        import shutil

        shutil.copyfile(src, dst)

    if islink(path):
        # Preserve symlinks
        copy_fn = copy
    elif hasattr(os, "replace"):
        # Python 3 (3.3+) only. Best choice when available, because it
        # removes <filename>.old on both *nix and Windows.
        copy_fn = os.replace
    elif os.name == "posix":
        # Removes <filename>.old on POSIX systems
        copy_fn = os.rename
    else:
        # Fall back on copying
        copy_fn = copy

    try:
        copy_fn(path, path + ".old")
    except Exception:
        # Ignore errors from 'path' missing as well as other errors.
        # <filename>.old file is usually more of a nice-to-have, and not worth
        # erroring out over e.g. if <filename>.old happens to be a directory or
        # <filename> is something like /dev/null.
        pass


def _locs(sc):
    # Symbol/Choice.name_and_loc helper. Returns the "(defined at ...)" part of
    # the string. 'sc' is a Symbol or Choice.

    if sc.nodes:
        return "(defined at {})".format(", ".join("{0.filename}:{0.linenr}".format(node) for node in sc.nodes))

    return "(undefined)"


# Menu manipulation


def _expr_depends_on(expr, sym):
    # Reimplementation of expr_depends_symbol() from mconf.c. Used to determine
    # if a submenu should be implicitly created. This also influences which
    # items inside choice statements are considered choice items.

    if expr.__class__ is not tuple:
        return expr is sym

    if expr[0] in _EQUAL_UNEQUAL:
        # Check for one of the following:
        # sym = y, y = sym, sym != n, n != sym

        left, right = expr[1:]

        if right is sym:
            left, right = right, left
        elif left is not sym:
            return False

        return (expr[0] == EQUAL and right is sym.kconfig.y) or (expr[0] == UNEQUAL and right is sym.kconfig.n)

    return expr[0] == AND and (_expr_depends_on(expr[1], sym) or _expr_depends_on(expr[2], sym))


def _auto_menu_dep(node1, node2):
    # Returns True if node2 has an "automatic menu dependency" on node1. If
    # node2 has a prompt, we check its condition. Otherwise, we look directly
    # at node2.dep.

    return _expr_depends_on(node2.prompt[1] if node2.prompt else node2.dep, node1.item)


def _flatten(node):
    # "Flattens" menu nodes without prompts (e.g. 'if' nodes and non-visible
    # symbols with children from automatic menu creation) so that their
    # children appear after them instead. This gives a clean menu structure
    # with no unexpected "jumps" in the indentation.
    #
    # Do not flatten promptless choices (which can appear "legitimately" if a
    # named choice is defined in multiple locations to add on symbols). It
    # looks confusing, and the menuconfig already shows all choice symbols if
    # you enter the choice at some location with a prompt.

    while node:
        if node.list and not node.prompt and node.item.__class__ is not Choice:
            last_node = node.list
            while 1:
                last_node.parent = node.parent
                if not last_node.next:
                    break
                last_node = last_node.next

            last_node.next = node.next
            node.next = node.list
            node.list = None

        node = node.next


def _remove_ifs(node):
    # Removes 'if' nodes (which can be recognized by MenuNode.item being None),
    # which are assumed to already have been flattened. The C implementation
    # doesn't bother to do this, but we expose the menu tree directly, and it
    # makes it nicer to work with.

    cur = node.list
    while cur and not cur.item:
        cur = cur.next

    node.list = cur

    while cur:
        next = cur.next
        while next and not next.item:
            next = next.next

        # Equivalent to
        #
        #   cur.next = next
        #   cur = next
        #
        # due to tricky Python semantics. The order matters.
        cur.next = cur = next


def _finalize_choice(node):
    # Finalizes a choice, marking each symbol whose menu node has the choice as
    # the parent as a choice symbol, and automatically determining types if not
    # specified.

    choice = node.item

    cur = node.list
    while cur:
        if cur.item.__class__ is Symbol:
            cur.item.choice = choice
            choice.syms.append(cur.item)
        cur = cur.next

    # If no type is specified for the choice, its type is that of
    # the first choice item with a specified type
    if not choice.orig_type:
        for item in choice.syms:
            if item.orig_type:
                choice.orig_type = item.orig_type
                break

    # Each choice item of UNKNOWN type gets the type of the choice
    for sym in choice.syms:
        if not sym.orig_type:
            sym.orig_type = choice.orig_type


def _check_dep_loop_sym(sym, ignore_choice):
    # Detects dependency loops using depth-first search on the dependency graph
    # (which is calculated earlier in Kconfig._build_dep()).
    #
    # Algorithm:
    #
    #  1. Symbols/choices start out with _visited = 0, meaning unvisited.
    #
    #  2. When a symbol/choice is first visited, _visited is set to 1, meaning
    #     "visited, potentially part of a dependency loop". The recursive
    #     search then continues from the symbol/choice.
    #
    #  3. If we run into a symbol/choice X with _visited already set to 1,
    #     there's a dependency loop. The loop is found on the call stack by
    #     recording symbols while returning ("on the way back") until X is seen
    #     again.
    #
    #  4. Once a symbol/choice and all its dependencies (or dependents in this
    #     case) have been checked recursively without detecting any loops, its
    #     _visited is set to 2, meaning "visited, not part of a dependency
    #     loop".
    #
    #     This saves work if we run into the symbol/choice again in later calls
    #     to _check_dep_loop_sym(). We just return immediately.
    #
    # Choices complicate things, as every choice symbol depends on every other
    # choice symbol in a sense. When a choice is "entered" via a choice symbol
    # X, we visit all choice symbols from the choice except X, and prevent
    # immediately revisiting the choice with a flag (ignore_choice).
    #
    # Maybe there's a better way to handle this (different flags or the
    # like...)

    if not sym._visited:
        # sym._visited == 0, unvisited

        sym._visited = 1

        for dep in sym._dependents:
            # Choices show up in Symbol._dependents when the choice has the
            # symbol in a 'prompt' or 'default' condition (e.g.
            # 'default ... if SYM').
            #
            # Since we aren't entering the choice via a choice symbol, all
            # choice symbols need to be checked, hence the None.
            loop = _check_dep_loop_choice(dep, None) if dep.__class__ is Choice else _check_dep_loop_sym(dep, False)

            if loop:
                # Dependency loop found
                return _found_dep_loop(loop, sym)

        if sym.choice and not ignore_choice:
            loop = _check_dep_loop_choice(sym.choice, sym)
            if loop:
                # Dependency loop found
                return _found_dep_loop(loop, sym)

        # The symbol is not part of a dependency loop
        sym._visited = 2

        # No dependency loop found
        return None

    if sym._visited == 2:
        # The symbol was checked earlier and is already known to not be part of
        # a dependency loop
        return None

    # sym._visited == 1, found a dependency loop. Return the symbol as the
    # first element in it.
    return (sym,)


def _check_dep_loop_choice(choice, skip):
    if not choice._visited:
        # choice._visited == 0, unvisited

        choice._visited = 1

        # Check for loops involving choice symbols. If we came here via a
        # choice symbol, skip that one, as we'd get a false positive
        # '<sym FOO> -> <choice> -> <sym FOO>' loop otherwise.
        for sym in choice.syms:
            if sym is not skip:
                # Prevent the choice from being immediately re-entered via the
                # "is a choice symbol" path by passing True
                loop = _check_dep_loop_sym(sym, True)
                if loop:
                    # Dependency loop found
                    return _found_dep_loop(loop, choice)

        # The choice is not part of a dependency loop
        choice._visited = 2

        # No dependency loop found
        return None

    if choice._visited == 2:
        # The choice was checked earlier and is already known to not be part of
        # a dependency loop
        return None

    # choice._visited == 1, found a dependency loop. Return the choice as the
    # first element in it.
    return (choice,)


def _found_dep_loop(loop, cur):
    # Called "on the way back" when we know we have a loop

    # Is the symbol/choice 'cur' where the loop started?
    if cur is not loop[0]:
        # Nope, it's just a part of the loop
        return loop + (cur,)

    # Yep, we have the entire loop. Throw an exception that shows it.

    msg = "\nDependency loop\n===============\n\n"

    for item in loop:
        if item is not loop[0]:
            msg += "...depends on "
            if item.__class__ is Symbol and item.choice:
                msg += "the choice symbol "

        msg += f"{item.name_and_loc}, with definition...\n\n{item}\n\n"

        # Small wart: Since we reuse the already calculated
        # Symbol/Choice._dependents sets for recursive dependency detection, we
        # lose information on whether a dependency came from a 'select'/'imply'
        # condition or e.g. a 'depends on'.
        #
        # This might cause selecting symbols to "disappear". For example,
        # a symbol B having 'select A if C' gives a direct dependency from A to
        # C, since it corresponds to a reverse dependency of B && C.
        #
        # Always print reverse dependencies for symbols that have them to make
        # sure information isn't lost. I wonder if there's some neat way to
        # improve this.

        if item.__class__ is Symbol:
            if item.rev_dep is not item.kconfig.n:
                msg += f"(select-related dependencies: {expr_str(item.rev_dep)})\n\n"

            if item.weak_rev_dep is not item.kconfig.n:
                msg += f"(imply-related dependencies: {expr_str(item.rev_dep)})\n\n"

    msg += "...depends again on " + loop[0].name_and_loc

    raise KconfigError(msg)


def _decoding_error(e, filename, macro_linenr=None):
    # Gives the filename and context for UnicodeDecodeError's, which are a pain
    # to debug otherwise. 'e' is the UnicodeDecodeError object.
    #
    # If the decoding error is for the output of a $(shell,...) command,
    # macro_linenr holds the line number where it was run (the exact line
    # number isn't available for decoding errors in files).

    raise KconfigError(
        "\nMalformed {} in {}\nContext: {}\nProblematic data: {}\nReason: {}".format(
            e.encoding,
            f"'{filename}'" if macro_linenr is None else f"output from macro at {filename}:{macro_linenr}",
            e.object[max(e.start - 40, 0) : e.end + 40],
            e.object[e.start : e.end],
            e.reason,
        )
    )


def _warn_verbose_deprecated(fn_name):
    sys.stderr.write(
        f"Deprecation warning: {fn_name}()'s 'verbose' argument has no effect. Since "
        f"Kconfiglib 12.0.0, the message is returned from {fn_name}() instead, "
        f"and is always generated. Do e.g. print(kconf.{fn_name}()) if you want to "
        "want to show a message like \"Loaded configuration '.config'\" on "
        "stdout. The old API required ugly hacks to reuse messages in "
        "configuration interfaces.\n"
    )


# Predefined preprocessor functions


def _filename_fn(kconf, _):
    return kconf.filename


def _lineno_fn(kconf, _):
    return str(kconf.linenr)


def _info_fn(kconf, _, msg):
    print(f"{kconf.filename}:{kconf.linenr}: {msg}")

    return ""


def _warning_if_fn(kconf, _, cond, msg):
    if cond == "y":
        kconf._warn(msg, kconf.filename, kconf.linenr)

    return ""


def _error_if_fn(kconf, _, cond, msg):
    if cond == "y":
        raise KconfigError(f"{kconf.filename}:{kconf.linenr}: {msg}")

    return ""


def _shell_fn(kconf, _, command):
    import subprocess  # Only import as needed, to save some startup time

    stdout, stderr = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    try:
        stdout = stdout.decode(kconf._encoding)
        stderr = stderr.decode(kconf._encoding)
    except UnicodeDecodeError as e:
        _decoding_error(e, kconf.filename, kconf.linenr)

    if stderr:
        kconf._warn(
            "'{}' wrote to stderr: {}".format(command, "\\n".join(stderr.splitlines())),
            kconf.filename,
            kconf.linenr,
        )

    # Universal newlines with splitlines() (to prevent e.g. stray \r's in
    # command output on Windows), trailing newline removal, and
    # newline-to-space conversion.
    return "\n".join(stdout.splitlines()).rstrip("\n").replace("\n", " ")


#
# Global constants
#

BOOL_TO_STR = {
    0: "n",
    2: "y",
}

STR_TO_BOOL = {
    "n": 0,
    "y": 2,
}

# Constant representing that there's no cached choice selection. This is
# distinct from a cached None (no selection). Any object that's not None or a
# Symbol will do. We test this with 'is'.
_NO_CACHED_SELECTION = 0

try:
    _UNAME_RELEASE = os.uname()[2]
except AttributeError:
    # Only import as needed, to save some startup time
    import platform

    _UNAME_RELEASE = platform.uname()[2]

# The token and type constants below are safe to test with 'is', which is a bit
# faster (~30% faster on my machine, and a few % faster for total parsing
# time), even without assuming Python's small integer optimization (which
# caches small integer objects). The constants end up pointing to unique
# integer objects, and since we consistently refer to them via the names below,
# we always get the same object.
#
# Client code should use == though.

# Tokens, with values 1, 2, ... . Avoiding 0 simplifies some checks by making
# all tokens except empty strings truthy.
# NOTE: some tokens were removed, but the numbering to be preserved, some positions are blank (_)
(
    _T_ALLNOCONFIG_Y,
    _T_AND,
    _T_BOOL,
    _T_CHOICE,
    _T_CLOSE_PAREN,
    _T_COMMENT,
    _T_CONFIG,
    _T_DEFAULT,
    _T_DEFCONFIG_LIST,
    _,
    _,
    _,
    _,
    _,
    _T_DEPENDS,
    _T_ENDCHOICE,
    _T_ENDIF,
    _T_ENDMENU,
    _T_ENV,
    _T_EQUAL,
    _T_GREATER,
    _T_GREATER_EQUAL,
    _T_HELP,
    _T_HEX,
    _T_IF,
    _T_IMPLY,
    _T_INT,
    _T_LESS,
    _T_LESS_EQUAL,
    _T_MAINMENU,
    _T_MENU,
    _T_MENUCONFIG,
    _,
    _T_NOT,
    _T_ON,
    _T_OPEN_PAREN,
    _T_OPTION,
    _T_OPTIONAL,
    _T_OR,
    _T_ORSOURCE,
    _T_OSOURCE,
    _T_PROMPT,
    _T_RANGE,
    _T_RSOURCE,
    _T_SELECT,
    _T_SOURCE,
    _T_STRING,
    _,
    _T_UNEQUAL,
    _T_VISIBLE,
) = range(1, 51)

# Keyword to token map, with the get() method assigned directly as a small
# optimization
_get_keyword = {
    "---help---": _T_HELP,  # NOTE: deprecated; reason: ---help--- not supported in Kconfiglib_v2
    "allnoconfig_y": _T_ALLNOCONFIG_Y,
    "bool": _T_BOOL,
    "boolean": _T_BOOL,
    "choice": _T_CHOICE,
    "comment": _T_COMMENT,
    "config": _T_CONFIG,
    "default": _T_DEFAULT,
    "defconfig_list": _T_DEFCONFIG_LIST,
    "depends": _T_DEPENDS,
    "endchoice": _T_ENDCHOICE,
    "endif": _T_ENDIF,
    "endmenu": _T_ENDMENU,
    "env": _T_ENV,
    "grsource": _T_ORSOURCE,  # Backwards compatibility
    "gsource": _T_OSOURCE,  # Backwards compatibility
    "help": _T_HELP,
    "hex": _T_HEX,
    "if": _T_IF,
    "imply": _T_IMPLY,
    "int": _T_INT,
    "mainmenu": _T_MAINMENU,
    "menu": _T_MENU,
    "menuconfig": _T_MENUCONFIG,
    "on": _T_ON,
    "option": _T_OPTION,
    "optional": _T_OPTIONAL,
    "orsource": _T_ORSOURCE,
    "osource": _T_OSOURCE,
    "prompt": _T_PROMPT,
    "range": _T_RANGE,
    "rsource": _T_RSOURCE,
    "select": _T_SELECT,
    "source": _T_SOURCE,
    "string": _T_STRING,
    "visible": _T_VISIBLE,
}.get

# The constants below match the value of the corresponding tokens to remove the
# need for conversion

# Node types
MENU = _T_MENU
COMMENT = _T_COMMENT

# Expression types
AND = _T_AND
OR = _T_OR
NOT = _T_NOT
EQUAL = _T_EQUAL
UNEQUAL = _T_UNEQUAL
LESS = _T_LESS
LESS_EQUAL = _T_LESS_EQUAL
GREATER = _T_GREATER
GREATER_EQUAL = _T_GREATER_EQUAL

REL_TO_STR = {
    EQUAL: "=",
    UNEQUAL: "!=",
    LESS: "<",
    LESS_EQUAL: "<=",
    GREATER: ">",
    GREATER_EQUAL: ">=",
}

# Symbol/choice types. UNKNOWN is 0 (falsy) to simplify some checks.
# Client code shouldn't rely on it though, as it was non-zero in
# older versions.
UNKNOWN = 0
BOOL = _T_BOOL
STRING = _T_STRING
INT = _T_INT
HEX = _T_HEX

TYPE_TO_STR = {
    UNKNOWN: "unknown",
    BOOL: "bool",
    STRING: "string",
    INT: "int",
    HEX: "hex",
}

# Used in comparisons. 0 means the base is inferred from the format of the
# string.
_TYPE_TO_BASE = {
    HEX: 16,
    INT: 10,
    STRING: 0,
    UNKNOWN: 0,
}

# Tokens after which strings are expected. This is used to tell strings from
# constant symbol references during tokenization, both of which are enclosed in
# quotes.
#
# Identifier-like lexemes ("missing quotes") are also treated as strings after
# these tokens. _T_CHOICE is included to avoid symbols being registered for
# named choices.
_STRING_LEX = frozenset(
    {
        _T_BOOL,
        _T_CHOICE,
        _T_COMMENT,
        _T_HEX,
        _T_INT,
        _T_MAINMENU,
        _T_MENU,
        _T_ORSOURCE,
        _T_OSOURCE,
        _T_PROMPT,
        _T_RSOURCE,
        _T_SOURCE,
        _T_STRING,
    }
)

# Various sets for quick membership tests. Gives a single global lookup and
# avoids creating temporary dicts/tuples.

_TYPE_TOKENS = frozenset(
    {
        _T_BOOL,
        _T_INT,
        _T_HEX,
        _T_STRING,
    }
)

_SOURCE_TOKENS = frozenset(
    {
        _T_SOURCE,
        _T_RSOURCE,
        _T_OSOURCE,
        _T_ORSOURCE,
    }
)

_REL_SOURCE_TOKENS = frozenset(
    {
        _T_RSOURCE,
        _T_ORSOURCE,
    }
)

# Obligatory (non-optional) sources
_OBL_SOURCE_TOKENS = frozenset(
    {
        _T_SOURCE,
        _T_RSOURCE,
    }
)

_BOOL_UNKNOWN = frozenset(
    {
        BOOL,
        UNKNOWN,
    }
)

_INT_HEX = frozenset(
    {
        INT,
        HEX,
    }
)

_SYMBOL_CHOICE = frozenset(
    {
        Symbol,
        Choice,
    }
)

_MENU_COMMENT = frozenset(
    {
        MENU,
        COMMENT,
    }
)

_EQUAL_UNEQUAL = frozenset(
    {
        EQUAL,
        UNEQUAL,
    }
)

_RELATIONS = frozenset(
    {
        EQUAL,
        UNEQUAL,
        LESS,
        LESS_EQUAL,
        GREATER,
        GREATER_EQUAL,
    }
)


_KCONFIG_IGNORE_PRAGMA = "# ignore:"
_MULTIPLE_DEFINITION_LONG = "multiple-definition"
_MULTIPLE_DEFINITION_SHORT = "MD"
# Various regular expressions used during parsing

# The "# kconfig ignore: multiple-definitions" pragma.
_kconfig_ignore_match = re.compile(
    rf"^\s*(?P<option>config|choice)\s+([a-zA-Z0-9_]+)\s+{_KCONFIG_IGNORE_PRAGMA} (?P<type>{_MULTIPLE_DEFINITION_LONG}|{_MULTIPLE_DEFINITION_SHORT}).*"  # noqa: E501
).match

# The initial token on a line. Also eats leading and trailing whitespace, so
# that we can jump straight to the next token (or to the end of the line if
# there is only one token).
#
# This regex will also fail to match for empty lines and comment lines.
#
# '$' is included to detect preprocessor variable assignments with macro
# expansions in the left-hand side.
_command_match = re.compile(r"\s*([A-Za-z0-9_$-]+)\s*", re.ASCII).match

# An identifier/keyword after the first token. Also eats trailing whitespace.
# '$' is included to detect identifiers containing macro expansions.
_id_keyword_match = re.compile(r"([A-Za-z0-9_$/.-]+)\s*", re.ASCII).match

# A fragment in the left-hand side of a preprocessor variable assignment. These
# are the portions between macro expansions ($(foo)). Macros are supported in
# the LHS (variable name).
_assignment_lhs_fragment_match = re.compile("[A-Za-z0-9_-]*", re.ASCII).match

# The assignment operator and value (right-hand side) in a preprocessor
# variable assignment
_assignment_rhs_match = re.compile(r"\s*(=|:=|\+=)\s*(.*)", re.ASCII).match

# Special characters/strings while expanding a macro ('(', ')', ',', and '$(')
_macro_special_search = re.compile(r"\(|\)|,|\$\(", re.ASCII).search

# Special characters/strings while expanding a string (quotes, '\', and '$(')
_string_special_search = re.compile(r'"|\'|\\|\$\(', re.ASCII).search

# Special characters/strings while expanding a symbol name. Also includes
# end-of-line, in case the macro is the last thing on the line.
_name_special_search = re.compile(r"[^A-Za-z0-9_$/.-]|\$\(|$", re.ASCII).search

# A valid right-hand side for an assignment to a string symbol in a .config
# file, including escaped characters. Extracts the contents.
_conf_string_match = re.compile(r'"((?:[^\\"]|\\.)*)"', re.ASCII).match
