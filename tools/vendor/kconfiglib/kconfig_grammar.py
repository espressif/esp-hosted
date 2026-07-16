# SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import re
from typing import TYPE_CHECKING
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from pyparsing import Forward
from pyparsing import Group
from pyparsing import IndentedBlock
from pyparsing import Keyword
from pyparsing import LineEnd
from pyparsing import Literal
from pyparsing import OneOrMore
from pyparsing import OpAssoc
from pyparsing import Opt
from pyparsing import ParseException
from pyparsing import PrecededBy
from pyparsing import QuotedString
from pyparsing import Regex
from pyparsing import Suppress
from pyparsing import Token
from pyparsing import Word
from pyparsing import ZeroOrMore
from pyparsing import alphanums
from pyparsing import infix_notation
from pyparsing import one_of

if TYPE_CHECKING:
    from kconfiglib.kconfig_parser import Parser


class KconfigBlock(Token):
    """
    Abstract class for custom Kconfig related pyparsiong blocks.
    """

    def __init__(self):
        super().__init__()

    def get_preceding_line(self, instring: str, loc: int) -> Optional[str]:
        if loc <= 0 or loc > len(instring):
            return None

        # Start looking backward from loc - 1 to find the previous newline
        pos = instring.rfind("\n", 0, loc - 1)
        # Now find the start of the line preceding the one we just found
        end_of_prev_line = pos
        pos = instring.rfind("\n", 0, pos - 1)

        start_of_prev_line = pos + 1

        # Return the preceding line
        return instring[start_of_prev_line : end_of_prev_line + 1]

    def leading_whitespace_len(self, line: str) -> int:
        return len(line) - len(line.lstrip())

    def first_non_empty_line_idx(self, lines: list, idx: int = 0) -> Optional[int]:
        # More than 90 % of the time, one of first two lines is not empty.
        # This will save 30 % of execution time for this function
        if lines[idx].strip():
            return idx
        if idx + 1 < len(lines) and lines[idx + 1].strip():
            return idx + 1

        for i, line in enumerate(lines[idx + 2 :]):
            if line.strip():
                return i + idx + 2
        return None


class KconfigHelpBlock(KconfigBlock):
    """
    This ParserElement is used to parse help blocks in Kconfig files.
    It turned out it is easier to do that directly than to use pyparsing's IndentedBlock or other approach.
    It is little strange to cast it into one token, but help block is just a plain text without any structure
    or meaning for the parser. Parsing it as a one token helps
    the parser not to search for possible matches in it.
    """

    def __init__(self):
        super().__init__()

    def _generateDefaultName(self) -> str:
        return "help_block"

    def parseImpl(self, instring: str, loc: int, doActions: bool = True) -> Tuple[int, List[str]]:
        result = []
        lines = instring[loc:].split("\n")

        # This is the indentation of the first line of the help block.
        # Every line with the same or bigger indentation (plus empty lines after which the same
        # indentation level continues) is considered to be part of the help block.
        help_keyword_indent = self.leading_whitespace_len(lines[1])

        # Preserve whitespaces cause that loc originally point to the \n char on line preceding the help keyword,
        # effectively creating ["", "<indent>help", <help body>] list.
        # We ignore the first two lines, as the first one is empty and the second one contains the help keyword.
        loc += len(lines[1]) + 1  # +1 for \n
        lines = lines[2:]

        idx = self.first_non_empty_line_idx(lines)
        if idx is None:
            raise ParseException(instring, loc, "Error parsing help block.", self)
        block_indent = self.leading_whitespace_len(lines[idx])
        if block_indent <= help_keyword_indent:
            raise ParseException(instring, loc, "Help block must be indented more than the help keyword.", self)

        line: str
        for i, line in enumerate(lines):
            # blank line:
            if not line or line.isspace():
                # lookahead if help block continues after the blank line(s)
                idx = self.first_non_empty_line_idx(lines, idx=i)
                if not idx:
                    break
                # if block continues, append blank line
                if self.leading_whitespace_len(lines[idx]) >= block_indent:
                    result.append(line)
                    loc += len(line) + 1  # +1 for \n
            # line of help block
            elif self.leading_whitespace_len(line) >= block_indent:
                result.append(line[block_indent:])  # cannot use strip as I want to preserve inner indentation
                loc += len(line) + 1  # +1 for \n
            else:
                break
            # end of help block
        # if some lines are overindented in the help, the overindentation is preserved in the help text itself
        return loc, result


# Complete regex combining all the parts:
symbol_regex = r"""(?<!\S)
                    (y|n|\"y\"|\"n\")(?!\S)  # y, n, "y", "n" symbols
                    |0[x|X][\da-fA-F]+  # hexnums: 0x1234, 0X1234ABCD
                    |-?[\d]+   # numbers: 1234, -1234
                    |[A-Z\d_]+  # variables: FOO, BAR_BAR, ENABLE_ESP64
                    |'[^']*'  # strings: 'here is a đĐđĐ[]tring'
                    |\"[^\"]*\" # strings: "hello world", ""
                    |\"\$\([A-Z_\d]+\)\" # "$(ENVVAR)"
                    |\"\$[A-Z_\d]+\""""  # "$ENVVAR"
symbol = Regex(symbol_regex, re.X)
operator_with_precedence = [
    (Literal("!"), 1, OpAssoc.RIGHT),
    (one_of("= != < > <= >="), 2, OpAssoc.LEFT),
    (one_of("&&"), 2, OpAssoc.LEFT),
    (one_of("||"), 2, OpAssoc.LEFT),
]

# Expression has operators above and symbols as operands
expression = infix_notation(symbol, operator_with_precedence)


class KconfigOptionBlock(KconfigBlock):
    """
    From the nature of pyparsing, if some ParserElement does not succeed,
    it raises an Exception and pyparsing backtracks (or raises the Exception signalizing unsuccessful parsing).
    However, this approach proved to be too slow for our needs.
    In order to speed up the parsing, option blocks are now parsed manually, which speeds up the parsing by up to 50 %.
    """

    def __init__(self):
        self.help_block = KconfigHelpBlock()
        self.entry_keywords = (
            "config",
            "menu",
            "endmenu",
            "choice",
            "endchoice",
            "source",
            "osource",
            "orsource",
            "rsource",
            "menuconfig",
            "if",
            "endif",
            "comment",
        )

        super().__init__()

    def _generateDefaultName(self) -> str:
        return "option_block"

    def parseImpl(self, instring: str, loc: int, doActions: bool = True) -> Tuple[int, dict]:
        """
        Overview:
        1) Go to first non-empty line after the loc
        2) Parse the option block line-by-line until the first token of the line is not recognized
           (i.e. it is not an option keyword)
           NOTE: Indentation is not controlled, because many third party components have very inconsistent indentation,
                 eg. lvgl
           NOTE: Help block is handled separately in KconfigHelpBlock
        3) Return the parsed dictionary with the options parsed
           NOTE: Semantic checks are not performed here, they are done in the Parser class
                 (e.g. "can this entry have this option?")
        """
        option_dict: Dict = {
            "type": None,
            "depends_on": [],
            "default": [],
            "range": [],
            "prompt": [],
            "select": [],
            "imply": [],
            "option": [],
            "visible_if": [],
            "help": None,
        }

        def is_line_with_option(tokens: List[str]) -> bool:
            if tokens[0].startswith(self.entry_keywords):
                return False
            return True

        def prompt_from_token_list(tokens: List[str]) -> Tuple[str, int]:
            """
            Get prompt (i.e. quoted string) from the list of tokens. Start and end are determined by the quotes.
            """
            prompt_tokens = []
            current_token_idx = 1
            well_formed_prompt = False
            for token in tokens:
                prompt_tokens.append(token)
                current_token_idx += 1
                if token.endswith(('"', "'")):
                    well_formed_prompt = True
                    break
            if not well_formed_prompt:
                raise ParseException(
                    instring, current_loc, "Error parsing option block: prompt must be a quoted string.", self
                )
            return " ".join(prompt_tokens)[1:-1], current_token_idx

        # Unfortunately, pyparsing sometimes points KconfigOptionBlock to the end of the previous line,
        # sometimes directly to the start of current line,
        # sometimes to the start of the actual text on the current line, depending on what is parsed before this block.
        # This is caused by pyparsing's whitespace handling and cannot be mitigated in it.
        # The parsing algorithm here supposes loc point to the end of previous line.
        # This helps to handle the loc in a unified manner.
        while instring[loc] != "\n":
            loc -= 1

        lines = instring[loc + 1 :].split("\n")
        current_loc = loc

        help_text_indices: List[int] = []
        for idx, line in enumerate(lines):
            ############################################
            # Skipping lines
            ############################################
            # Already parsed help line
            if idx in help_text_indices:
                help_text_indices.remove(idx)
                continue

            # Blank line inside option block, there may be something after it
            if not line.strip():
                current_loc += len(line) + 1  # +1 for \n
                continue

            tokens = line.strip().split()

            # If the line does not contain any option keyword, the option block ends
            if not is_line_with_option(tokens):
                break

            ############################################
            # Parsing the option block
            ############################################
            # parse type
            if tokens[0] in ("bool", "int", "string", "hex"):
                option_dict["type"] = tokens[0]

                if len(tokens) > 1:  # inline prompt
                    if not tokens[1].startswith(('"', "'")):
                        raise ParseException(
                            instring, current_loc, "Error parsing option block: prompt must be a quoted string.", self
                        )

                    prompt_str, current_token_idx = prompt_from_token_list(tokens[1:])

                    cond = None
                    if len(tokens) > current_token_idx and tokens[current_token_idx] == "if":  # inline condition
                        cond = expression.parse_string(
                            " ".join(tokens[current_token_idx + 1 :]), parse_all=True
                        ).as_list()

                    option_dict["prompt"].append((prompt_str, cond))
                current_loc += len(line) + 1  # +1 for \n

            # parse default
            elif tokens[0] == "default":
                if "if" in tokens:
                    option_dict["default"].append(
                        (
                            expression.parse_string(" ".join(tokens[1 : tokens.index("if")]), parse_all=True).as_list(),
                            expression.parse_string(
                                " ".join(tokens[tokens.index("if") + 1 :]), parse_all=True
                            ).as_list(),
                        )
                    )
                else:
                    option_dict["default"].append(
                        (expression.parse_string(" ".join(tokens[1:]), parse_all=True).as_list(), None)
                    )

                current_loc += len(line) + 1  # +1 for \n

            elif tokens[0] == "help":
                new_loc, parsed_help = self.help_block.parseImpl(instring, current_loc)
                option_dict["help"] = "\n".join(parsed_help)
                current_loc = new_loc
                help_text_indices = [i for i in range(idx + 1, idx + 1 + len(parsed_help))]

            elif tokens[0] == "depends":
                if not tokens[1] == "on":
                    raise ParseException(
                        instring,
                        current_loc,
                        'Error parsing option block: "depends" must be followed by "on" keyword.',
                        self,
                    )
                else:
                    expr = expression.parse_string(" ".join(tokens[2:]), parse_all=True).as_list()
                    option_dict["depends_on"].append(expr)
                    current_loc += len(line) + 1  # +1 for \n

            elif tokens[0] == "range":
                symbol1 = symbol.parse_string(tokens[1], parse_all=True).as_list()[0]
                symbol2 = symbol.parse_string(tokens[2], parse_all=True).as_list()[0]
                cond = None
                if len(tokens) > 3:
                    if not tokens[3] == "if":
                        raise ParseException(
                            instring,
                            current_loc,
                            "Error parsing option block: extra tokens after range sym1 sym2.",
                            self,
                        )
                    cond = expression.parse_string(" ".join(tokens[4:]), parse_all=True).as_list()
                option_dict["range"].append((symbol1, symbol2, cond))
                current_loc += len(line) + 1  # +1 for \n

            elif tokens[0] == "prompt":
                if not tokens[1].startswith(('"', "'")):
                    raise ParseException(
                        instring, current_loc, "Error parsing option block: prompt must be a quoted string.", self
                    )
                prompt_str, current_token_idx = prompt_from_token_list(tokens[1:])

                cond = None
                if len(tokens) > current_token_idx and tokens[current_token_idx] == "if":  # inline condition
                    cond = expression.parse_string(" ".join(tokens[current_token_idx + 1 :]), parse_all=True).as_list()
                option_dict["prompt"].append((prompt_str, cond))
                current_loc += len(line) + 1  # +1 for \n

            elif tokens[0] == "select" or tokens[0] == "imply":
                sym = symbol.parse_string(tokens[1], parse_all=True).as_list()[0]
                if len(tokens) > 2:
                    if not tokens[2] == "if":
                        raise ParseException(
                            instring,
                            current_loc,
                            f"Error parsing option block: extra tokens after {tokens[0]} option.",
                            self,
                        )
                    cond = expression.parse_string(" ".join(tokens[3:]), parse_all=True).as_list()
                    option_dict[tokens[0]].append((sym, cond))
                else:
                    option_dict[tokens[0]].append((sym, None))
                current_loc += len(line) + 1  # +1 for \n

            elif tokens[0] == "visible":
                if not tokens[1] == "if":
                    raise ParseException(
                        instring,
                        current_loc,
                        "Error parsing option block: visible if must be followed by if keyword.",
                        self,
                    )
                else:
                    expr = expression.parse_string(" ".join(tokens[2:]), parse_all=True).as_list()
                    option_dict["visible_if"].append(expr)
                    current_loc += len(line) + 1  # +1 for \n

            elif tokens[0] == "option":
                if not tokens[1].startswith("env="):
                    raise ParseException(
                        instring,
                        current_loc,
                        'Error parsing option block: option must be in a form "option env=<env_var>".',
                        self,
                    )
                option = tokens[1][5:-1]
                # Converting the option to default with the same semantics
                option_dict["option"].append(option)
                current_loc += len(line) + 1  # +1 for \n

            else:
                raise ParseException(
                    instring, current_loc, f"Error parsing option block: unsupported option at line {line}.", self
                )

        return current_loc, option_dict


class KconfigGrammar:
    """
    Grammar of the Kconfig language.
    Parse actions are defined in the Parser class -> if another format will be used,
    it is enough to change the Parser class.
    """

    def __init__(self, parser: "Parser") -> None:
        self.init_grammar(parser)

    def init_grammar(self, parser: "Parser") -> None:
        """
        Initializes the grammar for the Kconfig language.

        The pyparsing package embraces "bottom-up" approach both in the parsing and grammar definition.
        Thus, the logic behind the definitions is that first, nested elements are defined (e.g. config options)
        and only then the wrapping element (e.g. config itself) is defined.

        More info is directly in the code below.
        """
        self.parser = parser

        condition = Suppress(Keyword("if")) + Group(expression)
        # By default, pyparsing removes all the leading and trailing whitespaces
        # and thus these two constructs appear the same:
        #    ...
        #    default y
        #
        # if SOME_VAR...
        #
        # and
        #
        # default y if SOME_VAR
        #
        # Here, we are forcing that there cannot be a line break between "default" and "if"
        # to correctly parse inline condition (second case in the example above).
        inline_condition = ~PrecededBy(LineEnd()) + condition

        ##########################
        # Options
        #
        # They are mostly for configs & choices, but:
        # - "depends on" can be used for menus, comments
        # - "visible if" can be used for menus
        ##########################

        # Prompt is a string (with optional condition) defined either with "prompt" keyword or implicitly
        # (without a keyword) after a type definition
        # Every config/choice can have max. one prompt which is used to show to the user.
        # Optionally, it can be conditioned.
        # Explicit inline prompt parsing occurs because in some cases, inline prompt is not part of an option block.
        inline_prompt = (QuotedString('"') | QuotedString("'")) + Opt(inline_condition)

        ###########################
        # Config
        ###########################
        # List of all possible options for the config/choice.

        config_name = Word(alphanums.upper() + "_").set_results_name("config_name", list_all_matches=True)
        config_opts = KconfigOptionBlock().leave_whitespace().set_results_name("config_opts")
        config = Keyword("config") + config_name + config_opts
        config = config.set_parse_action(parser.parse_config)

        ###########################
        # Comment
        ###########################
        # Not a #-like comment (which is ignored), but a comment block showed in generated sdkconfig file.
        comment_opts = KconfigOptionBlock().leave_whitespace().set_results_name("comment_opts")

        comment = Keyword("comment") + inline_prompt + Opt(comment_opts)
        comment = comment.set_parse_action(parser.parse_comment)

        ###########################
        # Source
        ###########################
        # [o|r|or]source point to a file that should be placed in the place of source.
        # Details about the source are in the documentation.
        # parsing info is in the __call__ method of the KconfigGrammar class
        # (TLDR: recursively, new KconfigGrammar class is created and sourced_root is parsed).
        source_type = one_of(["orsource", "source", "rsource", "osource"], as_keyword=True)
        path = QuotedString('"').set_results_name("path")
        source = (source_type + path).set_parse_action(parser.parse_sourced)

        ########################
        # Choice
        ########################
        choice_if_entry = Forward()
        choice_if_entry << Keyword("if") + (expression + IndentedBlock(choice_if_entry | config)).set_parse_action(
            parser.parse_if_entry
        ) + Keyword("endif")

        # Choice is a group of configs that can have only one active at a time.
        choice = (
            Keyword("choice")
            + (
                Opt(symbol).set_results_name("name")
                + Opt(KconfigOptionBlock().set_results_name("config_opts"))
                + IndentedBlock(config | choice_if_entry).set_results_name("configs")
            ).set_parse_action(parser.parse_choice)
            + Keyword("endchoice")
        )

        ########################
        # Menuconfig
        ########################
        menuconfig = (Keyword("menuconfig") + config_name + config_opts).set_parse_action(parser.parse_config)

        ########################
        # Menu
        ########################
        menu = Forward()  # Forward declaration is needed because of recursive structure of the menu.
        # if_entry is a recursive element (if_entry can contain menu, menu can contain if_entry).
        # So this is just a declaration of the element, definition comes later.
        if_entry = Forward()  # Forward declaration is needed because of recursive structure of the menu.

        # List of all possible entries in the menu/if block.
        # Every entry should have the same indentation and optionally, an empty line in between two entries.
        # But e.g. lvgl does not follow any rules in their Kconfig files and thus,
        # formal specifications needs to be loosen.
        entries = ZeroOrMore(config | menu | choice | source | menuconfig | if_entry | comment).set_results_name(
            "entries"
        )

        menu << (
            Keyword("menu")
            + QuotedString('"')
            + Opt(KconfigOptionBlock().leave_whitespace())
            + entries
            + Keyword("endmenu")
        ).set_parse_action(parser.parse_menu).set_results_name("menu")

        ###########################
        # If entry
        ###########################
        # Not "if" as a condition, but "if" as a separate block
        if_entry << Keyword("if") + (expression + entries).set_parse_action(parser.parse_if_entry) + Keyword("endif")

        ###########################
        # Main menu
        ###########################
        mainmenu = (Keyword("mainmenu") + (QuotedString('"') | QuotedString("'")) + entries).set_parse_action(
            parser.parse_mainmenu
        )

        ############################
        # Entry points
        ############################
        # The root of the grammar. It is used to parse the main Kconfig file,
        # which needs to contain mainmenu as a top element.
        self.root = mainmenu

        # sourced file can have different structure than the main Kconfig file, thus using a separate root.
        sourced_root = OneOrMore(config | menu | choice | source | menuconfig | if_entry | comment)
        self.sourced_root = sourced_root

    def preprocess_file(self, file: str, ensure_end_newline: bool = True) -> str:
        """
        Helper method for preprocessing the Kconfig files.
        Unfortunately, some Kconfig syntax would be so difficult to support in pyparsing
        that it is easier to preprocess the files.
        Specifically, it:
            * merges lines split with '\'
            * removes inline comments
        """

        def remove_inline_comments(line: str) -> str:
            """
            Removes inline comments from the string, preserving # inside quotes.
            """
            self.parser.kconfig.check_pragmas(line)

            quote = None  # Tracks if we're inside a quote
            result = []

            for char in line:
                if quote:
                    # Close the quote if we encounter a matching quote character
                    if char == quote:
                        quote = None
                    result.append(char)
                elif char in {'"', "'"}:
                    # Start a new quote if we encounter a quote character
                    quote = char
                    result.append(char)
                elif char == "#":
                    # Stop processing if we encounter a `#` **outside** quotes
                    break
                else:
                    result.append(char)

            # Join the result and check if the original line ended with a newline
            cleaned_line = "".join(result).rstrip()
            return cleaned_line + "\n"

        with open(file, "r") as f:
            lines = f.readlines()
            return_file = ""
            split_lines_idxs: List[int] = []

            for line_idx, line in enumerate(lines):
                line = line.replace("\t", "    ")  # Replace tabs with 4 spaces
                # Remove unnecessary whitespaces from otherwise empty line
                if line.isspace():
                    return_file += "\n"
                    continue

                # Remove inline comments
                line = remove_inline_comments(line)

                # Merge lines split with '\' and place blank lines to preserve line numbering.
                if line_idx in split_lines_idxs:
                    split_lines_idxs.remove(line_idx)
                    return_file += "\n"  # In order to match the original numbering, blank lines are added.
                    continue
                if line.endswith("\\\n"):
                    merged_line = line.rstrip("\\\n")
                    for next_line_idx, next_line in enumerate(lines[line_idx + 1 :]):
                        if next_line.endswith("\\\n"):
                            merged_line += remove_inline_comments(next_line).lstrip(" ").rstrip("\\\n")
                            split_lines_idxs.append(next_line_idx + line_idx + 1)
                        else:  # first line without '\' is still part of the merged line
                            merged_line += " " + remove_inline_comments(next_line).lstrip(" ")
                            split_lines_idxs.append(next_line_idx + line_idx + 1)
                            break
                    return_file += merged_line
                    continue

                return_file += line

        return return_file if not ensure_end_newline else return_file + "\n"

    def __call__(self, file: str, sourced: bool = False):
        """
        Here, pyparsing parsing takes place.
        Parameters:
        * file: str - path to the Kconfig file
        * sourced: bool - flag indicating if the file is from [o|r|or]source entry
        """
        file_content = self.preprocess_file(file)
        # If the file is empty, we have nothing to do
        if not file_content or file_content.isspace():
            return ""

        if not sourced:
            try:
                output = self.root.parse_string(file_content, parse_all=True)
            except ParseException as e:
                raise KconfigParseError(f"Error parsing file {file}: {str(e)}")
        else:
            try:
                output = self.sourced_root.parse_string(file_content, parse_all=True)
            except ParseException as e:
                raise KconfigParseError(f"Error parsing sourced file {file}: {str(e)}")
        return output


class KconfigParseError(Exception):
    """
    Exception raised when parsing of the Kconfig file fails.
    """

    pass
