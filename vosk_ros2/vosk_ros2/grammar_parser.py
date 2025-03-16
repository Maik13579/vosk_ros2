import re
import sys

##############################################################################
# Exceptions
##############################################################################

class GrammarParseError(Exception):
    """Raised when there's an error (e.g. undefined reference, cycle detection)."""
    pass

##############################################################################
# Parse Tree Classes
##############################################################################

class GrammarNode:
    """Base class for parse tree nodes."""
    def expand(self, context):
        """Return a list of strings after expansion."""
        raise NotImplementedError()

class TextNode(GrammarNode):
    """Plain text or a single <tag> reference."""
    def __init__(self, text):
        self.text = text

    def expand(self, context):
        """
        If this is a reference <tag>, expand it using context.get_reference_expansion(tag).
        Otherwise, return [self.text].
        """
        if is_reference(self.text):
            return context.get_reference_expansion(self.text)
        else:
            return [self.text]

class OptionalNode(GrammarNode):
    """[ ... ] optional."""
    def __init__(self, child):
        self.child = child

    def expand(self, context):
        # Return expansions with and without the optional child
        without_child = ['']
        with_child = self.child.expand(context)
        # Merge them
        return without_child + with_child

class GroupNode(GrammarNode):
    """( ... ) group subexpression."""
    def __init__(self, child):
        self.child = child

    def expand(self, context):
        return self.child.expand(context)

class OrNode(GrammarNode):
    """Represents top-level OR: part1 | part2 | part3..."""
    def __init__(self, branches):
        self.branches = branches  # list of GrammarNode

    def expand(self, context):
        expansions = []
        for branch in self.branches:
            expansions.extend(branch.expand(context))
        return expansions

class SequenceNode(GrammarNode):
    """Sequence of tokens/nodes in a row."""
    def __init__(self, children):
        self.children = children  # list of GrammarNode

    def expand(self, context):
        if not self.children:
            return ['']
        # Expand children in a cartesian product style
        result = self.children[0].expand(context)
        for child in self.children[1:]:
            new_result = []
            right_side = child.expand(context)
            for left_part in result:
                for right_part in right_side:
                    combined = (left_part + ' ' + right_part).strip()
                    new_result.append(combined)
            result = new_result
        return result

##############################################################################
# Grammar Parsing
##############################################################################

REFERENCE_DEF_REGEX = re.compile(r'^(<[^>]+>)\s*:\s*(.+)$')

def is_reference(text):
    """Check if text is exactly <something> (for a reference)."""
    return text.startswith('<') and text.endswith('>')

class GrammarContext:
    """
    Holds:
      - reference_trees: dict mapping <tag> -> list of parse trees for each alternative
      - reference_expansions: dict mapping <tag> -> final list of expanded strings
      - grammar_trees: parse trees for the normal grammar lines
      - used_references: set of references actually used by normal lines
      - recursion_stack: cycle detection
    """
    def __init__(self):
        # <tag> -> [parse_tree, parse_tree, ...] for each alternative
        self.reference_trees = {}
        # <tag> -> [expanded strings] (final expansions after parse)
        self.reference_expansions = {}
        # parse trees for normal lines
        self.grammar_trees = []
        # references used in normal expansions
        self.used_references = set()
        # cycle detection
        self.recursion_stack = set()

    def get_reference_expansion(self, ref_tag):
        """
        Return the final expansions for a <tag>.
        If not computed yet, compute now by expanding the parse trees.
        If it's undefined, raise an error.
        If we detect recursion, raise an error.
        """
        if ref_tag not in self.reference_trees:
            raise GrammarParseError(f"Reference {ref_tag} used but not defined.")

        # Mark reference as used
        self.used_references.add(ref_tag)

        # If expansions are cached, return them
        if ref_tag in self.reference_expansions:
            return self.reference_expansions[ref_tag]

        # Cycle detection
        if ref_tag in self.recursion_stack:
            raise GrammarParseError(f"Cycle detected in references at {ref_tag}.")

        self.recursion_stack.add(ref_tag)

        # Expand all parse trees for this reference
        expansions = []
        for tree in self.reference_trees[ref_tag]:
            expansions.extend(tree.expand(self))

        # De-duplicate, preserve order
        expansions = list(dict.fromkeys(expansions))

        # Cache result
        self.reference_expansions[ref_tag] = expansions

        self.recursion_stack.remove(ref_tag)
        return expansions

def parse_grammar_lines(lines):
    """
    Reads all lines. Lines like "<tag>: expansions" are reference definitions.
    Others are grammar lines. Build parse trees for everything, then do expansions:
      1) Expand each reference into final strings
      2) Expand each grammar line into final strings
    Returns (normal_line_expansions, warnings_list).
    """
    context = GrammarContext()

    # 1) Collect definitions and grammar lines
    normal_lines = []
    for raw_line in lines:
        line = raw_line.strip()
        if not line:
            continue

        ref_match = REFERENCE_DEF_REGEX.match(line)
        if ref_match:
            # It's a reference definition
            ref_tag = ref_match.group(1).strip()   # e.g. <robot>
            expansion_part = ref_match.group(2).strip()
            # Split top-level OR to get separate alternatives
            alternatives = split_top_level_or(expansion_part)
            # Parse each alternative into a parse tree
            parse_trees = [parse_grammar_line(alt) for alt in alternatives]
            context.reference_trees[ref_tag] = parse_trees
        else:
            normal_lines.append(line)

    # Build parse trees for normal lines
    for line in normal_lines:
        tree = parse_grammar_line(line)
        context.grammar_trees.append(tree)

    # 2) Expand normal grammar lines. Each line can contain references.
    #    We'll compute expansions at final stage, references get expanded on-demand.
    normal_expansions = []
    for tree in context.grammar_trees:
        normal_expansions.extend(tree.expand(context))

    # De-duplicate normal expansions
    normal_expansions = list(dict.fromkeys(normal_expansions))

    # 3) Warn about references never used
    defined_refs = set(context.reference_trees.keys())
    used_refs = context.used_references
    unused_refs = defined_refs - used_refs
    warnings = []
    for ur in sorted(unused_refs):
        warnings.append(f"Warning: defined reference {ur} was never used.")

    return normal_expansions, warnings

##############################################################################
# Parsing Helpers
##############################################################################

def parse_grammar_line(line):
    """
    Parse a single grammar line into a parse tree.
    We handle top-level OR, then parse the sequence.
    """
    or_splits = split_top_level_or(line)
    if len(or_splits) > 1:
        branches = [parse_grammar_line(part) for part in or_splits]
        return OrNode(branches)
    else:
        return parse_sequence(line)

def split_top_level_or(line):
    """
    Splits on '|' not inside parentheses/brackets.
    """
    parts = []
    bracket_level = 0
    paren_level = 0
    start_idx = 0
    for i, ch in enumerate(line):
        if ch == '(':
            paren_level += 1
        elif ch == ')':
            paren_level -= 1
        elif ch == '[':
            bracket_level += 1
        elif ch == ']':
            bracket_level -= 1
        elif ch == '|' and paren_level == 0 and bracket_level == 0:
            parts.append(line[start_idx:i])
            start_idx = i + 1
    parts.append(line[start_idx:])
    return [p.strip() for p in parts if p.strip()]

def parse_sequence(line):
    """
    Parse a sequence with parentheses, optional brackets, references, etc.
    We tokenize first, then build a SequenceNode of children.
    """
    tokens = tokenize(line)
    nodes = []
    for token in tokens:
        if token.startswith('(') and token.endswith(')'):
            # Group
            inner = token[1:-1].strip()
            child = parse_grammar_line(inner)
            nodes.append(GroupNode(child))
        elif token.startswith('[') and token.endswith(']'):
            # Optional
            inner = token[1:-1].strip()
            child = parse_grammar_line(inner)
            nodes.append(OptionalNode(child))
        else:
            # Plain text or <tag>
            nodes.append(TextNode(token))
    return SequenceNode(nodes)

def tokenize(line):
    """
    Splits parentheses/brackets as single tokens.
    E.g. "Hello (morning | evening) [everyone]"
    -> ["Hello", "(morning | evening)", "[everyone]"]
    Then we split plain text by spaces if outside parentheses/brackets.
    """
    tokens = []
    current = []
    bracket_level = 0
    paren_level = 0
    for ch in line:
        if ch == '(':
            if bracket_level == 0 and paren_level == 0 and current:
                tokens.append(''.join(current).strip())
                current = []
            paren_level += 1
            current.append(ch)
        elif ch == ')':
            current.append(ch)
            paren_level -= 1
            if paren_level == 0 and bracket_level == 0:
                tokens.append(''.join(current).strip())
                current = []
        elif ch == '[':
            if bracket_level == 0 and paren_level == 0 and current:
                tokens.append(''.join(current).strip())
                current = []
            bracket_level += 1
            current.append(ch)
        elif ch == ']':
            current.append(ch)
            bracket_level -= 1
            if bracket_level == 0 and paren_level == 0:
                tokens.append(''.join(current).strip())
                current = []
        else:
            current.append(ch)

    leftover = ''.join(current).strip()
    if leftover:
        tokens.append(leftover)

    final_tokens = []
    for t in tokens:
        # If it's a parenthesized group or bracket group, keep as one piece
        if (t.startswith('(') and t.endswith(')')) or (t.startswith('[') and t.endswith(']')):
            final_tokens.append(t)
        else:
            # split by space
            final_tokens.extend(t.split())

    return [ft for ft in final_tokens if ft]

##############################################################################
# Example
##############################################################################

if __name__ == "__main__":
    grammar_lines = [
        # Reference definitions (any line matching <tag>: something)
        "<robot>: R2D2 | C3PO [hi]",
        "<question>: which <day> is it?",
        "<day>: Monday | Tuesday | Sunday",

        # Normal grammar lines (expanded fully):
        "Hello (morning | evening) [everyone]",
        "Bye | Farewell (friend | mate)",
        "I like (apples | bananas)",
        "My <robot> says [good] night",
        "Today is <day>",
        "Ask: <question>"
    ]

    try:
        expansions, warnings = parse_grammar_lines(grammar_lines)
        print("=== Expansions ===")
        for ex in expansions:
            print("  ", ex)

        if warnings:
            print("\n=== Warnings ===")
            for w in warnings:
                print("  ", w)

    except GrammarParseError as e:
        print("Grammar error:", e)
        sys.exit(1)
