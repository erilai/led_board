"""Minimal S-expression reader/writer for KiCad files.

Parses into nested lists of str/Node. Round-trips text faithfully enough for
KiCad to reopen the file (KiCad re-formats on its next save anyway).
"""

from __future__ import annotations


class Node(list):
    """An S-expression list. Node[0] is the tag; the rest are children."""

    @property
    def tag(self):
        return self[0] if self else None

    def find_all(self, tag):
        return [c for c in self[1:] if isinstance(c, Node) and c.tag == tag]

    def find(self, tag):
        for c in self[1:]:
            if isinstance(c, Node) and c.tag == tag:
                return c
        return None

    def value(self, tag, default=None):
        """First scalar argument of the first child with `tag`."""
        n = self.find(tag)
        if n is None or len(n) < 2:
            return default
        return n[1]


def parse(text: str) -> Node:
    i, n = 0, len(text)
    stack: list[Node] = []
    root = None
    while i < n:
        c = text[i]
        if c.isspace():
            i += 1
        elif c == "(":
            node = Node()
            if stack:
                stack[-1].append(node)
            else:
                root = node
            stack.append(node)
            i += 1
        elif c == ")":
            stack.pop()
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j : j + 2])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            stack[-1].append(Quoted("".join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in "()":
                j += 1
            stack[-1].append(text[i:j])
            i = j
    if root is None:
        raise ValueError("no s-expression found")
    return root


class Quoted(str):
    """A string that was quoted in the source, so it stays quoted on write."""


def _atom(x) -> str:
    if isinstance(x, Quoted):
        return '"' + x + '"'
    return str(x)


def dumps(node, indent: int = 0) -> str:
    pad = "\t" * indent
    parts = [pad + "(" + _atom(node[0])]
    inline = all(not isinstance(c, Node) for c in node[1:])
    if inline:
        for c in node[1:]:
            parts.append(" " + _atom(c))
        parts.append(")")
        return "".join(parts)
    parts[0] += "\n"
    for c in node[1:]:
        if isinstance(c, Node):
            parts.append(dumps(c, indent + 1) + "\n")
        else:
            parts.append("\t" * (indent + 1) + _atom(c) + "\n")
    parts.append(pad + ")")
    return "".join(parts)


def load(path) -> Node:
    with open(path, "r", encoding="utf-8") as f:
        return parse(f.read())


def save(node: Node, path) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(dumps(node) + "\n")
