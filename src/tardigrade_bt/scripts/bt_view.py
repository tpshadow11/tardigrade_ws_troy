#!/usr/bin/env python3
"""Show a BehaviorTree.CPP XML tree without Groot2 (works on a Mac).

    python3 bt_view.py trees/playground.xml               # tree in the terminal
    python3 bt_view.py trees/playground.xml --mermaid out.md   # diagram

The Mermaid file renders as a diagram on GitHub, and in VS Code with the
"Markdown Preview Mermaid Support" extension (open it, then Cmd+Shift+V).
Only the Python standard library is used.
"""

import argparse
import sys
import xml.etree.ElementTree as ET

CONTROL = {
    'Sequence', 'ReactiveSequence', 'SequenceWithMemory', 'Fallback',
    'ReactiveFallback', 'Parallel', 'ParallelAll', 'IfThenElse',
    'WhileDoElse', 'Switch2', 'Switch3', 'Switch4',
}
DECORATOR = {
    'Timeout', 'Delay', 'Inverter', 'ForceSuccess', 'ForceFailure',
    'Repeat', 'RetryUntilSuccessful', 'KeepRunningUntilFailure',
    'RunOnce', 'Precondition',
}
CONDITION = {'OdometryFresh', 'IsArmed'}
SHAPE = {
    'control': ('[', ']'),
    'decorator': ('{{', '}}'),
    'condition': ('([', '])'),
    'action': ('[/', '/]'),
}


def kind(tag):
    if tag in CONTROL:
        return 'control'
    if tag in DECORATOR:
        return 'decorator'
    if tag in CONDITION or tag.startswith('Is') or tag.endswith('?'):
        return 'condition'
    return 'action'


def label(element):
    name = element.get('name')
    args = ' '.join(
        f'{key}={value}' for key, value in element.attrib.items()
        if key not in ('name', 'ID'))
    text = element.tag
    if name:
        text = f'{element.tag} "{name}"'
    return f'{text} ({args})' if args else text


def load(path):
    root = ET.parse(path).getroot()
    trees = {tree.get('ID'): tree for tree in root.findall('BehaviorTree')}
    main = root.get('main_tree_to_execute') or next(iter(trees))
    return trees, main


def children(element, trees):
    if element.tag == 'SubTree' and element.get('ID') in trees:
        return list(trees[element.get('ID')])
    return list(element)


def print_tree(element, trees, prefix='', last=True, top=True):
    marker = '' if top else ('└── ' if last else '├── ')
    print(prefix + marker + label(element))
    kids = children(element, trees)
    for index, child in enumerate(kids):
        extension = '' if top else ('    ' if last else '│   ')
        print_tree(child, trees, prefix + extension, index == len(kids) - 1, False)


def mermaid(element, trees, lines, counter):
    node_id = f'n{counter[0]}'
    counter[0] += 1
    left, right = SHAPE[kind(element.tag)]
    text = label(element).replace('"', "'")
    lines.append(f'    {node_id}{left}"{text}"{right}')
    for child in children(element, trees):
        child_id = mermaid(child, trees, lines, counter)
        lines.append(f'    {node_id} --> {child_id}')
    return node_id


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('tree_file')
    parser.add_argument('--mermaid', metavar='OUT_MD',
                        help='also write a Mermaid diagram to this .md file')
    args = parser.parse_args()

    trees, main_id = load(args.tree_file)
    if main_id not in trees:
        sys.exit(f'No BehaviorTree with ID {main_id!r} in {args.tree_file}')
    body = list(trees[main_id])
    print(f'Tree: {main_id}')
    for index, element in enumerate(body):
        print_tree(element, trees, '', index == len(body) - 1, True)

    if args.mermaid:
        lines = ['```mermaid', 'flowchart TD']
        counter = [0]
        for element in body:
            mermaid(element, trees, lines, counter)
        lines.append('```')
        legend = ('Shapes: [box] = control (Sequence/Fallback), {{hexagon}} = '
                  'decorator (Timeout), ([rounded]) = condition, '
                  '[/slanted/] = action.')
        with open(args.mermaid, 'w', encoding='utf-8') as stream:
            stream.write(f'# {main_id}\n\n' + '\n'.join(lines) + f'\n\n{legend}\n')
        print(f'\nWrote diagram to {args.mermaid}')


if __name__ == '__main__':
    main()
