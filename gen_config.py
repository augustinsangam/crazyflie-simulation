#!/usr/bin/env python3

import pathlib
import random
import sys

arena_index = 0 if (
    len(sys.argv) < 2 or sys.argv[1] == '') else int(sys.argv[1])

if arena_index not in [1, 2, 3, 4, 5]:
    arena_index = 0

parent = pathlib.Path(__file__).parent
arena_dir = parent / 'arenas'
balise = '<!-- arena_config -->\n'

with (parent / 'config.xml.in').open('r') as config_in_file:
    config_in_xml = config_in_file.read()

with (arena_dir / f'arena{arena_index}.xml').open('r') as arena_file:
    arena_xml = arena_file.read()

with pathlib.Path('config.xml').open('w') as config_file:
    config_in_xml = config_in_xml.replace(
        '@SEED@', str(random.randrange(1, 65000)))
    config_splitted = config_in_xml.split(balise)
    config_splitted[1] = arena_xml
    config_file.seek(0)
    config_file.write(balise.join(config_splitted))
    config_file.truncate()

print(f'[arena_selector] arena{arena_index} loaded.')
