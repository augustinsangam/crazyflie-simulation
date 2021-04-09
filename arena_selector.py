#!/usr/bin/python

from argparse import ArgumentParser
from random import randint

parser = ArgumentParser()
parser.add_argument("arena_index", type=int, default=-1)

arena_index = parser.parse_args().arena_index

if arena_index not in [1, 2, 3, 4, 5]:
    '''
if arena_index == -1:
    print("[arena_selector] Using a random arena.")
else:
    print(
        f"[arena_selector] arena {arena_index} does not exist, defaulting to a random one.")
arena_index = randint(1, 5)
    '''
    exit(0)

config_path = "./config.xml"
arena_path = "./arenas/"
balise = "<!-- arena_config -->\n"

with open(arena_path + f"arena{arena_index}.xml", 'r') as arena_file:
    arena_xml = arena_file.read()

with open(config_path, 'r+') as config_file:
    config_xml = config_file.read()
    config_splitted = config_xml.split(balise)
    config_splitted[1] = arena_xml
    config_file.seek(0)
    config_file.write(balise.join(config_splitted))
    config_file.truncate()

print(f"[arena_selector] arena{arena_index} loaded.")
