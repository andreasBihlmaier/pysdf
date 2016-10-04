#!/usr/bin/env python

import sys
import argparse

import pysdf


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('sdf', help='SDF file to convert')
  parser.add_argument('urdf', help='Resulting URDF file to be written')
  parser.add_argument('-p', '--plot', nargs=1, help='Plot SDF to file')
  parser.add_argument('--no-prefix', action='store_true', help='Do not use model name as name prefix')
  args = parser.parse_args()

  sdf = pysdf.SDF(file=args.sdf)
  world = sdf.world
  if args.plot:
    world.plot_to_file(args.plot[0])
  if len(world.models) != 1:
    print('SDF contains %s instead of exactly one model. Aborting.' % len(world.models))
    sys.exit(1)

  model = world.models[0]
  print(model)
  model.save_urdf(args.urdf, prefix=None if args.no_prefix else '')


if __name__ == '__main__':
  main()
