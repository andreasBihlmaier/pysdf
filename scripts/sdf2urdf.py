#!/usr/bin/env python

import sys
import argparse

import pysdf


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('sdf', help='SDF file to convert')
  parser.add_argument('urdf', help='Resulting URDF file to be written')
  args = parser.parse_args()

  sdf = pysdf.SDF(file=args.sdf)
  world = sdf.world
  if len(world.models) != 1:
    print('SDF contains %s instead of exactly one model. Aborting.' % len(world.models))
    sys.exit(1)

  model = world.models[0]
  print(model)
  #model.save_urdf(args.urdf)


if __name__ == '__main__':
  main()
