from __future__ import print_function

import itertools
import os
import xml.etree.ElementTree as ET
import xml.dom.minidom
import numbers

from tf.transformations import *


def prettyXML(uglyXML):
  return xml.dom.minidom.parseString(uglyXML).toprettyxml(indent='  ')


def homogeneous2translation_quaternion(homogeneous):
  """
  Translation: [x, y, z]
  Quaternion: [x, y, z, w]
  """
  translation = translation_from_matrix(homogeneous)
  quaternion = quaternion_from_matrix(homogeneous)
  return translation, quaternion


def rounded(val):
  if isinstance(val, numbers.Number):
    return int(round(val,6) * 1e5) / 1.0e5
  else:
    return numpy.array([rounded(v) for v in val])


def homogeneous2tq_string(homogeneous):
  return 't=%s q=%s' % homogeneous2translation_quaternion(homogeneous)


def homogeneous2tq_string_rounded(homogeneous):
  return 't=%s q=%s' % tuple(rounded(o) for o in homogeneous2translation_quaternion(homogeneous))


def get_tag(node, tagname, default):
  tag = node.findall(tagname)
  if tag:
    return tag[0].text
  else:
    return default


def string2float_list(s):
  return [float(i) for i in s.split()]


def pose_string2homogeneous(pose):
  pose_float = string2float_list(pose)
  translate = pose_float[:3]
  angles = pose_float[3:]
  homogeneous = compose_matrix(None, None, angles, translate)
  return homogeneous


def get_tag_pose(node):
  pose = get_tag(node, 'pose', '0 0 0  0 0 0')
  return pose_string2homogeneous(pose)






class SDF(object):
  def __init__(self, **kwargs):
    self.world = World()
    if 'file' in kwargs:
      self.from_file(kwargs['file'])


  def from_file(self, filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    if root.tag != 'sdf':
      print('Not a SDF file. Aborting.')
      return
    self.version = float(root.attrib['version'])
    self.world.from_tree(root)



class World(object):
  def __init__(self):
    self.name = '__default__'
    self.models = []
    self.lights = []


  def from_tree(self, node):
    if node.findall('world'):
      node = node.findall('world')[0]
      # TODO lights
    self.models = [Model(tree=model_node) for model_node in node.findall('model')]




class SpatialEntity(object):
  def __init__(self):
    self.name = ''
    self.pose = identity_matrix()


  def __repr__(self):
    return ''.join((
      'name: %s\n' % self.name,
      'pose: %s\n' % homogeneous2tq_string_rounded(self.pose),
    ))


  def from_tree(self, node):
    self.name = node.attrib['name']
    self.pose = get_tag_pose(node)



class Model(SpatialEntity):
  def __init__(self, **kwargs):
    super(Model, self).__init__()
    self.submodels = []
    self.links = []
    self.joints = []
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Model(\n', 
      '  %s\n' % super(Model, self).__repr__().replace('\n', '\n  ').strip(),
      '  links:\n',
      '    %s' % '\n    '.join([str(l).replace('\n', '\n    ').strip() for l in self.links]),
      '\n',
      '  joints:\n',
      '    %s' % '\n    '.join([str(j).replace('\n', '\n    ').strip() for j in self.joints]),
      '\n',
      ')'
    ))


  def from_tree(self, node):
    if node.tag != 'model':
      print('Invalid node of type %s instead of model. Aborting.' % node.tag)
      return
    super(Model, self).from_tree(node)
    self.links = [Link(self, tree=link_node) for link_node in node.iter('link')]
    self.joints = [Joint(self, tree=joint_node) for joint_node in node.iter('joint')]



  def to_urdf(self):
    # TODO
    return ''


  def save_urdf(self, filename):
    urdf_file = open(filename, 'w')
    pretty_urdf_string = prettyXML(self.to_urdf())
    urdf_file.write(pretty_urdf_string)
    urdf_file.close()



class Link(SpatialEntity):
  def __init__(self, parent_model, **kwargs):
    super(Link, self).__init__()
    self.parent_model = parent_model
    self.gravity = True
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Link(\n',
      '  %s\n' % super(Link, self).__repr__().replace('\n', '\n  ').strip(),
      ')'
    ))


  def from_tree(self, node):
    if node.tag != 'link':
      print('Invalid node of type %s instead of link. Aborting.' % node.tag)
      return
    super(Link, self).from_tree(node)
    #TODO-next



class Joint(SpatialEntity):
  def __init__(self, parent_model, **kwargs):
    super(Joint, self).__init__()
    self.parent_model = parent_model
    self.parent = ''
    self.child = ''
    self.axis = Axis()
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Joint(\n',
      '  %s\n' % super(Joint, self).__repr__().replace('\n', '\n  ').strip(),
      ')'
    ))


  def from_tree(self, node):
    if node.tag != 'joint':
      print('Invalid node of type %s instead of joint. Aborting.' % node.tag)
      return
    super(Joint, self).from_tree(node)
    #TODO self.pose = numpy.dot(self.child.pose, self.pose)
    # TODO



class Axis(object):
  def __init__(self):
    self.xyz = numpy.array([0, 0, 0])
