from __future__ import print_function

import itertools
import os
import xml.etree.ElementTree as ET
import xml.dom.minidom
import numbers

from tf.transformations import *

models_path = os.path.expanduser('~/.gazebo/models/')

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


def get_tag(node, tagname, default = None):
  tag = node.findall(tagname)
  if tag:
    return tag[0].text
  else:
    return default

def get_node(node, tagname, default = None):
  tag = node.findall(tagname)
  if tag:
    return tag[0]
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


def indent(string, spaces):
  return string.replace('\n', '\n' + ' ' * spaces).strip()


def model_from_include(parent, include_node):
    submodel_uri = get_tag(include_node, 'uri')
    submodel_path = submodel_uri.replace('model://', models_path) + os.path.sep + 'model.sdf'
    submodel_name = get_tag(include_node, 'name')
    submodel_pose = get_tag_pose(include_node)
    return Model(parent, name=submodel_name, pose=submodel_pose, file=submodel_path)






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
    if self.version != 1.4:
      print('Unsupported SDF version in %s. Aborting.\n' % filename)
      return
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
    for include_node in node.iter('include'):
      self.models.append(model_from_include(None, include_node))


  def plot_to_file(self, plot_filename):
    import pygraphviz as pgv
    graph = pgv.AGraph()
    self.plot(graph)
    graph.draw(plot_filename, prog='dot')


  def plot(self, graph):
    graph.add_node('world')

    for model in self.models:
      model.plot(graph)
      graph.add_edge('world', model.name)



class SpatialEntity(object):
  def __init__(self, **kwargs):
    self.name = kwargs.get('name', '')
    self.pose = kwargs.get('pose', identity_matrix())


  def __repr__(self):
    return ''.join((
      'name: %s\n' % self.name,
      'pose: %s\n' % homogeneous2tq_string_rounded(self.pose),
    ))


  def from_tree(self, node, **kwargs):
    if node == None:
      return
    self.name = node.attrib['name']
    self.pose = numpy.dot(kwargs.get('pose', identity_matrix()), get_tag_pose(node))



class Model(SpatialEntity):
  def __init__(self, parent_model = None, **kwargs):
    super(Model, self).__init__(**kwargs)
    self.parent_model = parent_model
    self.submodels = []
    self.links = []
    self.joints = []
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'], **kwargs)
    elif 'file' in kwargs:
      self.from_file(kwargs['file'], **kwargs)

    self.root_link = self.find_root_link()


  def __repr__(self):
    return ''.join((
      'Model(\n', 
      '  %s\n' % indent(super(Model, self).__repr__(), 2),
      '  root_link: %s\n' % self.root_link.name,
      '  links:\n',
      '    %s' % '\n    '.join([indent(str(l), 4) for l in self.links]),
      '\n',
      '  joints:\n',
      '    %s' % '\n    '.join([indent(str(j), 4) for j in self.joints]),
      '\n',
      '  submodels:\n',
      '    %s' % '\n    '.join([indent(str(m), 4) for m in self.submodels]),
      '\n',
      ')'
    ))


  def from_file(self, filename, **kwargs):
    tree = ET.parse(filename)
    root = tree.getroot()
    if root.tag != 'sdf':
      print('Not a SDF file. Aborting.')
      return
    version = float(root.attrib['version'])
    if version != 1.4:
      print('Unsupported SDF version in %s. Aborting.\n' % filename)
      return
    modelnode = get_node(root, 'model')
    self.from_tree(modelnode, **kwargs)

    # Override name (e.g. for <include>)
    kwargs_name = kwargs.get('name')
    if kwargs_name:
      self.name = kwargs_name

    # External pose offset (from <include>)
    self.pose = numpy.dot(kwargs.get('pose', identity_matrix()), self.pose)


  def from_tree(self, node, **kwargs):
    if node == None:
      return
    if node.tag != 'model':
      print('Invalid node of type %s instead of model. Aborting.' % node.tag)
      return
    super(Model, self).from_tree(node, **kwargs)
    self.links = [Link(self, tree=link_node) for link_node in node.iter('link')]
    self.joints = [Joint(self, tree=joint_node) for joint_node in node.iter('joint')]

    for include_node in node.iter('include'):
      self.submodels.append(model_from_include(self, include_node))


  def to_urdf(self):
    # TODO
    return ''


  def save_urdf(self, filename):
    urdf_file = open(filename, 'w')
    pretty_urdf_string = prettyXML(self.to_urdf())
    urdf_file.write(pretty_urdf_string)
    urdf_file.close()


  def get_joint(self, requested_jointname, prefix = ''):
    full_prefix = prefix + '::' if prefix else ''
    for joint in self.joints:
      if full_prefix + joint.name == requested_jointname:
        return joint
    for submodel in self.submodels:
      res = submodel.get_joint(requested_jointname, submodel.name)
      if res:
        return res


  def get_link(self, requested_linkname, prefix = ''):
    full_prefix = prefix + '::' if prefix else ''
    for link in self.links:
      if full_prefix + link.name == requested_linkname:
        return link
    for submodel in self.submodels:
      res = submodel.get_link(requested_linkname, self.name + '::' + submodel.name)
      if res:
        return res


  def find_root_link(self):
    curr_link = self.links[0]
    while True:
      parent_link = self.get_parent(curr_link.name)
      if not parent_link:
        return curr_link
      curr_link = parent_link


  def get_parent(self, requested_linkname, prefix = ''):
    full_prefix = prefix + '::' if prefix else ''
    for joint in self.joints:
      if joint.child == full_prefix + requested_linkname:
        return self.get_link(joint.parent)
    if self.parent_model:
      return self.parent_model.get_parent(requested_linkname, self.name)


  def plot(self, graph):
    pass



class Link(SpatialEntity):
  def __init__(self, parent_model, **kwargs):
    super(Link, self).__init__(**kwargs)
    self.parent_model = parent_model
    self.gravity = True
    self.inertial = Inertial()
    self.collision = Collision()
    self.visual = Visual()
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Link(\n',
      '  %s\n' % indent(super(Link, self).__repr__(), 2),
      '  %s\n' % indent(str(self.inertial), 2),
      '  collision: %s\n' % self.collision,
      '  visual: %s\n' % self.visual,
      ')'
    ))


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'link':
      print('Invalid node of type %s instead of link. Aborting.' % node.tag)
      return
    super(Link, self).from_tree(node)
    self.inertial = Inertial(tree=get_node(node, 'inertial'))
    self.collision = Collision(tree=get_node(node, 'collision'))
    self.visual = Visual(tree=get_node(node, 'visual'))



class Joint(SpatialEntity):
  def __init__(self, parent_model, **kwargs):
    super(Joint, self).__init__(**kwargs)
    self.parent_model = parent_model
    self.type = ''
    self.parent = ''
    self.child = ''
    self.axis = Axis()
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Joint(\n',
      '  %s\n' % indent(super(Joint, self).__repr__(), 2),
      '  type: %s\n' % self.type,
      '  parent: %s\n' % self.parent,
      '  child: %s\n' % self.child,
      '  axis: %s\n' % self.axis,
      ')'
    ))


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'joint':
      print('Invalid node of type %s instead of joint. Aborting.' % node.tag)
      return
    super(Joint, self).from_tree(node)
    self.type = node.attrib['type']
    #TODO self.pose = numpy.dot(self.child.pose, self.pose)
    self.parent = get_tag(node, 'parent', '')
    self.child = get_tag(node, 'child', '')
    self.axis = Axis(tree=get_node(node, 'axis'))



class Axis(object):
  def __init__(self, **kwargs):
    self.xyz = numpy.array([0, 0, 0])
    self.lower_limit = 0
    self.upper_limit = 0
    self.effort_limit = 0
    self.velocity_limit = 0
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return 'Axis(xyz=%s, lower_limit=%s, upper_limit=%s, effort=%s, velocity=%s)' % (self.xyz, self.lower_limit, self.upper_limit, self.effort_limit, self.velocity_limit)


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'axis':
      print('Invalid node of type %s instead of axis. Aborting.' % node.tag)
      return
    self.xyz = numpy.array(get_tag(node, 'xyz'))
    limitnode = get_node(node, 'limit')
    if limitnode == None:
      print('limit Tag missing from joint. Aborting.')
      return
    self.lower_limit = float(get_tag(limitnode, 'lower', 0))
    self.upper_limit = float(get_tag(limitnode, 'upper', 0))
    self.effort_limit = float(get_tag(limitnode, 'effort', 0))
    self.velocity_limit = float(get_tag(limitnode, 'velocity', 0))




class Inertial(object):
  def __init__(self, **kwargs):
    self.pose = identity_matrix()
    self.mass = 0
    self.inertia = Inertia()
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Inertial(\n',
      '  pose: %s\n' % homogeneous2tq_string_rounded(self.pose),
      '  mass: %s\n' % self.mass,
      '  inertia: %s\n' % self.inertia,
      ')'
    ))


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'inertial':
      print('Invalid node of type %s instead of inertial. Aborting.' % node.tag)
      return
    self.pose = get_tag_pose(node)
    self.mass = get_tag(node, 'mass', 0)
    self.inertia = Inertia(tree=get_node(node, 'inertia'))



class Inertia(object):
  def __init__(self, **kwargs):
    self.ixx = 0
    self.ixy = 0
    self.ixz = 0
    self.iyy = 0
    self.iyz = 0
    self.izz = 0
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return 'Inertia(ixx=%s, ixy=%s, ixz=%s, iyy=%s, iyz=%s, izz=%s)' % (self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz)


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'inertia':
      print('Invalid node of type %s instead of inertia. Aborting.' % node.tag)
      return
    for coord in 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz':
      setattr(self, coord, get_tag(node, coord, 0))



class LinkPart(SpatialEntity):
  def __init__(self, **kwargs):
    super(LinkPart, self).__init__(**kwargs)
    self.geometry_type = None
    self.geometry_data = {}
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'visual' and node.tag != 'collision':
      print('Invalid node of type %s instead of visual or collision. Aborting.' % node.tag)
      return
    super(LinkPart, self).from_tree(node)
    gnode = get_node(node, 'geometry')
    if gnode == None:
      return
    for gtype in 'box', 'cylinder', 'sphere', 'mesh':
      typenode = get_node(gnode, gtype)
      if typenode != None:
        self.geometry_type = gtype
        if gtype == 'box':
          self.geometry_data = {'size': get_tag(typenode, 'size')}
        elif gtype == 'cylinder':
          self.geometry_data = {'radius': get_tag(typenode, 'radius'), 'length': get_tag(typenode, 'length')}
        elif gtype == 'sphere':
          self.geometry_data = {'radius': get_tag(typenode, 'radius')}
        elif gtype == 'mesh':
          self.geometry_data = {'uri': get_tag(typenode, 'uri'), 'scale': get_tag(typenode, 'scale')}


  def __repr__(self):
    return '%s geometry_type: %s, geometry_data: %s' % (super(LinkPart, self).__repr__().replace('\n', ', ').strip(), self.geometry_type, self.geometry_data)



class Collision(LinkPart):
  def __init__(self, **kwargs):
    super(Collision, self).__init__(**kwargs)


  def __repr__(self):
    return 'Collision(%s)' % super(Collision, self).__repr__()



class Visual(LinkPart):
  def __init__(self, **kwargs):
    super(Visual, self).__init__(**kwargs)


  def __repr__(self):
    return 'Visual(%s)' % super(Visual, self).__repr__()
