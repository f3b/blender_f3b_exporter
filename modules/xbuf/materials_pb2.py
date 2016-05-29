# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: xbuf/materials.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import xbuf.primitives_pb2

from xbuf.primitives_pb2 import *

DESCRIPTOR = _descriptor.FileDescriptor(
  name='xbuf/materials.proto',
  package='xbuf',
  serialized_pb=_b('\n\x14xbuf/materials.proto\x12\x04xbuf\x1a\x15xbuf/primitives.proto\"\xbc\x04\n\x08Material\x12\n\n\x02id\x18\x01 \x02(\t\x12\x0e\n\x06\x66\x61mily\x18\x03 \x01(\t\x12\x0c\n\x04name\x18\x04 \x01(\t\x12\x17\n\tshadeless\x18\x05 \x01(\x08:\x04true\x12\x1a\n\x05\x63olor\x18\n \x01(\x0b\x32\x0b.xbuf.Color\x12 \n\tcolor_map\x18\x0b \x01(\x0b\x32\r.xbuf.Texture\x12\x0f\n\x07opacity\x18\x0c \x01(\x02\x12\"\n\x0bopacity_map\x18\r \x01(\x0b\x32\r.xbuf.Texture\x12\x1a\n\x06normal\x18\x0e \x01(\x0b\x32\n.xbuf.Vec3\x12!\n\nnormal_map\x18\x0f \x01(\x0b\x32\r.xbuf.Texture\x12\x11\n\troughness\x18\x10 \x01(\x02\x12$\n\rroughness_map\x18\x11 \x01(\x0b\x32\r.xbuf.Texture\x12\x11\n\tmetalness\x18\x12 \x01(\x02\x12$\n\rmetalness_map\x18\x13 \x01(\x0b\x32\r.xbuf.Texture\x12\x1d\n\x08specular\x18\x14 \x01(\x0b\x32\x0b.xbuf.Color\x12#\n\x0cspecular_map\x18\x15 \x01(\x0b\x32\r.xbuf.Texture\x12\x16\n\x0especular_power\x18\x16 \x01(\x02\x12)\n\x12specular_power_map\x18\x17 \x01(\x0b\x32\r.xbuf.Texture\x12\x1d\n\x08\x65mission\x18\x18 \x01(\x0b\x32\x0b.xbuf.Color\x12#\n\x0c\x65mission_map\x18\x19 \x01(\x0b\x32\r.xbuf.TextureP\x00')
  ,
  dependencies=[xbuf.primitives_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_MATERIAL = _descriptor.Descriptor(
  name='Material',
  full_name='xbuf.Material',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='xbuf.Material.id', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='family', full_name='xbuf.Material.family', index=1,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='name', full_name='xbuf.Material.name', index=2,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='shadeless', full_name='xbuf.Material.shadeless', index=3,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='color', full_name='xbuf.Material.color', index=4,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='color_map', full_name='xbuf.Material.color_map', index=5,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='opacity', full_name='xbuf.Material.opacity', index=6,
      number=12, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='opacity_map', full_name='xbuf.Material.opacity_map', index=7,
      number=13, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='normal', full_name='xbuf.Material.normal', index=8,
      number=14, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='normal_map', full_name='xbuf.Material.normal_map', index=9,
      number=15, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='roughness', full_name='xbuf.Material.roughness', index=10,
      number=16, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='roughness_map', full_name='xbuf.Material.roughness_map', index=11,
      number=17, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='metalness', full_name='xbuf.Material.metalness', index=12,
      number=18, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='metalness_map', full_name='xbuf.Material.metalness_map', index=13,
      number=19, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='specular', full_name='xbuf.Material.specular', index=14,
      number=20, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='specular_map', full_name='xbuf.Material.specular_map', index=15,
      number=21, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='specular_power', full_name='xbuf.Material.specular_power', index=16,
      number=22, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='specular_power_map', full_name='xbuf.Material.specular_power_map', index=17,
      number=23, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='emission', full_name='xbuf.Material.emission', index=18,
      number=24, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='emission_map', full_name='xbuf.Material.emission_map', index=19,
      number=25, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=54,
  serialized_end=626,
)

_MATERIAL.fields_by_name['color'].message_type = xbuf.primitives_pb2._COLOR
_MATERIAL.fields_by_name['color_map'].message_type = xbuf.primitives_pb2._TEXTURE
_MATERIAL.fields_by_name['opacity_map'].message_type = xbuf.primitives_pb2._TEXTURE
_MATERIAL.fields_by_name['normal'].message_type = xbuf.primitives_pb2._VEC3
_MATERIAL.fields_by_name['normal_map'].message_type = xbuf.primitives_pb2._TEXTURE
_MATERIAL.fields_by_name['roughness_map'].message_type = xbuf.primitives_pb2._TEXTURE
_MATERIAL.fields_by_name['metalness_map'].message_type = xbuf.primitives_pb2._TEXTURE
_MATERIAL.fields_by_name['specular'].message_type = xbuf.primitives_pb2._COLOR
_MATERIAL.fields_by_name['specular_map'].message_type = xbuf.primitives_pb2._TEXTURE
_MATERIAL.fields_by_name['specular_power_map'].message_type = xbuf.primitives_pb2._TEXTURE
_MATERIAL.fields_by_name['emission'].message_type = xbuf.primitives_pb2._COLOR
_MATERIAL.fields_by_name['emission_map'].message_type = xbuf.primitives_pb2._TEXTURE
DESCRIPTOR.message_types_by_name['Material'] = _MATERIAL

Material = _reflection.GeneratedProtocolMessageType('Material', (_message.Message,), dict(
  DESCRIPTOR = _MATERIAL,
  __module__ = 'xbuf.materials_pb2'
  # @@protoc_insertion_point(class_scope:xbuf.Material)
  ))
_sym_db.RegisterMessage(Material)


# @@protoc_insertion_point(module_scope)