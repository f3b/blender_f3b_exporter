# This file is part of blender_io_f3b.  blender_io_f3b is free software: you can
# redistribute it and/or modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation, version 2.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Copyright David Bernard, Riccardo Balbo

# <pep8 compliant>

import mathutils
import bpy_extras
import math
import f3b
import f3b.datas_pb2
import f3b.custom_params_pb2
import f3b.animations_kf_pb2
import f3b.physics_pb2

from . import helpers 
from .utils import * 
from .exporter_utils import *

import re,os
import subprocess

DDS_WRITER_PATH=os.path.dirname(__file__)+"/bin/dds_writer.sh"
DDS_SUPPORT=True

class ExportCfg:
    def __init__(self, is_preview=False, assets_path="/tmp",option_export_selection=False,textures_to_dds=False,export_tangents=False,remove_doubles=False):
        self.is_preview = is_preview
        self.assets_path = bpy.path.abspath(assets_path)
        self._modified = {}
        self._ids = {}
        self.option_export_selection=option_export_selection
        self.textures_to_dds=textures_to_dds
        self.export_tangents=export_tangents
        self.remove_doubles=remove_doubles

    def _k_of(self, v):
        # hash(v) or id(v) ?
        return str(hash(v))

    def id_of(self, v):
        k = self._k_of(v)
        if k in self._ids:
            out = self._ids[k]
        else:
            # out = str(uuid.uuid4().clock_seq)
            out = str(hash(v))
            self._ids[k] = out
        return out

    def need_update(self, v, modified=False):
        k = self._k_of(v)
        old = (k not in self._modified) or self._modified[k]
        self._modified[k] = modified
        return old

    def info(self, txt):
        print("INFO: " + txt)

    def warning(self, txt):
        print("WARNING: " + txt)

    def error(self, txt):
        print("ERROR: " + txt)



def export(scene, data, cfg):
    export_all_tobjects(scene, data, cfg)
    export_all_geometries(scene, data, cfg)
    export_all_materials(scene, data, cfg)
    export_all_lights(scene, data, cfg)
    export_all_skeletons(scene, data, cfg)
    export_all_actions(scene, data, cfg)
    export_all_physics(scene, data, cfg)

def export_all_tobjects(scene, data, cfg):
    for obj in scene.objects:
        if obj.hide_render or (cfg.option_export_selection and not obj.select):
            print("Skip ",obj,"not selected/render disabled")
            continue
        if cfg.need_update(obj):
            tobject = data.tobjects.add()
            tobject.id = cfg.id_of(obj)
            tobject.name = obj.name
            loc, quat, scale = obj.matrix_local.decompose()
            cnv_scale(scale, tobject.scale)
            cnv_translation(loc, tobject.translation)
            if obj.type == 'MESH':
                cnv_rotation(quat, tobject.rotation)
            elif obj.type == 'Armature':
                cnv_rotation(quat, tobject.rotation)
            elif obj.type == 'LAMP':
                rot = helpers.z_backward_to_forward(quat)
                cnv_quatZupToYup(rot, tobject.rotation)
            else:
                cnv_rotation(helpers.rot_quat(obj), tobject.rotation)
            if obj.parent is not None:
                add_relation_raw(data.relations, f3b.datas_pb2.TObject.__name__, cfg.id_of(obj.parent), f3b.datas_pb2.TObject.__name__, cfg.id_of(obj), cfg)
            export_obj_customproperties(obj, tobject, data, cfg)
        else:
            print("Skip ",obj,"already exported")


def export_all_physics(scene, data, cfg):
    for obj in scene.objects:
        if obj.hide_render or (cfg.option_export_selection and not obj.select):
            continue
        phy_data = None
        phy_data = export_rb(obj, phy_data, data, cfg)
        export_rbct(obj, phy_data, data, cfg)

def export_rbct(ob, phy_data, data, cfg):
    btct = ob.rigid_body_constraint

    if not btct or not cfg.need_update(btct):
        return

    if phy_data == None:
        phy_data = data.physics.add()
    ct_type = btct.type
    constraint = phy_data.constraint
    constraint.id = cfg.id_of(btct)

    o1 = btct.object1
    o2 = btct.object2

    o1_wp = o1.matrix_world.to_translation()
    o2_wp = o2.matrix_world.to_translation()

    constraint.a_ref = cfg.id_of(o1.rigid_body)
    constraint.b_ref = cfg.id_of(o2.rigid_body)

    if ct_type == "GENERIC":
        generic = constraint.generic
        cnv_vec3((0, 0, 0), generic.pivotA)
        cnv_vec3(cnv_toVec3ZupToYup(o1_wp-o2_wp), generic.pivotB)
        generic.disable_collisions = btct.disable_collisions

        if btct.use_limit_lin_x:
            limit_lin_x_upper = btct.limit_lin_x_upper
            limit_lin_x_lower = btct.limit_lin_x_lower
        else:
            limit_lin_x_upper = float('inf')
            limit_lin_x_lower = float('-inf')

        if btct.use_limit_lin_y:
            limit_lin_y_upper = btct.limit_lin_y_upper
            limit_lin_y_lower = btct.limit_lin_y_lower
        else:
            limit_lin_y_upper = float('inf')
            limit_lin_y_lower = float('-inf')

        if btct.use_limit_lin_z:
            limit_lin_z_upper = btct.limit_lin_z_upper
            limit_lin_z_lower = btct.limit_lin_z_lower
        else:
            limit_lin_z_upper = float('inf')
            limit_lin_z_lower = float('-inf')

        if btct.use_limit_ang_x:
            limit_ang_x_upper = btct.limit_ang_x_upper
            limit_ang_x_lower = btct.limit_ang_x_lower
        else:
            limit_ang_x_upper = float('inf')
            limit_ang_x_lower = float('-inf')

        if btct.use_limit_ang_y:
            limit_ang_y_upper = btct.limit_ang_y_upper
            limit_ang_y_lower = btct.limit_ang_y_lower
        else:
            limit_ang_y_upper = float('inf')
            limit_ang_y_lower = float('-inf')

        if btct.use_limit_ang_z:
            limit_ang_z_upper = btct.limit_ang_z_upper
            limit_ang_z_lower = btct.limit_ang_z_lower
        else:
            limit_ang_z_upper = float('inf')
            limit_ang_z_lower = float('-inf')

        cnv_vec3(cnv_toVec3ZupToYup((limit_lin_x_upper, limit_lin_y_upper, limit_lin_z_upper)), generic.upperLinearLimit)
        cnv_vec3(cnv_toVec3ZupToYup((limit_lin_x_lower, limit_lin_y_lower, limit_lin_z_lower)), generic.lowerLinearLimit)
        cnv_vec3(cnv_toVec3ZupToYup((limit_ang_x_upper, limit_ang_y_upper, limit_ang_z_upper)), generic.upperAngularLimit)
        cnv_vec3(cnv_toVec3ZupToYup((limit_ang_x_lower, limit_ang_y_lower, limit_ang_z_lower)), generic.lowerAngularLimit)


def export_rb(ob, phy_data, data, cfg):
    if not  ob.rigid_body or not cfg.need_update(ob.rigid_body):
        return

    if phy_data is None:
        phy_data = data.physics.add()

    rigidbody = phy_data.rigidbody
    rigidbody.id = cfg.id_of(ob.rigid_body)

    rbtype = ob.rigid_body.type
    dynamic = ob.rigid_body.enabled
    if rbtype == "PASSIVE" or not dynamic:
        rigidbody.type = f3b.datas_pb2.RigidBody.tstatic
    else:
        rigidbody.type = f3b.datas_pb2.RigidBody.tdynamic
    # Ghost?

    rigidbody.mass = ob.rigid_body.mass
    rigidbody.isKinematic = ob.rigid_body.kinematic
    rigidbody.friction = ob.rigid_body.friction
    rigidbody.restitution = ob.rigid_body.restitution
    if not ob.rigid_body.use_margin:
        rigidbody.margin = 0
    else:
        rigidbody.margin = ob.rigid_body.collision_margin

    rigidbody.linearDamping = ob.rigid_body.linear_damping
    rigidbody.angularDamping = ob.rigid_body.angular_damping
    cnv_vec3((1, 1, 1), rigidbody.angularFactor) #Not used
    cnv_vec3((1, 1, 1), rigidbody.linearFactor) #Not used

    shape = ob.rigid_body.collision_shape
    if shape == "MESH":
        shape = f3b.datas_pb2.PhysicsData.smesh
    elif shape == "SPHERE":
        shape = f3b.datas_pb2.PhysicsData.ssphere
    elif shape == "CONVEX_HULL":
        shape = f3b.datas_pb2.PhysicsData.shull
    elif shape == "BOX":
        shape = f3b.datas_pb2.PhysicsData.sbox
    elif shape == "CAPSULE":
        shape = f3b.datas_pb2.PhysicsData.scapsule
    elif shape == "CYLINDER":
        shape = f3b.datas_pb2.PhysicsData.scylinder
    elif shape == "CONE":
        shape = f3b.datas_pb2.PhysicsData.scone


    rigidbody.shape = shape

    collision_groups = ob.rigid_body.collision_groups
    collision_group = 0
    i = 0
    for g in collision_groups:
        if g:
            collision_group |= (g<<i)
        i += 1

    rigidbody.collisionGroup = collision_group
    rigidbody.collisionMask = collision_group

    add_relation_raw(data.relations, f3b.datas_pb2.TObject.__name__, cfg.id_of(ob), f3b.datas_pb2.RigidBody.__name__, rigidbody.id, cfg)
    return phy_data


def export_all_geometries(scene, data, cfg):
    for obj in scene.objects:
        if obj.hide_render or (cfg.option_export_selection and not obj.select):
            print("Skip ",obj,"not selected/render disabled")
            continue
        if obj.type == 'MESH':
            if len(obj.data.polygons) != 0 and cfg.need_update(obj.data):
                meshes = export_meshes(obj, data.meshes, scene, cfg)
                for material_index, mesh in meshes.items():
                    # several object can share the same mesh
                    for obj2 in scene.objects:
                        if obj2.data == obj.data: 
                            add_relation_raw(data.relations, f3b.datas_pb2.Mesh.__name__, mesh.id, f3b.datas_pb2.TObject.__name__, cfg.id_of(obj2), cfg)
                    if material_index > -1 and material_index < len(obj.material_slots):
                        src_mat = obj.material_slots[material_index].material
                        add_relation_raw(data.relations, f3b.datas_pb2.Mesh.__name__, mesh.id, f3b.datas_pb2.Material.__name__, cfg.id_of(src_mat), cfg)
            else:
                print("Skip ",obj,"already exported")


def export_all_materials(scene, data, cfg):
    for obj in scene.objects:
        if obj.hide_render or (cfg.option_export_selection and not obj.select):
            continue
        if obj.type == 'MESH':
            for i in range(len(obj.material_slots)):
                src_mat = obj.material_slots[i].material
                if cfg.need_update(src_mat):
                    dst_mat = data.materials.add()
                    export_material(src_mat, dst_mat, cfg)
    __async_wait()


def export_all_lights(scene, data, cfg):
    for obj in scene.objects:
        if obj.hide_render or (cfg.option_export_selection and not obj.select):
            continue
        if obj.type == 'LAMP':
            src_light = obj.data
            if 1==1:#cfg.need_update(src_light):
                dst_light = data.lights.add()
                export_light(src_light, dst_light, cfg)
                add_relation_raw(data.relations, f3b.datas_pb2.TObject.__name__, cfg.id_of(obj), f3b.datas_pb2.Light.__name__, cfg.id_of(src_light), cfg)


def add_relation(relations, e1, e2, cfg):
    add_relation_raw(relations, type(e1).__name__, e1.id, type(e2).__name__, e2.id, cfg)


def add_relation_raw(relations, t1, ref1, t2, ref2, cfg):
    rel = relations.add()
    if t1 <= t2:
        rel.ref1 = ref1
        rel.ref2 = ref2
        cfg.info("add relation: '%s'(%s) to '%s'(%s)" % (t1, ref1, t2, ref2))
    else:
        rel.ref1 = ref2
        rel.ref2 = ref1
        cfg.info("add relation: '%s'(%s) to '%s'(%s)" % (t2, ref2, t1, ref1))


def export_meshes(src_geometry, meshes, scene, cfg):
    mode = 'PREVIEW' if cfg.is_preview else 'RENDER'
    # Set up modifiers whether to apply deformation or not
    # tips from https://code.google.com/p/blender-cod/source/browse/blender_26/export_xmodel.py#185
    mod_armature = []
    mod_state_attr = 'show_viewport' if cfg.is_preview else 'show_render'
    for mod in src_geometry.modifiers:
        if mod.type == 'ARMATURE':
            mod_armature.append((mod, getattr(mod, mod_state_attr)))

    tmp_modifier=[]
    #Add triangulate modifier
    tmp_modifier.append(src_geometry.modifiers.new("TriangulateForF3b","TRIANGULATE"))

    # -- without armature applied
    for mod in mod_armature:
        setattr(mod[0], mod_state_attr, False)
    # FIXME apply transform for mesh under armature modify the blender data !!
    # if src_geometry.find_armature():
    #     apply_transform(src_geometry)
    src_mesh = src_geometry.to_mesh(scene, True, mode, True, False)
    # Restore modifier settings
    for mod in mod_armature:
        setattr(mod[0], mod_state_attr, mod[1])

    # dst.id = cfg.id_of(src_geometry.data)
    # dst.name = src_geometry.name
    faces = src_mesh.tessfaces
    dstMap = {}
    for face in faces:
        material_index = face.material_index
        if material_index not in dstMap:
            dstMap[material_index] = meshes.add()

    for material_index, dst in dstMap.items():
        dst.primitive = f3b.datas_pb2.Mesh.triangles
        dst.id = cfg.id_of(src_mesh) + "_" + str(material_index)
        dst.name = src_geometry.data.name + "_" + str(material_index)
        dst_mesh=dst

        #Collect mesh data 
        mesh=extract_meshdata(src_mesh,src_geometry,material_index,cfg.export_tangents,cfg.remove_doubles)   

        positions = dst_mesh.vertexArrays.add()
        positions.attrib = f3b.datas_pb2.VertexArray.position
        positions.floats.step = 3
        
        normals = dst_mesh.vertexArrays.add()
        normals.attrib = f3b.datas_pb2.VertexArray.normal
        normals.floats.step = 3
        
        indexes = dst_mesh.indexArrays.add()
        indexes.ints.step = 3

        texcoords=[]
        texcoords_ids=[f3b.datas_pb2.VertexArray.texcoord,f3b.datas_pb2.VertexArray.texcoord2,f3b.datas_pb2.VertexArray.texcoord3,f3b.datas_pb2.VertexArray.texcoord4,f3b.datas_pb2.VertexArray.texcoord5,f3b.datas_pb2.VertexArray.texcoord6,f3b.datas_pb2.VertexArray.texcoord7,f3b.datas_pb2.VertexArray.texcoord8]

        if mesh.verts[0].tg: 
            tangents_ids=[f3b.datas_pb2.VertexArray.tangent,f3b.datas_pb2.VertexArray.tangent2,f3b.datas_pb2.VertexArray.tangent3,f3b.datas_pb2.VertexArray.tangent4,f3b.datas_pb2.VertexArray.tangent5,f3b.datas_pb2.VertexArray.tangent6,f3b.datas_pb2.VertexArray.tangent7,f3b.datas_pb2.VertexArray.tangent8]
            tangents=[]

        print("Found ",len(mesh.verts[0].tx)," uvs")
        for i in range(0,min(9, len(mesh.verts[0].tx))):
            texcoords.append(dst_mesh.vertexArrays.add())
            texcoords[i].attrib=texcoords_ids[i]
            texcoords[i].floats.step = 2
            if mesh.verts[0].tg: 
                tangents.append(dst_mesh.vertexArrays.add())
                tangents[i].attrib = tangents_ids[i]
                tangents[i].floats.step = 4
            

        if mesh.verts[0].c:
            colors = dst_mesh.vertexArrays.add()
            colors.attrib = f3b.datas_pb2.VertexArray.color
            colors.floats.step = 4



        indexes.ints.values.extend(mesh.indexes)
        for v in mesh.verts:
            positions.floats.values.extend(v.p)
            normals.floats.values.extend(v.n)
            if v.c:
                colors.floats.values.extend(v.c)
            if v.tx:
                for i,tx in enumerate(v.tx):
                    texcoords[i].floats.values.extend(tx)
                    if v.tg:
                        tangents[i].floats.values.extend(v.tg[i])            

        if mesh.has_skin:
            dst_skin=dst_mesh.skin
            dst_skin.boneCount.extend(mesh.skin.boneCount)
            dst_skin.boneIndex.extend(mesh.skin.boneIndex)
            dst_skin.boneWeight.extend(mesh.skin.boneWeight)

    for m in tmp_modifier:
        src_geometry.modifiers.remove(m)

    return dstMap


# FIXME side effect on the original scene (selection, and transform of the src_geometry)
def apply_transform(src_geometry):
    # bpy.ops.object.select_all(action='DESELECT') # deselect everything to avoid a mess
    override = {'selected_editable_objects': src_geometry}
    # src_geometry.select = True # lets select every mesh as we go
    bpy.ops.object.visual_transform_apply(override)
    # bpy.ops.object.transform_apply(override, location=True, rotation=True, scale=True)
    # src_geometry.select = False # we're done working on this object


CYCLES_EXPORTABLE_MATS_PATTERN=re.compile("\\!\s*([^;]+)")
CYCLES_MAT_INPUT_PATTERN=re.compile("\\!\s*([^;]+)");
CYCLES_CUSTOM_NODEINPUT_PATTERN=re.compile("([^;]+)");

def dumpCyclesExportableMats(intree,outarr,parent=None):
    if intree==None: return
    name=intree.name
    name_match=CYCLES_EXPORTABLE_MATS_PATTERN.match(name)
    if name_match!=None and intree!=None:
        material_path=name_match.group(1)
        mat=parent
        outarr.append([material_path,mat])
    if isinstance(intree, bpy.types.NodeTree):        
        for node in intree.nodes:
            dumpCyclesExportableMats(node,outarr,intree) 
    try:             
        dumpCyclesExportableMats(intree.node_tree,outarr,intree)
    except: pass

from  concurrent.futures import ThreadPoolExecutor
_ASYNC_POOL= ThreadPoolExecutor(max_workers=2)
_ASYNC_POOL_W=[]

def __async(f,*a):
   # f(*a)
   _ASYNC_POOL_W.append(_ASYNC_POOL.submit( f, *a))



def __async_wait():
    print("Wait for async tasks")
    global _ASYNC_POOL_W    
    for s  in  _ASYNC_POOL_W:
        s.result()
    _ASYNC_POOL_W=[]
    print("Done")



def parseNode(input_node,input_type,dst_mat,input_label,cfg):                    
    #input_node=input.links[0].from_node
    #input_type=input_node.type                    
    parts=input_label.split("+")
    input_label=parts[0]
    args=parts[1] if len(parts)>1 else ""

    if input_type=="RGB" or input_type=="RGBA":
        prop=dst_mat.properties.add()
        prop.id=input_label
        cnv_color(input_node.outputs[0].default_value,prop.vcolor)
        print("Found color",prop.vcolor)
    # Deprecated:  Use custom Fload and Int nodes instead
    #elif input_type=="VALUE":
    #    prop=dst_mat.properties.add()
    #    prop.id=input_label
    #    prop.value=input_node.outputs[0].default_value
    #    print("Found value",prop.value)
    elif input_type=="TEX_IMAGE":
        prop=dst_mat.properties.add()
        prop.id=input_label
        solid=len(input_node.outputs[1].links)==0 #If alpha is not connected= solid
        __async(export_tex,solid,args,input_node.image,prop.texture,cfg)
        print("Found texture")
    elif input_type=="GROUP": # Custom nodes groups as input
        name=input_node.node_tree.name
        name=CYCLES_CUSTOM_NODEINPUT_PATTERN.match(name)
        if name != None:
            name=name.group(0).upper()
            if name == "VEC3":
                x,y,z=input_node.inputs
                x=x.default_value
                y=y.default_value
                z=z.default_value
                prop=dst_mat.properties.add()
                prop.id=input_label
                cnv_vec3((x,y,z), prop.vvec3)
                print("Found vec3",prop.vvec3)
            elif name == "VEC2":
                x,y=input_node.inputs
                x=x.default_value
                y=y.default_value
                prop=dst_mat.properties.add()
                prop.id=input_label
                cnv_vec2((x,y), prop.vvec2)
                print("Found vec2",prop.vvec2)
            elif name == "VEC4" or name == "QTR":
                x,y,z,w=input_node.inputs
                x=x.default_value
                y=y.default_value
                z=z.default_value
                w=w.default_value
                prop=dst_mat.properties.add()
                prop.id=input_label
                cnv_vec4((x,y,z,w), prop.vvec4 if name == "QTR" else prop.vqtr)
                print("Found vec4",prop.vvec4)
            elif name=="RGBA":
                r,g,b,a=input_node.inputs
                
                r=r.default_value
                g=r.default_value
                b=r.default_value
                a=r.default_value
                
                prop=dst_mat.properties.add()
                prop.id=input_label
                cnv_color((r,g,b,a),prop.vcolor)
                print("Found color",prop.vcolor)
            elif name == "FLOAT":
                prop=dst_mat.properties.add()
                prop.id=input_label            
                prop.vfloat=float(input_node.inputs[0].default_value)
                print("Found Float",prop.vfloat)
            elif name == "INT":
                prop=dst_mat.properties.add()
                prop.id=input_label            
                prop.vint=int(input_node.inputs[0].default_value)
                print("Found Int",prop.vint)
            elif name == "TRUE":
                prop=dst_mat.properties.add()
                prop.id=input_label            
                prop.vbool=True
                print("Found boolean TRUE")
            elif name == "FALSE":
                prop=dst_mat.properties.add()
                prop.vbool=False     
                prop.id=input_label     
                print("Found boolean FALSE")          
            elif name == "PRESET":
                for n in input_node.outputs[0].links[0].from_node.node_tree.nodes:
                    if n.type=="GROUP_OUTPUT":
                        input_node=n.inputs[0].links[0].from_node
                        input_type=input_node.type
                        print("Found preset")
                        parseNode(input_node,input_type,dst_mat,input_label,cfg)
                        print("Preset end")
                        break    
            else: 
                print(input_type,"not supported [1]",name)
    else: 
        print(input_type,"not supported")
    
def export_material(src_mat, dst_mat, cfg):
    dst_mat.id = cfg.id_of(src_mat)
    dst_mat.name = src_mat.name
    cycles_mat=[]
    dumpCyclesExportableMats(src_mat.node_tree,cycles_mat)
    if len(cycles_mat)>0: 
        dst_mat.mat_id,cycles_mat=cycles_mat[0]
        dst_mat.name=src_mat.name
        for input in cycles_mat.inputs:
            input_label=input.name
            input_label=CYCLES_MAT_INPUT_PATTERN.match(input_label)
            if input_label == None:
               print("Skip ",input.name)                
            else:
                input_label=input_label.group(1)
                print("Export ",input_label)
                input_label=input_label.strip()
                if len(input.links) > 0: 
                    input_node=input.links[0].from_node
                    input_type=input_node.type    
                    parseNode(input_node,input_type,dst_mat,input_label,cfg)
                
             
                
EXT_FORMAT_MAP={"targa":"tga","jpeg":"jpg","targa_raw":"tga"}
def export_tex(solid,args,src, dst, cfg):
    base_name=src.name
    ext="."+src.file_format.lower()
    if ext == ".":
        ext="."+src.filepath.lower().split(".")[-1]
    if ext == "":
        ext="."+src.filepath_raw.lower().split(".")[-1]
    if ext == "":
        ext="."+src.name.lower().split(".")[-1]
    pext=ext[1:]
    if pext in EXT_FORMAT_MAP:
        ext="."+EXT_FORMAT_MAP[pext]

    origin_file=bpy.path.abspath(src.filepath)
    print("Base texture name",base_name,"assets path",cfg.assets_path)
    output_file=os.path.join(cfg.assets_path,"Textures",base_name)+ext  
    print("Write texture in",output_file)

    output_parent=os.path.dirname(output_file)
        
    is_packed=src.packed_file
    dst.id = cfg.id_of(src)

    if cfg.need_update(src):       
    
        if not os.path.exists(output_parent):
            os.makedirs(output_parent)
          
        if is_packed:
            print(base_name,"is packed inside the blend file. It will be extracted in",output_file)
            with open(output_file, 'wb') as f:
                f.write(src.packed_file.data)
        else:
            print(origin_file,"will be copied in",output_file)
            import shutil
            if origin_file != output_file:
                shutil.copyfile(origin_file, output_file)            
        
        if not ext==".dds" and cfg.textures_to_dds and DDS_SUPPORT: 
            print("Convert to DDS")
              
            expected_mipmaps= 1 + int(math.ceil(math.log(src.size[0] if src.size[0] >src.size[1] else src.size[1]) / math.log(2)))  
            dds_file=os.path.join(cfg.assets_path,"Textures",base_name)+".dds"    
       #     command="\
        #    "+IMAGEMAGICK_CONVERT_PATH+"\
         #    -format dds \
          #   -filter Mitchell \
           #  -define dds:compression=dxt5 \
            # -define dds:mipmaps="+str(expected_mipmaps)+" \
            # "+output_file+" "+dds_file
            dds_args=args
            format =None
            if "dds{" in dds_args:
                dds_args=dds_args[dds_args.index("dds{")+4:]
                dds_args=dds_args[:dds_args.index("}")]
                dds_args=dds_args.split(",")
                for dds_arg in dds_args:
                    key,value=dds_arg.split("=")
                    key=key.strip()
                    if key=="solid" and solid and value!=None:      
                        format=value.replace("'","").strip()
                        break
                    elif key=="alpha" and not solid and value!=None:
                        format=value.replace("'","").strip()

            if format==None:
                format="UNCOMPRESSED"

            format=format.upper()
           
            if format=="ATI2" or format=="3DC":
                format="ATI_3DC"
            elif format=="DXT1" or format=="DXT3" or format=="DXT5":
                format="S3TC_"+format
            elif format=="UNCOMPRESSED":
                format="ARGB8"

            command=DDS_WRITER_PATH+" --use_lwjgl --gen-mipmaps --format "+format+" --in "+output_file+"  --out "+dds_file
            print("Run",command)
            print(subprocess.getoutput(command))
            os.remove(output_file)
    else:
        print(base_name,"already up to date")
    if cfg.textures_to_dds and DDS_SUPPORT:  ext=".dds"
    dst.rpath = "Textures/"+base_name+ext
    print("Set rpath to", dst.rpath)
        
    # TODO use md5 (hashlib.md5().update(...)) to name or to check change ??
    # TODO If the texture has a scale and/or offset, then export a coordinate transform.
    # uscale = textureSlot.scale[0]
    # vscale = textureSlot.scale[1]
    # uoffset = textureSlot.offset[0]
    # voffset = textureSlot.offset[1]


# TODO redo Light, more clear definition,...
def export_light(src, dst, cfg):
    dst.id = cfg.id_of(src)
    dst.name = src.name
    kind = src.type
    if kind == 'SUN' or kind == 'AREA' or kind == 'HEMI':
        dst.kind = f3b.datas_pb2.Light.directional
    elif kind == 'POINT':
        dst.kind = f3b.datas_pb2.Light.point        
    elif kind == 'SPOT':
        dst.kind = f3b.datas_pb2.Light.spot
        dst.spot_angle.max = src.spot_size * 0.5
        dst.spot_angle.linear.begin = (1.0 - src.spot_blend)

    
    processed=False
    if src.node_tree!=None and len(src.node_tree.nodes)>0: 
        for node in src.node_tree.nodes:
            if node.type == "EMISSION":
                cnv_color(node.inputs[0].default_value, dst.color)
                if kind=="POINT":
                        dst.radial_distance.max=node.inputs[1].default_value
                dst.intensity = 1 #node.inputs[1].default_value
                dst.cast_shadow = src.cycles.cast_shadow
                processed=True                
                
    if not processed:         
        dst.cast_shadow = getattr(src, 'use_shadow', False)
        cnv_color(src.color, dst.color)
        dst.intensity = src.energy
        dst.radial_distance.max = src.distance
        if hasattr(src, 'falloff_type'):
            falloff = src.falloff_type
            if falloff == 'INVERSE_LINEAR':
                dst.radial_distance.max = src.distance
                dst.radial_distance.inverse.scale = 1.0
            elif falloff == 'INVERSE_SQUARE':
                dst.radial_distance.max = src.distance  # math.sqrt(src.distance)
                dst.radial_distance.inverse_square.scale = 1.0
            elif falloff == 'LINEAR_QUADRATIC_WEIGHTED':
                if src.quadratic_attenuation == 0.0:
                    dst.radial_distance.max = src.distance
                    dst.radial_distance.inverse.scale = 1.0
                    dst.radial_distance.inverse.constant = 1.0
                    dst.radial_distance.inverse.linear = src.linear_attenuation
                else:
                    dst.radial_distance.max = src.distance
                    dst.radial_distance.inverse_square.scale = 1.0
                    dst.radial_distance.inverse_square.constant = 1.0
                    dst.radial_distance.inverse_square.linear = src.linear_attenuation
                    dst.radial_distance.inverse_square.linear = src.quadratic_attenuation
        if getattr(src, 'use_sphere', False):
            dst.radial_distance.linear.end = 1.0


def export_all_skeletons(scene, data, cfg):
    for obj in scene.objects:
        if obj.type == 'ARMATURE':
            src_skeleton = obj.data
            # src_skeleton = obj.pose
            if cfg.need_update(src_skeleton):
                dst_skeleton = data.skeletons.add()
                export_skeleton(src_skeleton, dst_skeleton, cfg)
            add_relation_raw(data.relations, f3b.datas_pb2.TObject.__name__, cfg.id_of(obj), f3b.datas_pb2.Skeleton.__name__, cfg.id_of(src_skeleton), cfg)


def export_skeleton(src, dst, cfg):
    dst.id = cfg.id_of(src)
    dst.name = src.name
    # for src_bone in armature.pose.bones:
    for src_bone in src.bones:
        dst_bone = dst.bones.add()
        dst_bone.id = cfg.id_of(src_bone)
        dst_bone.name = src_bone.name

        # retreive transform local to parent
        boneMat = src_bone.matrix_local
        if src_bone.parent:
            boneMat = src_bone.parent.matrix_local.inverted() * src_bone.matrix_local
        loc, quat, sca = boneMat.decompose()

        # Can't use armature.convert_space
        # boneMat = armature.convert_space(pose_bone=src_bone, matrix=src_bone.matrix, from_space='POSE', to_space='LOCAL_WITH_PARENT')
        # loc, quat, sca = boneMat.decompose()

        cnv_scale(sca, dst_bone.scale)
        cnv_translation(loc, dst_bone.translation)
        # cnv_scale(loc, transform.translation)
        cnv_rotation(quat, dst_bone.rotation)
        # cnv_quatZupToYup(quat, transform.rotation)
        # cnv_quat(quat, transform.rotation)
        if src_bone.parent:
            rel = dst.bones_graph.add()
            rel.ref1 = cfg.id_of(src_bone.parent)
            rel.ref2 = dst_bone.id


def export_all_actions(scene, dst_data, cfg):
    fps = max(1.0, float(scene.render.fps))
#    for action in bpy.data.actions:
    frame_current = scene.frame_current
    frame_subframe = scene.frame_subframe
    for obj in scene.objects:
        if obj.animation_data:
            action_current = obj.animation_data.action
            for tracks in obj.animation_data.nla_tracks:
                for strip in tracks.strips:
                    action = strip.action
                    if cfg.need_update(action):
                        #dst = dst_data.Extensions[f3b.animations_kf_pb2.animations_kf].add()
                        dst = dst_data.animations_kf.add()
                        # export_action(action, dst, fps, cfg)
                        export_obj_action(scene, obj, action, dst, fps, cfg)
                        # relativize_bones(dst, obj)
                    add_relation_raw(
                        dst_data.relations,
                        f3b.datas_pb2.TObject.__name__, cfg.id_of(obj),
                        f3b.animations_kf_pb2.AnimationKF.__name__, cfg.id_of(action),
                        cfg)
            obj.animation_data.action = action_current
    scene.frame_set(frame_current, frame_subframe)


def export_obj_action(scene, obj, src, dst, fps, cfg):
    """
    export action by sampling matrixes of obj over frame.
    side effects :
    * change the current action of obj
    * change the scene.frame (when looping over frame of the animation) by scene.frame_set
    """
    def to_time(frame):
        return int((frame * 1000) / fps)

    obj.animation_data.action = src
    dst.id = cfg.id_of(src)
    dst.name = src.name
    frame_start = int(src.frame_range.x)
    frame_end = int(src.frame_range.y + 1)
    dst.duration = to_time(max(1, float(frame_end - frame_start)))
    samplers = []
    if src.id_root == 'OBJECT':
        dst.target_kind = f3b.animations_kf_pb2.AnimationKF.tobject
        samplers.append(Sampler(obj, dst))
        if obj.type == 'ARMATURE':
            for i in range(0, len(obj.pose.bones)):
                samplers.append(Sampler(obj, dst, i))
    elif src.id_root == 'ARMATURE':
        dst.target_kind = f3b.animations_kf_pb2.AnimationKF.skeleton
        for i in range(0, len(obj.pose.bones)):
            samplers.append(Sampler(obj, dst, i))
    else:
        cfg.warning("unsupported id_roor => target_kind : " + src.id_root)
        return

    for f in range(frame_start, frame_end):
        scene.frame_set(f)
        for sampler in samplers:
            sampler.capture(to_time(f))


class Sampler:

    def __init__(self, obj, dst, pose_bone_idx=None):
        self.obj = obj
        self.pose_bone_idx = pose_bone_idx
        self.clip = dst.clips.add()
        if pose_bone_idx is not None:
            self.clip.sampled_transform.bone_name = self.obj.pose.bones[self.pose_bone_idx].name
            self.rest_bone = obj.data.bones[self.pose_bone_idx]
            self.rest_matrix_inverted = self.rest_bone.matrix_local.copy().inverted()
        self.previous_mat4 = None
        self.last_equals = None
        self.cnv_mat = bpy_extras.io_utils.axis_conversion(from_forward='-Y', from_up='Z', to_forward='Z', to_up='Y').to_4x4()

    def capture(self, t):
        if self.pose_bone_idx is not None:
            pbone = self.obj.pose.bones[self.pose_bone_idx]
            mat4 = pbone.matrix
            if pbone.parent:
                mat4 = pbone.parent.matrix.inverted() * mat4
        else:
            mat4 = self.obj.matrix_local
        if self.previous_mat4 is None or not equals_mat4(mat4, self.previous_mat4, 0.000001):
            if self.last_equals is not None:
                self._store(self.last_equals, self.previous_mat4)
                self.last_equals = None
            self.previous_mat4 = mat4.copy()
            self._store(t, mat4)
        else:
            self.last_equals = t

    def _store(self, t, mat4):
        loc, quat, sca = mat4.decompose()
        dst_clip = self.clip
        dst_clip.sampled_transform.at.append(t)
        dst_clip.sampled_transform.translation_x.append(loc.x)
        dst_clip.sampled_transform.translation_y.append(loc.z)
        dst_clip.sampled_transform.translation_z.append(-loc.y)
        dst_clip.sampled_transform.scale_x.append(sca.x)
        dst_clip.sampled_transform.scale_y.append(sca.z)
        dst_clip.sampled_transform.scale_z.append(sca.y)
        dst_clip.sampled_transform.rotation_w.append(quat.w)
        dst_clip.sampled_transform.rotation_x.append(quat.x)
        dst_clip.sampled_transform.rotation_y.append(quat.z)
        dst_clip.sampled_transform.rotation_z.append(-quat.y)

def equals_mat4(m0, m1, max_cell_delta):
    for i in range(0, 4):
        for j in range(0, 4):
            d = m0[i][j] - m1[i][j]
            if d > max_cell_delta or d < -max_cell_delta:
                return False
    return True

def export_obj_customproperties(src, dst_node, dst_data, cfg):
    keys = [k for k in src.keys() if not (k.startswith('_') or k.startswith('cycles'))]
    if len(keys) > 0:
        # custom_params = dst_data.Extensions[f3b.custom_params_pb2.custom_params].add()
        custom_params = dst_data.custom_params.add()
        custom_params.id = "params_" + cfg.id_of(src)
        for key in keys:
            param = custom_params.params.add()
            param.name = key
            value = src[key]
            if isinstance(value, bool):
                param.vbool = value
            elif isinstance(value, str):
                param.vstring = value
            elif isinstance(value, float):
                param.vfloat = value
            elif isinstance(value, int):
                param.vint = value
            elif isinstance(value, mathutils.Vector):
                cnv_vec3(value, param.vvec3)
            elif isinstance(value, mathutils.Quaternion):
                cnv_qtr(value, param.vqtr)
        add_relation(dst_data.relations, dst_node, custom_params, cfg)


import bpy
from bpy_extras.io_utils import ExportHelper

def update_path(self, context):
    context.scene.f3b.assets_path = self.assets_path


class f3bExporter(bpy.types.Operator, ExportHelper):
    """Export to f3b format"""
    bl_idname = "export_scene.f3b"
    bl_label = "Export f3b"
    filename_ext = ".f3b"

    # settings = bpy.props.PointerProperty(type=f3bSettingsScene)
    option_export_selection = bpy.props.BoolProperty(name = "Export Selection", description = "Export only selected objects", default = True)
    option_export_tangents = bpy.props.BoolProperty(name = "Export Tangents", description = "", default = False)
    option_remove_doubles = bpy.props.BoolProperty(name = "Remove Doubles", description = "", default = True)

    if DDS_SUPPORT:
        option_convert_texture_dds = bpy.props.BoolProperty(name = "Convert textures to dds", description = "", default = True)
    else: 
        option_convert_texture_dds=False
        
    def __init__(self):
        pass

    def execute(self, context):
        scene = context.scene
       
        assets_path =   os.path.dirname(self.filepath)
        print("Export in", assets_path)

        data = f3b.datas_pb2.Data()
        cfg = ExportCfg(is_preview=False, assets_path=assets_path,option_export_selection=self.option_export_selection,textures_to_dds=self.option_convert_texture_dds,export_tangents=self.option_export_tangents,remove_doubles=self.option_remove_doubles)
        export(scene, data, cfg)

        file = open(self.filepath, "wb")
        file.write(data.SerializeToString())
        file.close()

        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.active_object is not None
