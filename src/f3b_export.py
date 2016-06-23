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

import f3b
import f3b.datas_pb2
import f3b.custom_params_pb2
import f3b.animations_kf_pb2
import f3b.physics_pb2
from . import helpers  # pylint: disable=W0406
from .math import *  # pylint: disable=W0406
from .conversions import *  # pylint: disable=W0406

import re,os
import subprocess
IMAGEMAGICK_CONVERT_PATH="convert"

DDS_SUPPORT=True

if DDS_SUPPORT:
    out=None
    
    try:
        out = subprocess.getoutput(IMAGEMAGICK_CONVERT_PATH+" -list format")
    except:
        pass
        
    if not out:
        print("Executable "+IMAGEMAGICK_CONVERT_PATH+" not found")
    else:    
        matched=False
        rx=re.compile('\s*DDS\*\s*rw.')
        for l in out.splitlines():
            if rx.match(l):
                matched=True
                break
            
        if not matched:
            print("The installed version of imagemagick doesn't support DDS writing")
            DDS_SUPPORT=False

if DDS_SUPPORT:
    print("DDS support is enabled!")
    



class ExportCfg:
    def __init__(self, is_preview=False, assets_path="/tmp",option_export_selection=False,textures_to_dds=False):
        self.is_preview = is_preview
        self.assets_path = bpy.path.abspath(assets_path)
        self._modified = {}
        self._ids = {}
        self.option_export_selection=option_export_selection
        self.textures_to_dds=textures_to_dds

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


# TODO avoid export obj with same id
# TODO optimize unify vertex with (same position, color, normal, texcoord,...)
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
            continue
        if cfg.need_update(obj):
            tobject = data.tobjects.add()
            tobject.id = cfg.id_of(obj)
            tobject.name = obj.name
            loc, quat, scale = obj.matrix_local.decompose()
            cnv_scale(scale, tobject.scale)
            # convert zup only for direct child of root (no parent)
            cnv_translation(loc, tobject.translation)
            # cnv_scale(helpers.rot_quat(obj), transform.rotation)
            # if obj.type == 'MESH':
            #    cnv_translation(obj.location, transform.translation)
            #    cnv_quatZupToYup(helpers.rot_quat(obj), transform.rotation)
            # if obj.parent is None:
            #     cnv_translation(obj.location, transform.translation)
            #     cnv_quatZupToYup(helpers.rot_quat(obj), transform.rotation)
            # else:
            if obj.type == 'MESH':
                # cnv_scale(helpers.rot_quat(obj), transform.rotation)
                # cnv_rotation(helpers.rot_quat(obj), transform.rotation)
                cnv_rotation(quat, tobject.rotation)
            elif obj.type == 'Armature':
                # cnv_rotation(helpers.rot_quat(obj), transform.rotation)
                cnv_rotation(quat, tobject.rotation)
            elif obj.type == 'LAMP':
                # rot = helpers.z_backward_to_forward(helpers.rot_quat(obj))
                rot = helpers.z_backward_to_forward(quat)
                cnv_quatZupToYup(rot, tobject.rotation)
            else:
                cnv_rotation(helpers.rot_quat(obj), tobject.rotation)
            if obj.parent is not None:
                #    tobject.parentId = cfg.id_of(obj.parent)
                add_relation_raw(data.relations, f3b.datas_pb2.TObject.__name__, cfg.id_of(obj.parent), f3b.datas_pb2.TObject.__name__, cfg.id_of(obj), cfg)
            export_obj_customproperties(obj, tobject, data, cfg)

def export_all_physics(scene, data, cfg):
    for obj in scene.objects:
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
    i = 1
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
        elif obj.type == 'LAMP':
            src_light = obj.data
            if cfg.need_update(src_light):
                dst_light = data.lights.add()
                export_light(src_light, dst_light, cfg)
                add_relation_raw(data.relations, f3b.datas_pb2.TObject.__name__, cfg.id_of(obj), f3b.datas_pb2.Light.__name__, dst_light.id, cfg)


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


def export_all_lights(scene, data, cfg):
    for obj in scene.objects:
        if obj.hide_render or (cfg.option_export_selection and not obj.select):
            continue
        if obj.type == 'LAMP':
            src_light = obj.data
            if cfg.need_update(src_light):
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
        
        positions = dst_mesh.vertexArrays.add()
        positions.attrib = f3b.datas_pb2.VertexArray.position
        positions.floats.step = 3
        positions=positions.floats.values
        
        normals = dst_mesh.vertexArrays.add()
        normals.attrib = f3b.datas_pb2.VertexArray.normal
        normals.floats.step = 3
        normals=normals.floats.values
        
        colors = None
        face_colors=None
        if len(src_mesh.tessface_vertex_colors)>=1:
            face_colors = src_mesh.tessface_vertex_colors.active.data
            colors = dst_mesh.vertexArrays.add()
            colors.attrib = f3b.datas_pb2.VertexArray.color
            colors.floats.step = 4
            colors=colors.floats.values

        indexes = dst_mesh.indexArrays.add()
        indexes.ints.step = 3
        indexes=indexes.ints.values
        latest_index=0
        index_order_map={}
        
        texcoords=[None]*min(9, len(src_mesh.tessface_uv_textures))
        texcoords_ids=[f3b.datas_pb2.VertexArray.texcoord,f3b.datas_pb2.VertexArray.texcoord2,f3b.datas_pb2.VertexArray.texcoord3,f3b.datas_pb2.VertexArray.texcoord4,f3b.datas_pb2.VertexArray.texcoord5,f3b.datas_pb2.VertexArray.texcoord6,f3b.datas_pb2.VertexArray.texcoord7,f3b.datas_pb2.VertexArray.texcoord8]
        
        armature = src_geometry.find_armature()
        if armature:
            print("Armature found")
            boneCount = []
            boneIndex = []
            boneWeight = []
            groupToBoneIndex = make_group_to_bone_index(armature, src_geometry, cfg)
        else: print("Armature not found")
       
  
        
        for i in range(0,len(texcoords)):
            texcoords[i]=dst_mesh.vertexArrays.add()
            texcoords[i].attrib=texcoords_ids[i]
            texcoords[i].floats.step = 2
            texcoords[i]=texcoords[i].floats.values
                 
           
        for i,f in enumerate(src_mesh.tessfaces):
            if material_index != f.material_index:
                continue
            is_smooth=f.use_smooth
            
            vertices=[]
            quad_ids=[] 
            quad_id=0
            for v in f.vertices:
                vertices.append(src_mesh.vertices[v])
                quad_ids.append(quad_id)
                quad_id+=1
                
            if len(vertices)==4: #Quad to tris
                vertices.append(vertices[0])
                quad_ids.append(quad_ids[0])
                vertices.append(vertices[2])
                quad_ids.append(quad_ids[2])
                vertices.append(vertices[3])
                quad_ids.append(quad_ids[3])
                
                del vertices[3]
                del quad_ids[3]
            
           
            for j,v in enumerate(vertices):
                j=quad_ids[j]
                positions.extend(cnv_toVec3ZupToYup(v.co))
                normals.extend(cnv_toVec3ZupToYup(v.normal if is_smooth else f.normal))
                indexes.append(latest_index)
               
                iom=index_order_map.get(v.index,None)
                if iom==None:
                    iom=[]
                    index_order_map[v.index]=iom
                iom.append(latest_index)
               
                latest_index+=1                
                for tx_id in range(0,len(texcoords)):
                    texcoords[tx_id].extend(src_mesh.tessface_uv_textures[tx_id].data[i].uv[j])
                             
            if face_colors!=None and colors!=None:
                fc = face_colors[f.index]
                colors.extend(fc.color1)
                colors.append(1.0)
                colors.extend(fc.color2)
                colors.append(1.0)
                colors.extend(fc.color3)
                colors.append(1.0)
                if len(f.vertices) == 4:
                    colors.extend(fc.color1)
                    colors.append(1.0)
                    colors.extend(fc.color3)
                    colors.append(1.0)
                    colors.extend(fc.color4)
                    colors.append(1.0)
                
                
        tangents_ids=[f3b.datas_pb2.VertexArray.tangent,f3b.datas_pb2.VertexArray.tangent2,f3b.datas_pb2.VertexArray.tangent3,f3b.datas_pb2.VertexArray.tangent4,f3b.datas_pb2.VertexArray.tangent5,f3b.datas_pb2.VertexArray.tangent6,f3b.datas_pb2.VertexArray.tangent7,f3b.datas_pb2.VertexArray.tangent8]
        
        #Find correspondent vertex from loops.
        ordered_vertfromloop=[0]*latest_index
        for face in src_mesh.polygons:
            for vert in [src_mesh.loops[i] for i in face.loop_indices]:
                i=vert.index
                iom=index_order_map.get(i,None)
                if iom!=None:
                    for ni in iom:
                        ordered_vertfromloop[ni]=vert

        #Tangents: Todo take in account flat shading.
        for k in range(0, len(src_mesh.tessface_uv_textures)):
            src_mesh.calc_tangents(uvmap=src_mesh.tessface_uv_textures[k].name)
            tangents = dst_mesh.vertexArrays.add()
            tangents.attrib = tangents_ids[k]
            tangents.floats.step = 4
            tangents=tangents.floats.values      
            for vert in ordered_vertfromloop:
                tan=cnv_toVec3ZupToYup(vert.tangent)
                btan=cnv_toVec3ZupToYup(vert.bitangent)
                tangents.extend(tan)           
                tangents.append(-1 if dot_vec3(cross_vec3( vert.normal, tan),btan) < 0  else 1)
               
        #Vertloop        
        for vert in ordered_vertfromloop:
            #Bone weights    
            if armature: 
                find_influence(src_mesh.vertices, vert.index, groupToBoneIndex, boneCount, boneIndex, boneWeight)
                    
        if armature:
            dst_skin = dst_mesh.skin
            dst_skin.boneCount.extend(boneCount)
            dst_skin.boneIndex.extend(boneIndex)
            dst_skin.boneWeight.extend(boneWeight)
        
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
    
def parseNode(input_node,input_type,dst_mat,input_label,cfg):                    
    #input_node=input.links[0].from_node
    #input_type=input_node.type                    
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
        export_tex(input_node.image,prop.texture,cfg)
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
                prop.vbool=False     
                prop=dst_mat.properties.add()
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
                print(input_type,"not supported")
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
                
             
                
                       
def export_tex(src, dst, cfg):
    base_name=os.path.splitext(src.name)[0]    
    ext="."+str(src.file_format).lower()
    origin_file=src.filepath   
    if origin_file.startswith("//"): 
        origin_file=bpy.path.abspath("//")+origin_file[2:]
    output_file=os.path.join(cfg.assets_path,"Textures",base_name)+ext  
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
        
        if cfg.textures_to_dds and DDS_SUPPORT: 
            print("Convert to DDS")
              
            dds_file=os.path.join(cfg.assets_path,"Textures",base_name)+".dds"    
            command="\
            "+IMAGEMAGICK_CONVERT_PATH+"\
             -format dds \
             -filter Mitchell \
             -define dds:compression=dxt5 \
             -define dds:mipmaps=5 \
             "+output_file+" "+dds_file
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
                dst.intensity = node.inputs[1].default_value
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




def make_group_to_bone_index(armature, src_geometry, cfg):
    groupToBoneIndex = []
    bones = armature.data.bones
    # Look up table for bone indices
    bones_table = [b.name for b in bones]

    for group in src_geometry.vertex_groups:
        groupName = group.name
        try:
            index = bones_table.index(group.name)
        except ValueError:
            index = -1  # bind to nothing if not found
        # for i in range(len(boneArray)):
        #     if (boneArray[i].name == groupName):
        #         index = i
        #         break
        groupToBoneIndex.append(index)
        if index < 0:
            cfg.warning("groupVertex can't be bind to bone %s -> %s" % (groupName, index))
    return groupToBoneIndex


def find_influence(vertices, index, groupToBoneIndex, boneCount, boneIndex, boneWeight):
    totalWeight = 0.0
    indexArray = []
    weightArray = []
    groups = sorted(vertices[index].groups, key=lambda x: x.weight, reverse=True)
    for el in groups:
        index = groupToBoneIndex[el.group]
        weight = el.weight
        if (index >= 0) and (weight > 0):
            totalWeight += weight
            indexArray.append(index)
            weightArray.append(weight)
    if totalWeight > 0:
        normalizer = 1.0 / totalWeight
        boneCount.append(len(weightArray))
        for i in range(0, len(weightArray)):
            boneIndex.append(indexArray[i])
            boneWeight.append(weightArray[i] * normalizer)
    else:
        # print("vertex without influence")
        boneCount.append(0)


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
            # mat4 = self.obj.convert_space(pbone, pbone.matrix, from_space='POSE', to_space='LOCAL')
            mat4 = pbone.matrix
            if pbone.parent:
                mat4 = pbone.parent.matrix.inverted() * mat4
        else:
            mat4 = self.obj.matrix_local
        # mat4 = self.cnv_mat * mat4
        # mat4 = self.obj.matrix_local.inverted() * mat4
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
        # print("capture : %r, %r, %r, %r, %r, %r " % (t, self.obj, self.pose_bone_idx, loc, quat, sca))
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
        # dst_clip.sampled_transform.translation_x.append(loc.x)
        # dst_clip.sampled_transform.translation_y.append(loc.y)
        # dst_clip.sampled_transform.translation_z.append(loc.z)
        # dst_clip.sampled_transform.scale_x.append(sca.x)
        # dst_clip.sampled_transform.scale_y.append(sca.y)
        # dst_clip.sampled_transform.scale_z.append(sca.z)
        # dst_clip.sampled_transform.rotation_w.append(quat.w)
        # dst_clip.sampled_transform.rotation_x.append(quat.x)
        # dst_clip.sampled_transform.rotation_y.append(quat.y)
        # dst_clip.sampled_transform.rotation_z.append(quat.z)


def equals_mat4(m0, m1, max_cell_delta):
    for i in range(0, 4):
        for j in range(0, 4):
            d = m0[i][j] - m1[i][j]
            if d > max_cell_delta or d < -max_cell_delta:
                return False
    return True
# ------------------------------------------------------------------------------
# def export_action(src, dst, fps, cfg):
#     def to_time(frame):
#         return int((frame * 1000) / fps)
#         # return float(f - src.frame_range.x) / frames_duration
#     dst.id = cfg.id_of(src)
#     dst.name = src.name
#     if src.id_root == 'OBJECT':
#         dst.target_kind = f3b.animations_kf_pb2.AnimationKF.tobject
#     elif src.id_root == 'ARMATURE':
#         dst.target_kind = f3b.animations_kf_pb2.AnimationKF.skeleton
#     else:
#         cfg.warning("unsupported id_roor => target_kind : " + src.id_root)
#         return
#
#     frame_start = int(src.frame_range.x)
#     frame_end = int(src.frame_range.y)
#     dst.duration = to_time(max(1, float(frame_end - frame_start)))
#     anims = fcurves_to_animTransforms(src.fcurves, cfg)
#     export_animTransforms(dst, anims, frame_start, frame_end + 1, to_time)
#
#
# def fcurves_to_animTransforms(fcurves, cfg):
#     import re
#
#     p = re.compile(r'pose.bones\["([^"]*)"\]\.(.*)')
#     anims = {}
#     for fcurve in fcurves:
#         anim_name = None
#         target_name = fcurve.data_path
#         m = p.match(target_name)
#         if m:
#             anim_name = m.group(1)
#             target_name = m.group(2)
#         if not (anim_name in anims):
#             anims[anim_name] = AnimTransform()
#         anim = anims[anim_name]
#         if target_name == "location":
#             anim.fcurve_t[fcurve.array_index] = fcurve
#         elif target_name == "scale":
#             anim.fcurve_s[fcurve.array_index] = fcurve
#         elif target_name == "rotation_quaternion":
#             anim.fcurve_r[fcurve.array_index] = fcurve
#         elif target_name == "rotation_euler":
#             anim.fcurve_r[fcurve.array_index] = fcurve
#         else:
#             cfg.warning("unsupported : " + target_name)
#             continue
#     return anims
#
#
# def export_animTransforms(dst, anims, frame_start, frame_end, to_time):
#     ats = []
#     for f in range(frame_start, frame_end):
#         ats.append(to_time(f))
#     for k in anims:
#         anim = anims[k]
#         if not anim.is_empty():
#             clip = dst.clips.add()
#             if k is not None:
#                 clip.sampled_transform.bone_name = k
#             clip.sampled_transform.at.extend(ats)
#             anim.to_clip(clip, frame_start, frame_end)
#
#
# class AnimTransform:
#     def __init__(self):
#         # x, y, z, w
#         self.fcurve_t = [None, None, None]  # x,y,z
#         self.fcurve_r = [None, None, None, None]  # w,x,y,z
#         self.fcurve_s = [None, None, None]  # x,y,z
#
#     def is_empty(self):
#         b = True
#         for v in self.fcurve_t:
#             b = b and (v is None)
#         for v in self.fcurve_r:
#             b = b and (v is None)
#         for v in self.fcurve_s:
#             b = b and (v is None)
#         return b
#
#     def sample(self, frame_start, frame_end, coeff, fcurve):
#         b = []
#         if fcurve is not None:
#             for f in range(frame_start, frame_end):
#                 b.append(fcurve.evaluate(f) * coeff)
#         return b
#
#     def cnv_translation(self, frame_start, frame_end):
#         return (
#             self.sample(frame_start, frame_end, 1, self.fcurve_t[0]),
#             self.sample(frame_start, frame_end, 1, self.fcurve_t[2]),
#             self.sample(frame_start, frame_end, -1, self.fcurve_t[1])
#         )
#
#     def cnv_scale(self, frame_start, frame_end):
#         return (
#             self.sample(frame_start, frame_end, 1, self.fcurve_s[0]),
#             self.sample(frame_start, frame_end, 1, self.fcurve_s[2]),
#             self.sample(frame_start, frame_end, 1, self.fcurve_s[1])
#         )
#
#     def cnv_rotation(self, frame_start, frame_end):
#         qx = []
#         qy = []
#         qz = []
#         qw = []
#         if (self.fcurve_r[3] is not None) and self.fcurve_r[3].data_path.endswith("rotation_quaternion"):
#             qw = self.sample(frame_start, frame_end, 1, self.fcurve_r[0])
#             qx = self.sample(frame_start, frame_end, 1, self.fcurve_r[1])
#             qy = self.sample(frame_start, frame_end, 1, self.fcurve_r[3])
#             qz = self.sample(frame_start, frame_end, -1, self.fcurve_r[2])
#         elif (self.fcurve_r[0] is not None) and self.fcurve_r[0].data_path.endswith("rotation_euler"):
#             # TODO use order of target
#             for f in range(frame_start, frame_end):
#                 eul = mathutils.Euler()
#                 eul.x = self.fcurve_r[0].evaluate(f)
#                 eul.y = self.fcurve_r[1].evaluate(f)
#                 eul.z = self.fcurve_r[2].evaluate(f)
#                 q = eul.to_quaternion()
#                 qw.append(q.w)
#                 qx.append(q.x)
#                 qy.append(q.z)
#                 qz.append(-q.y)
#         return (qw, qx, qy, qz)
#
#     def to_clip(self, dst_clip, frame_start, frame_end):
#         t = self.cnv_translation(frame_start, frame_end)
#         dst_clip.sampled_transform.translation_x.extend(t[0])
#         dst_clip.sampled_transform.translation_y.extend(t[1])
#         dst_clip.sampled_transform.translation_z.extend(t[2])
#
#         s = self.cnv_scale(frame_start, frame_end)
#         dst_clip.sampled_transform.scale_x.extend(s[0])
#         dst_clip.sampled_transform.scale_y.extend(s[1])
#         dst_clip.sampled_transform.scale_z.extend(s[2])
#
#         r = self.cnv_rotation(frame_start, frame_end)
#         dst_clip.sampled_transform.rotation_w.extend(r[0])
#         dst_clip.sampled_transform.rotation_x.extend(r[1])
#         dst_clip.sampled_transform.rotation_y.extend(r[2])
#         dst_clip.sampled_transform.rotation_z.extend(r[3])
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# def relativize_bones(dst_anim, obj):
#     if obj.type != 'ARMATURE':
#         return
#     src_skeleton = obj.data
#     if not(src_skeleton.bones):
#         return
#
#     def find_SampledTransform_by_bone_name(name):
#         for clip in dst_anim.clips:
#             if clip.sampled_transform.bone_name == name:
#                 return clip.sampled_transform
#         return None
#
#     def relativize_children_bone(parent, sampled_parent):
#         for child in parent.children:
#             sampled_child = find_SampledTransform_by_bone_name(child.name)
#             # children first
#             relativize_children_bone(child, sampled_child)
#             if sampled_child:
#                 relativize_scale(sampled_child, sampled_parent, parent)
#                 relativize_rotation(sampled_child, sampled_parent, parent)
#                 relativize_translation(sampled_child, sampled_parent, parent)
#                 # pass
#     for bone in src_skeleton.bones:
#         # only parent
#         if not(bone.parent):
#             relativize_children_bone(bone, find_SampledTransform_by_bone_name(bone.name))
#
#
# def relativize_rotation(sampled_child, sampled_parent, parent):
#     def to_quat(l, i):
#         return mathutils.Quaternion((l.rotation_w[i], l.rotation_x[i], l.rotation_y[i], l.rotation_z[i]))
#     if not(sampled_child) or not(sampled_child.rotation_w):
#         return
#     lg = len(sampled_child.at)
#     pq = parent.matrix_local.to_quaternion()
#     # pq.normalize()
#     t = pq.z
#     pq.z = -pq.y
#     pq.y = t
#     #        if src_bone.parent:
#     #            boneMat = src_bone.parent.matrix_local.inverted() * src_bone.matrix_local
#     #        loc, quat, sca = boneMat.decompose()
#     for i in range(0, lg):
#         p = pq
#         if sampled_parent and sampled_parent.rotation_w:
#             p = to_quat(sampled_parent, i)
#             p.invert()
#             # p.normalize()
#         c = to_quat(sampled_child, i)
#         c = c * p
#         sampled_child.rotation_w[i] = c.w
#         sampled_child.rotation_x[i] = c.x
#         sampled_child.rotation_y[i] = c.y
#         sampled_child.rotation_z[i] = c.z
#
#
# def relativize_translation(sampled_child, sampled_parent, parent):
#     def r_axe1(lg, cl, pl):
#         if cl:
#             for i in range(0, lg):
#                 p = pl[i]
#                 c = cl[i]
#                 cl[i] = c - p
#
#     def r_axe2(lg, cl, pdef):
#         if cl:
#             for i in range(0, lg):
#                 c = cl[i]
#                 cl[i] = c - pdef
#
#     def r_axe(lg, cs, ps, axe, pdef):
#         if hasattr(sampled_parent, 'axe'):
#             r_axe1(lg, getattr(cs, axe), getattr(ps, axe))
#         else:
#             r_axe2(lg, getattr(cs, axe), pdef)
#
#     lg = len(sampled_child.at)
#     ploc = parent.matrix_local.to_translation()
#     r_axe(lg, sampled_child, sampled_parent, 'translation_x', ploc.x)
#     r_axe(lg, sampled_child, sampled_parent, 'translation_y', ploc.z)
#     r_axe(lg, sampled_child, sampled_parent, 'translation_z', -ploc.y)
#
#
# def relativize_scale(sampled_child, sampled_parent, parent):
#     def r_axe1(lg, cl, pl):
#         if cl:
#             for i in range(0, lg):
#                 p = pl[i]
#                 c = cl[i]
#                 cl[i] = c / p
#
#     def r_axe2(lg, cl, pdef):
#         if cl:
#             for i in range(0, lg):
#                 c = cl[i]
#                 cl[i] = c / pdef
#
#     def r_axe(lg, cs, ps, axe, pdef):
#         if hasattr(sampled_parent, 'axe'):
#             r_axe1(lg, getattr(cs, axe), getattr(ps, axe))
#         else:
#             r_axe2(lg, getattr(cs, axe), pdef)
#
#     lg = len(sampled_child.at)
#     pscale = parent.matrix_local.to_scale()
#     r_axe(lg, sampled_child, sampled_parent, 'scale_x', pscale.x)
#     r_axe(lg, sampled_child, sampled_parent, 'scale_y', pscale.z)
#     r_axe(lg, sampled_child, sampled_parent, 'scale_z', pscale.y)
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# def export_action(src, dst, fps, cfg):
#     import re
#
#     def to_time(frame):
#         return int((frame * 1000) / fps)
#         # return float(f - src.frame_range.x) / frames_duration
#     dst.id = cfg.id_of(src)
#     dst.name = src.name
#     if src.id_root == 'OBJECT':
#         dst.target_kind = f3b.animations_kf_pb2.AnimationKF.tobject
#     elif src.id_root == 'ARMATURE':
#         dst.target_kind = f3b.animations_kf_pb2.AnimationKF.skeleton
#     else:
#         cfg.warning("unsupported id_roor => target_kind : " + src.id_root)
#         return
#
#     frames_duration = max(0.001, float(src.frame_range.y - src.frame_range.x))
#     dst.duration = to_time(frames_duration)
#     clip = dst.clips.add()
#     p = re.compile(r'pose.bones\["([^"]*)"\]\.(.*)')
#     for fcurve in src.fcurves:
#         target_name = fcurve.data_path
#         dst_kf = None
#         dst_kfs_coef = 1.0
#         m = p.match(target_name)
#         if m:
#             clip.transforms.bone_name = m.group(1)
#             target_name = m.group(2)
#         if target_name == "location":
#             dst_kf, dst_kfs_coef = vec3_array_index(clip.transforms.translation, fcurve.array_index)
#         elif target_name == "scale":
#             dst_kf, dst_kfs_coef = vec3_array_index(clip.transforms.scale, fcurve.array_index)
#         elif target_name == "rotation_quaternion":
#             dst_kf, dst_kfs_coef = quat_array_index(clip.transforms.rotation, fcurve.array_index)
#         elif target_name == "rotation_euler":
#             cfg.warning("unsupported : rotation_euler , use rotation_quaternion")
#             continue
#         else:
#             cfg.warning("unsupported : " + target_name)
#             continue
#         if dst_kf is not None:
#             has_bezier = False
#             ats = []
#             # values should be in ref Yup Zforward
#             values = []
#             # parameter of interpolation
#             interpolations = []
#             for src_kf in fcurve.keyframe_points:
#                 ats.append(to_time(src_kf.co[0]))
#                 values.append(src_kf.co[1] * dst_kfs_coef)
#                 interpolations.append(cnv_interpolation(src_kf.interpolation))
#                 has_bezier = has_bezier or ('BEZIER' == src_kf.interpolation)
#             dst_kf.at.extend(ats)
#             dst_kf.value.extend(values)
#             dst_kf.interpolation.extend(interpolations)
#             # each interpolation segment as  x in [0,1], y in same ref as values[]
#             if has_bezier:
#                 kps = fcurve.keyframe_points
#                 for i in range(len(kps)):
#                     bp = dst_kf.bezier_params.add()
#                     if ('BEZIER' == kps[i].interpolation) and (i < (len(kps) - 1)):
#                         p0 = kps[i]
#                         p1 = kps[(i + 1)]
#                         seg_duration = p1.co[0] - p0.co[0]
#                         # print("kf co(%s) , left (%s), right(%s) : (%s, %s)" % ())
#                         bp.h0_x = (p0.handle_right[0] - p0.co[0]) / seg_duration
#                         bp.h0_y = p0.	handle_right[1] * dst_kfs_coef
#                         bp.h1_x = (p1.handle_left[0] - p0.co[0]) / seg_duration
#                         bp.h1_y = p1.handle_left[1] * dst_kfs_coef
#             # print("res dst_kf %r" % (dst_kf))
#
#
# def cnv_interpolation(inter):
#     if 'CONSTANT' == inter:
#         return f3b.animations_kf_pb2.KeyPoints.constant
#     elif 'BEZIER' == inter:
#         return f3b.animations_kf_pb2.KeyPoints.bezier
#     return f3b.animations_kf_pb2.KeyPoints.linear
#
#
# def vec3_array_index(vec3, idx):
#     "find the target vec3 and take care of the axis change (to Y up, Z forward)"
#     if idx == 0:
#         # x => x
#         return (vec3.x, 1.0)
#     if idx == 1:
#         # y => -z
#         return (vec3.z, -1.0)
#     if idx == 2:
#         return (vec3.y, 1.0)
#
#
# def quat_array_index(vec3, idx):
#     "find the target quat and take care of the axis change (to Y up, Z forward)"
#     if idx == 0:
#         return (vec3.w, 1.0)
#     if idx == 1:
#         return (vec3.x, 1.0)
#     if idx == 2:
#         return (vec3.z, -1.0)
#     if idx == 3:
#         return (vec3.y, 1.0)
# ------------------------------------------------------------------------------


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
    option_export_selection = bpy.props.BoolProperty(name = "Export Selection", description = "Export only selected objects", default = False)
    if DDS_SUPPORT:
        option_convert_texture_dds = bpy.props.BoolProperty(name = "Convert textures to dds", description = "", default = True)
    else: 
        option_convert_texture_dds=False
        
    def __init__(self):
        pass

    def execute(self, context):
        scene = context.scene
       
        assets_path =   os.path.dirname(self.filepath) #scene.f3b.assets_path
        print("Export in", assets_path)
        # originalFrame = scene.frame_current
        # originalSubframe = scene.frame_subframe
        # self.restoreFrame = False
        # self.beginFrame = scene.frame_start
        # self.endFrame = scene.frame_end
        # self.frameTime = 1.0 / (scene.render.fps_base * scene.render.fps)

        data = f3b.datas_pb2.Data()
        cfg = ExportCfg(is_preview=False, assets_path=assets_path,option_export_selection=self.option_export_selection,textures_to_dds=self.option_convert_texture_dds)
        export(scene, data, cfg)

        file = open(self.filepath, "wb")
        file.write(data.SerializeToString())
        file.close()

        # if (self.restoreFrame):
        #    scene.frame_set(originalFrame, originalSubframe)

        return {'FINISHED'}

    @classmethod
    def poll(cls, context):
        return context.active_object is not None
