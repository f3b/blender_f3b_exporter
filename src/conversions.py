import mathutils
def cnv_vec3(src, dst):
    # dst = f3b.math_pb2.Vec3()
    # dst.x = src.x
    # dst.y = src.y
    # dst.z = src.z
    dst.x = src[0]
    dst.y = src[1]
    dst.z = src[2]
    return dst
 
def cnv_vec4(src, dst):
    # dst = f3b.math_pb2.Vec3()
    # dst.x = src.x
    # dst.y = src.y
    # dst.z = src.z
    dst.x = src[0]
    dst.y = src[1]
    dst.z = src[2]
    dst.w = src[3]
    return dst
    
def cnv_vec2(src, dst):
    dst.x = src[0]
    dst.y = src[1]
    return dst


def cnv_translation(src, dst):
    # same as src.rotate(Quaternion((1,1,0,0))) # 90 deg CW axis X
    # src0 = src.copy()
    # q = mathutils.Quaternion((-1, 1, 0, 0))
    # q.normalize()
    # src0.rotate(q)
    # dst.x = src0[0]
    # dst.y = src0[1]
    # dst.z = src0[2]
    dst.x = src[0]
    dst.y = src[2]
    dst.z = -src[1]
    return dst


def cnv_scale(src, dst):
    dst.x = src[0]
    dst.y = src[2]
    dst.z = src[1]
    return dst


def cnv_toVec3ZupToYup(src):
    #dst = src.copy()
    #q = mathutils.Quaternion((1, 1, 0, 0))
    #q.normalize()
    #dst.rotate(q)
    dst = [src[0], src[2], -src[1]]
    return dst


def cnv_quatZupToYup(src, dst):
    # dst = f3b.math_pb2.Quaternion()
    src0 = src.copy()
    q = mathutils.Quaternion((-1, 1, 0, 0))
    q.normalize()
    src0.rotate(q)
    # orig = src
    # src = mathutils.Quaternion((-1, 1, 0, 0))
    # src.normalize()
    # src.rotate(orig)
    dst.w = src0.w  # [0]
    dst.x = src0.x  # [1]
    dst.y = src0.y  # [2]
    dst.z = src0.z  # [3]
    return dst


def cnv_rotation(src, dst):
    # dst = f3b.math_pb2.Quaternion()
    dst.w = src.w  # [0]
    dst.x = src.x  # [1]
    dst.y = src.z  # [2]
    dst.z = -src.y  # [3]
    return dst


def cnv_qtr(src, dst):
    # dst = f3b.math_pb2.Quaternion()
    dst.w = src.w  # [0]
    dst.x = src.x  # [1]
    dst.y = src.y  # [2]
    dst.z = src.z  # [3]
    return dst




def cnv_color(src, dst):
    dst.x = src[0]
    dst.y = src[1]
    dst.z = src[2]
    dst.w = 1.0 if len(src) < 4 else src[3]
    return dst
