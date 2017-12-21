import bpy,re,os;
from bpy_extras.io_utils import ExportHelper;
from bpy.props import StringProperty, BoolProperty, EnumProperty;
from bpy.types import Operator;


def empty(mat):
    for n in mat.node_tree.nodes:
        mat.node_tree.nodes.remove(n);
    

def emptySelected():
    for  obj in bpy.context.scene.objects:
        if (obj.select):
            for  i in range(0,len(obj.material_slots)):
                material=obj.material_slots[i].material;
                if(not material):
                    continue;
                empty(material);

if (__name__ == "__main__"):
    emptySelected();
    

