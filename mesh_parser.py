# -*- coding:utf-8 -*-
"""
@Time: 2024/2/26 19:21
@author: RefineM
@file: mesh_parser.py

"""
import numpy as np
import xmltodict
from utils import get_scene_box


class SceneReader(object):
    def __init__(self, obj, xml):
        """get the scene information of the ROI by parsing .obj file and the metadata of mesh"""
        self.srs_origin = None
        self.srs = None
        self.mesh_obj_path = obj
        self.mesh_xml_path = xml
        self.get_srs()

    def get_srs(self):
        """get the reference coordinate system and center coordinate of the mesh"""
        with open(self.mesh_xml_path, 'r') as f:
            mesh_xml = xmltodict.parse(f.read())
        model_meta = mesh_xml['ModelMetadata']
        if 'SRS' not in model_meta.keys():
            self.srs = 'local system'
            self.srs_origin = [0.0, 0.0, 0.0]
        else:
            self.srs = mesh_xml['ModelMetadata']['SRS']
            ori = mesh_xml['ModelMetadata']['SRSOrigin'].split(",")
            self.srs_origin = [float(ori[0]), float(ori[1]), float(ori[2])]

    def get_3d_points(self):
        """
        get the vertex(world coordinates) of the mesh
        note: vertex/np.array(N,3)
        """
        _v = []
        for line in open(self.mesh_obj_path, 'r'):
            line = line.split(' ')
            if line[0] == 'v':
                # note: the vertex coordinates saved in obj are relative to the center point coordinates, so need to be restored
                v = np.array([float(line[1]) + self.srs_origin[0],
                              float(line[2]) + self.srs_origin[1],
                              float(line[3]) + self.srs_origin[2]])
                _v.append(v)
        vertex = np.stack(_v, axis=0)
        return vertex

    def centered_and_scaled_3d_points(self, output_path):
        """
        centered and scaled 3d points of the mesh, and then save it to a new path
        """
        _v = []
        _vt = []
        _f = []
        for line in open(self.mesh_obj_path, 'r'):
            line_split = line.split(' ')
            if line_split[0] == 'v':
                v = [float(line_split[1]) + self.srs_origin[0],
                     float(line_split[2]) + self.srs_origin[1],
                     float(line_split[3]) + self.srs_origin[2]]
                _v.append(v)
            if line_split[0] == 'vt':
                _vt.append(line)
            if line_split[0] == 'f':
                _f.append(line)

        p_world = _v
        p_world = np.array(p_world)  # np.array(n,3)
        scene = get_scene_box(p_world, tar_radius=1.0)
        center = scene['center']  # np.array(3,1)
        scale = scene['scale']  # float
        p_world = (p_world - center[:, 0]) * scale  # np.array(n,3)

        f = open(output_path, 'w')
        for i in range(np.size(p_world, 0)):
            f.write("v {0} {1} {2}\n".format(p_world[i, 0], p_world[i, 1], p_world[i, 2]))
        for i in _vt:
            f.write(i)
        for i in _f:
            f.write(i)
        f.close()
