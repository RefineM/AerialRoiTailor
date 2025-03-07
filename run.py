# -*- coding:utf-8 -*-
"""
@Time: 2024/1/30 12:22
@author: RefineM
@file: run.py
"""
import json
import os.path
import numpy as np
from PIL import Image
from camera_parser import CameraReader
from mesh_parser import SceneReader
from utils import get_scene_box, select_target_images, get_target_img_path, move_images
from tqdm import tqdm
from config import BaseConfig


def pipeline():
    cfg = BaseConfig()
    """config"""
    # the paths of at file, obj file and mesh meta file exported by CC
    xml_path = cfg.dataset_dir + r"\AT.xml"
    obj_path = cfg.dataset_dir + r"\Model.obj"
    mesh_xml_path = cfg.dataset_dir + r"\metadata.xml"
    # the target folder to save the images that cover the ROI
    tar_folder = cfg.dataset_dir + r"\images"
    # the path of the text file to save the index of images that cover the ROI
    use_id_path = cfg.dataset_dir + r"\required_img_id.txt"
    # the path to save the cropped images
    crop_folder = cfg.dataset_dir + r"\images_crop"
    # the path of the json file to save the intrinsic and extrinsic parameters of the virtual cameras
    json_file_path = cfg.dataset_dir + r".\transforms.json"
    # the path to save the resized obj file
    resized_obj_path = cfg.dataset_dir + r"\Model_resized.obj"

    """get camera and scene information"""
    cam_info = CameraReader(xml_path)
    scene_info = SceneReader(obj_path, mesh_xml_path)
    cam_in = cam_info.get_cam_intrinsic()
    cam_ex = cam_info.get_cam_extrinsic()
    ori_img_path = cam_info.get_image_path()
    p_world = scene_info.get_3d_points()
    scene = get_scene_box(p_world, cfg.tar_radius)
    scene_info.centered_and_scaled_3d_points(resized_obj_path)

    """get the index of target images that cover the ROI"""
    if os.path.isfile(use_id_path):
        use_img_id = []
        for line in open(use_id_path, "r"):
            use_img_id.append(int(line.split('\n')[0]))
    else:
        use_img_id = select_target_images(cam_ex["rot_c2w"], cam_ex["cam_xyz"],
                                 p_world,
                                 cam_in['cam_fov'] / 2,
                                 cover_ratio=0.05)
        f = open(use_id_path, "w")
        for i in use_img_id:
            f.write(str(i) + '\n')
        f.close()

    """move target images to target folder"""
    old_path, new_path = get_target_img_path(ori_img_path, use_img_id, tar_folder)
    move_images(old_path, new_path)

    """get metadata of the target images"""
    file_path = []
    for i in range(len(new_path)):
        file_path.append("images/" + new_path[i].split("\\")[-1])

    intrinsic_matrix = cam_in["cam_intrinsic"][use_img_id]  # c2p (n,3,3)
    transform_matrix = cam_ex["cam_extrinsic_c2w"][use_img_id]  # c2w (n,4,4)
    w = np.array(cam_in["imgs_w"])[use_img_id]
    h = np.array(cam_in["imgs_h"])[use_img_id]

    """crop the ROI from images"""
    if cfg.if_mask_crop:
       crop_file_path = []
       print("crop images")
       for img_idx in tqdm(range(len(new_path))):
           c2p = intrinsic_matrix[img_idx, :, :]  # c2p (3,3)
           w2c = np.linalg.inv(transform_matrix[img_idx, :-1, :-1])  # w2c (3,3)
           trans = transform_matrix[img_idx, :-1, 3].reshape(1, 3)  # (3)
           trans = np.tile(trans, reps=(p_world.shape[0], 1))  # (N 3)

           # get the pixel coordinate of mesh vertice
           p_cam = (p_world - trans) @ w2c.T  # (N,3) @ (3,3) - (N,3)
           p_pixel = p_cam @ c2p.T  # (N,3) @ (3,3)
           p_pixel[:, 0] /= p_cam[:, -1]  # np(N,3)
           p_pixel[:, 1] /= p_cam[:, -1]  # np(N,3)
           p_pixel = p_pixel[:, :-1]  # np(N,2)

           # get bbox of ROI
           bbox_min = np.clip(np.min(p_pixel, axis=0), a_min=[0, 0], a_max=[w[img_idx]-1, h[img_idx]-1])
           bbox_max = np.clip(np.max(p_pixel, axis=0), a_min=[0, 0], a_max=[w[img_idx]-1, h[img_idx]-1])
           bbox_min_x, bbox_min_y = int(bbox_min[0]), int(bbox_min[1])
           bbox_max_x, bbox_max_y = int(bbox_max[0]), int(bbox_max[1])
           box = (bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y)

           # make sure the size of ROI of each image is the same
           if bbox_min_x + cfg.tar_size_w >= w[img_idx]:
              bbox_min_x = w[img_idx] - cfg.tar_size_w
           if bbox_min_y + cfg.tar_size_h >= h[img_idx]:
              bbox_min_y = h[img_idx] - cfg.tar_size_h
           box = (bbox_min_x, bbox_min_y, bbox_min_x + cfg.tar_size_w, bbox_min_y + cfg.tar_size_h)

           # crop image according to the bbox
           ori_path = os.path.join(cfg.dataset_dir, file_path[img_idx])
           img = Image.open(ori_path)
           region = img.crop(box)
           save_path = os.path.join(crop_folder, file_path[img_idx].split("/")[-1])
           crop_file_path.append(save_path)
           region.save(save_path)

           # update the principal points (cx,cy)
           intrinsic_matrix[img_idx, 0, 2] -= bbox_min_x
           intrinsic_matrix[img_idx, 1, 2] -= bbox_min_y
           # update the size of images
           w[img_idx] = cfg.tar_size_w
           h[img_idx] = cfg.tar_size_h

       file_path = crop_file_path

    """centered and scaled the scene"""
    if cfg.if_standardization:
        # update the trans vector of c2w matrix
        transform_matrix[:, :3, 3] -= scene["center"].flatten()
        transform_matrix[:, :3, 3] *= scene["scale"]
        # update bounding box
        scene["bounding_box"] -= scene["center"]
        scene["bounding_box"] *= scene["scale"]
        # update center to [0, 0, 0]
        scene["center"] = np.array([0, 0, 0])
        # update radius of the scene to target radius
        scene["radius"] = cfg.tar_radius

    """save json"""
    out = {
        "camera_mode": cam_in["cam_type"],
        "camera_orientation": cam_in["cam_orient"],
        "aabb_scale": np.exp2(np.rint(np.log2(scene["radius"]))),
        "aabb_range": scene["bounding_box"].tolist(),
        "sphere_center": scene["center"].tolist(),
        "sphere_radius": scene["radius"],
        "frames": []
    }

    for i in range(len(new_path)):
        frame = {
            "file_path": "images_crop/" + file_path[i].split("\\")[-1],
            "intrinsic_matrix": intrinsic_matrix[i].tolist(),
            "transform_matrix": transform_matrix[i].tolist(),
            "w": w[i].tolist(),
            "h": h[i].tolist()
        }
        out["frames"].append(frame)

    with open(json_file_path, "w") as outputfile:
        json.dump(out, outputfile, indent=2)

if __name__ == "__main__":
    pipeline()