# -*- coding:utf-8 -*-
"""
@Time: 2025/1/9 14:40
@author: Junfan W
@file: config.py

"""
class BaseConfig:
    # 数据集路径
    dataset_dir = r'.\dataset\isprs_01'
    # 是否要裁剪可见影像
    if_mask_crop = True
    # 裁剪后影像的目标尺寸
    tar_size_w = 1200
    tar_size_h = 1000
    # 是否将场景规范化到指定范围
    if_standardization = True
    # 将场景规范化的目标半径
    tar_radius = 1.0
