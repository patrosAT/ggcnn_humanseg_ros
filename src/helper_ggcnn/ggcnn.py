from os import path
import cv2
import numpy as np
import scipy.ndimage as ndimage

import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.backend import set_session


MODEL_FILE = 'models/epoch_29_model.hdf5'
sess = tf.Session()
set_session(sess)
graph = tf.get_default_graph()
model = load_model(path.join(path.dirname(__file__), MODEL_FILE))


def process_depth_mask(mask, crop_size, out_size=300, crop_y_offset=0):
    imh, imw = mask.shape

    # Crop.
    mask_crop = mask[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                     (imw - crop_size) // 2:(imw - crop_size) // 2 + crop_size]

    mask_crop = mask_crop[1:-1, 1:-1]
    mask_crop = cv2.resize(mask_crop, (out_size, out_size), cv2.INTER_AREA)

    return mask_crop


def process_depth_image(depth, crop_size, out_size=300, crop_y_offset=0):
    imh, imw = depth.shape

    # Crop.
    depth_crop = depth[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                       (imw - crop_size) // 2:(imw - crop_size) // 2 + crop_size]

    # Inpaint --> OpenCV inpainting does weird things at the border.
    depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
    depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)
    depth_crop[depth_nan_mask==1] = 0

    # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
    depth_scale = np.abs(depth_crop).max()
    depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.
    depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

    # Back to original size and value range.
    depth_crop = depth_crop[1:-1, 1:-1]
    depth_crop = depth_crop * depth_scale
    depth_crop = cv2.resize(depth_crop, (out_size, out_size), cv2.INTER_AREA)
    
    return depth_crop


def predict(depth, mask=None, crop_size=300, out_size=300, crop_y_offset=0, filters=(2.0, 1.0, 1.0)):

    # Inference
    depth = np.clip((depth - depth.mean()), -1, 1)
    set_session(sess)
    with graph.as_default():
        pred_out = model.predict(depth.reshape((1, 300, 300, 1)))

    points_out = pred_out[0].squeeze()
    points_out[mask == 255] = 0

    # Calculate the angle map.
    cos_out = pred_out[1].squeeze()
    sin_out = pred_out[2].squeeze()
    ang_out = np.arctan2(sin_out, cos_out) / 2.0

    width_out = pred_out[3].squeeze() * 150.0  # Scaled 0-150:0-1

    # Filter the outputs.
    if filters[0]:
        points_out = ndimage.filters.gaussian_filter(points_out, filters[0])  # 3.0
    if filters[1]:
        ang_out = ndimage.filters.gaussian_filter(ang_out, filters[1])
    if filters[2]:
        width_out = ndimage.filters.gaussian_filter(width_out, filters[2])

    points_out = np.clip(points_out, 0.0, 1.0-1e-3)

    return points_out, ang_out, width_out, depth