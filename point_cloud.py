import sys
import os
from PIL import Image
import cv2
import numpy as np
# from open3d import read_point_cloud, draw_geometries
import open3d as o3d
from open3d.io import read_point_cloud
from open3d.visualization import draw_geometries

# Instructions on how to control the view:
# http://www.open3d.org/docs/release/tutorial/Basic/visualization.html

"""

-- Mouse view control --
  Left button + drag         : Rotate.
  Ctrl + left button + drag  : Translate.
  Wheel button + drag        : Translate.
  Shift + left button + drag : Roll.
  Wheel                      : Zoom in/out.

-- Keyboard view control --
  [/]          : Increase/decrease field of view.
  R            : Reset view point.
  Ctrl/Cmd + C : Copy current view status into the clipboard.
  Ctrl/Cmd + V : Paste view status from clipboard.

-- General control --
  Q, Esc       : Exit window.
  H            : Print help message.
  P, PrtScn    : Take a screen capture.
  D            : Take a depth capture.
  O            : Take a capture of current rendering settings.

  Ctrl + c     : Save view point as JSON to clipboard
  Ctrl + v     : Load view point from clipboard

"""

def generate_point_cloud_fn(rgb, depth):

    rgb = cv2.flip(rgb, 1)
    depth = cv2.flip(depth, 1)
    color_coverted = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    np_rgb = np.array(color_coverted)
    
    # Create an Open3D image from the numpy array
    rgb = o3d.geometry.Image(np_rgb)

    color_coverted = cv2.cvtColor(depth, cv2.COLOR_BGR2RGB)
    np_depth = np.array(color_coverted)
    
    # Create an Open3D image from the numpy array
    depth = o3d.geometry.Image(np_depth)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    rgb, depth,convert_rgb_to_intensity=False)

    # Camera intrinsic parameters built into Open3D for Prime Sense
    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)

    draw_geometries([pcd]
                    ,zoom=0.57999999999999985,
                    front=[ -0.079396103249738878, -0.18810111504642085, 0.97893525286765015 ],
                    lookat=[
                    3.8885851667839176e-05,
                    5.7830592379018295e-05,
                    0.00059226867597317323
                    ],
                    up=[ -0.031156232914671171, -0.981087363276261, -0.19104155247013882 ]
                    )


# ,zoom=0.57999999999999985,
#                                   front=[ -0.079396103249738878, -0.18810111504642085, 0.97893525286765015 ],
#                                   lookat=[
#                 3.8885851667839176e-05,
#                 5.7830592379018295e-05,
#                 0.00059226867597317323
#             ],
#                                   up=[ -0.031156232914671171, -0.981087363276261, -0.19104155247013882 ]
    
# {
#     "class_name" : "ViewTrajectory",
#     "interval" : 29,
#     "is_loop" : false,
#     "trajectory" : 
#     [
#         {
#             "boundingbox_max" : 
#             [
#                 0.00073619611783041843,
#                 0.00051137527583965234,
#                 0.00096746673807501793
#             ],
#             "boundingbox_min" : 
#             [
#                 -0.00051212169235527872,
#                 -0.00041151421349717391,
#                 0.00021707061387132853
#             ],
#             "field_of_view" : 60.0,
#             "front" : [ -0.079396103249738878, -0.18810111504642085, 0.97893525286765015 ],
#             "lookat" : 
#             [
#                 3.8885851667839176e-05,
#                 5.7830592379018295e-05,
#                 0.00059226867597317323
#             ],
#             "up" : [ -0.031156232914671171, -0.981087363276261, -0.19104155247013882 ],
#             "zoom" : 0.57999999999999985
#         }
#     ],
#     "version_major" : 1,
#     "version_minor" : 0
# }


