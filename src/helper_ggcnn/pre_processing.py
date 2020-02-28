import numpy as np
import cv2


class Preparations:
  
    def __init__(self):

        self.depth = None
        self.mask_body = None
        self.mask_hand = None
        self.dist_fg = None
        self.dist_bg = None
        self.dist_obj = None
        self.dist_ignore = None
        self.gripper_height = None


    # Internal function preparing the depth image
    def _prepare_depth(self):
        
        depth_bg = self.depth.copy()
        depth_bg[self.depth_nan == 1] = self.dist_bg
        depth_bg[self.depth < self.dist_fg] = self.dist_bg
        depth_bg[self.depth > self.dist_bg] = self.dist_bg
        
        return depth_bg


    # Internal function preparing the mask
    def _prepare_mask(self):
        
        mask = np.zeros(self.depth.shape, dtype=np.uint8)
        mask[self.mask_body != 0] = 255
        mask[self.mask_hand != 0] = 255
        mask[self.depth_nan == 1] = 0
        mask[self.depth > self.dist_bg] = 0

        return cv2.dilate(mask, np.ones((5,5),np.uint8), iterations = 2)

    # Public function returning the adapted depth image and mask
    def prepare_image_mask(self, depth, depth_nan, mask_body, mask_hand, dist_obj, dist_ignore, grip_height):

        self.depth = depth
        self.depth_nan = depth_nan
        self.mask_body = mask_body
        self.mask_hand = mask_hand
        self.dist_obj = dist_obj
        self.dist_ignore = dist_ignore
        self.grip_height = grip_height

        # Calculate planes
        if (self.dist_obj < self.dist_ignore):
            self.dist_bg = self.dist_obj + grip_height
        else:
            self.dist_bg = self.dist_ignore

        self.dist_fg = self.dist_bg - 2 * self.grip_height


        # Prepare images
        depth_adapt = self._prepare_depth()
        mask_adapt = self._prepare_mask()

        return depth_adapt, mask_adapt




