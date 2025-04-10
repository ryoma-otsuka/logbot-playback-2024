import os
import cv2
import glob
import random
import numpy as np
import matplotlib.pyplot as plt
import torch
from ultralytics import YOLO


def create_masked_image(
    results, 
    image, 
    mask_color=np.array([167, 121, 204]),
    alpha=0.5,
    show=True,
):

    # FIG_SIZE = (6.4, 4.8)
    FIG_SIZE = (6.4/1.5, 4.8/1.5)
    
    masks = results[0].masks  # fetch the mask
    selected_mask_index = 0
    null_mask = np.zeros((480, 640), dtype=np.float32)
    
    if masks is None:
        pixel_count = 0
        _image = image
        final_mask = null_mask
    else:
        # Mask data
        num_masks = masks.shape[0]
        mask_height, mask_width = masks.shape[1], masks.shape[2]
        
        # select one mask
        selected_mask = masks.data[selected_mask_index]
        
        # convert the mask data to numpy array
        if isinstance(selected_mask, torch.Tensor):
            selected_mask = selected_mask.cpu().numpy()
        
        # resize the mask
        resized_mask = cv2.resize(selected_mask, (image.shape[1], image.shape[0]))

        # Count the number of pixels recognized as the first class (bird)
        # Use a threshold of 0.5 to count pixels closer to 1 than to 0.5
        threshold = 0.5
        pixel_count = np.sum(resized_mask > threshold)
                    
        # apply color
        masked_image = image.copy()
        for c in range(3):  # 3 channel (RGB)
            masked_image[:, :, c] = np.where(
                resized_mask > threshold, 
                (1 - alpha) * masked_image[:, :, c] + alpha * mask_color[c] * (resized_mask > threshold), 
                masked_image[:, :, c]
            )
        _image = masked_image
        final_mask = resized_mask

    # show the resutls
    if show == True:
        print(f"Number of pixels in the class {selected_mask_index}: {pixel_count:,} ({pixel_count/(image.shape[1]*image.shape[0])*100:.1f}%)")
        fig, ax = plt.subplots(1, 1, figsize=FIG_SIZE)
        plt.imshow(cv2.cvtColor(_image, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        plt.show()
        # plt.close()
    
    return final_mask, pixel_count, fig


def vis_mask_in_black_and_white(mask):
    FIG_SIZE = (6.4/1.5, 4.8/1.5)
    fig, ax = plt.subplots(1, 1, figsize=FIG_SIZE)
    plt.imshow(cv2.cvtColor(mask, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.show()
    return fig
