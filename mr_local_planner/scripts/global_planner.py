#!/usr/bin/env python3

# ---------------------------------------------------------------------------------------------------------------------
# Based on initial implementation from F1tenth Team 2020 TU Wien: Thomas Pintaric et al. and major modified Version 2021: Andreas Brandstaetter et al. (see https://github.com/CPS-TUWien/f1tenth_maps)
# SPDX-License-Identifier: GPL-3.0-or-later
# ---------------------------------------------------------------------------------------------------------------------

import os
import argparse

import tf2_ros
import yaml
import numpy as np
import scipy.interpolate as si
#import cmapy
#import cv2
from skimage import io, morphology, img_as_ubyte
from scipy import ndimage

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float64

class Map:
    def __init__(self, width, height, resolution, data, origin=[0,0]):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.data = data
        self.origin = np.array(origin)

class GlobalPlanner(object):
    def __init__(self):
        rospy.loginfo("GlobalPlanner: init")
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.path_publisher = rospy.Publisher('/waypoints', Path, queue_size=1)
        
        self.initpose = PoseWithCovarianceStamped()
        self.initpose.header.stamp = rospy.get_rostime()
        self.initpose.header.frame_id = "map"
        self.initpose.pose.pose.position.x = 0
        self.initpose.pose.pose.position.y = 0

        self.export_debug_images = True
        self.output_path = "/tmp/ros_global_planner_debug"
        self.output_filename_root, _ = os.path.splitext(self.output_path)

        self.occupied_thresh = 0.5 # this is normalized (0 to 1)

        self.erosion_value = 5 #12
        self.d_value = 25
        self.use_blurred_factor = True
        self.sample_every = 10 #20

        self.race_line_points = np.zeros(2);
        self.grid_starting_position = np.zeros(2);
        self.world_starting_position = np.zeros(2);
        self.grid_goal_position = np.zeros(2);
        self.world_goal_position = np.zeros(2);
        self.resolution = 1; 
        self.origin = np.array([0.0, 0.0]);
        self.image_shape = np.array([10, 10]);
        self.map_init = False
        self.map = None

        rospy.Subscriber('/goal', PoseStamped, self.goal_callback, queue_size = 1)
        rospy.Subscriber('/pose', PoseWithCovarianceStamped, self.initialpose_callback, queue_size = 1)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size = 1)

        rospy.loginfo("GlobalPlanner: init done")


    # ===================================================================================================================
    def compute_distance_transform(self, starting_position, goal_position, erosion_value = 0, forward_direction = True):

        if erosion_value == 0:
            binary_image = np.copy(self.binary_image)
            filename_extra = "full"
        else:
            binary_image = np.copy(self.binary_image)
            binary_image = morphology.binary_erosion(binary_image, footprint=morphology.footprints.disk(radius=erosion_value, dtype=np.bool))
            filename_extra = "eroded-by-{}".format(erosion_value)

        distances = np.zeros_like(binary_image, np.float)

        mask = np.zeros_like(binary_image, np.bool)
        mask_start = np.zeros_like(binary_image, np.bool)
        mask_target = np.zeros_like(binary_image, np.bool)
        finish_line = np.zeros_like(binary_image, np.bool)
        mask[goal_position[1], goal_position[0]] = True
        mask_start[goal_position[1], goal_position[0]+2] = True
        mask_target[goal_position[1], goal_position[0]-2] = True

        rospy.loginfo("Computing distance to goal [{}, {}] from position [{}, {}] ({} direction with erosion {})...".format(
            goal_position[0], goal_position[1], starting_position[0], starting_position[1], "forward" if forward_direction else "backward", erosion_value))

        current_distance = 0.0
        while True:
            current_distance = current_distance + 1.0
            dilated_mask = morphology.binary_dilation(mask, footprint=morphology.footprints.square(width=3, dtype=np.bool))
            new_pixels = np.logical_and(binary_image, np.logical_xor(mask, dilated_mask))
            x,y = np.nonzero(new_pixels)
            if len(x) == 0:
                break
            distances[x,y] = current_distance
            mask = np.logical_or(mask, new_pixels)
            #if(np.mod(current_distance, 100) == 0):
            #    rospy.loginfo("distance = {}".format(current_distance))

        distances[np.nonzero(finish_line)] = (current_distance)
        distances = distances * self.resolution
        normalized_distances = distances / np.amax(distances.flatten())
        drivable_area = np.logical_or(mask, finish_line)

        #For debugging:
        if self.export_debug_images:
            io.imsave(self.output_filename_root + '.' + filename_extra + '.binary_image.png', img_as_ubyte(binary_image), check_contrast=False)
            io.imsave(self.output_filename_root + '.' + filename_extra + '.normalized_distances.png', img_as_ubyte(normalized_distances), check_contrast=False)

        return drivable_area, distances, normalized_distances, mask_start, mask_start, mask_target, mask_target

    # ===================================================================================================================
    def compute_distance_transform_smoothed(self, starting_position, goal_position, erosion_value = 0, forward_direction = True):

        drivable_area, distances, normalized_distances, mask_start, mask_start_small, mask_target, mask_target_small = \
            self.compute_distance_transform(starting_position=starting_position, goal_position=goal_position, erosion_value=erosion_value, forward_direction=forward_direction)

        distances_blurred = distances
        t_distance = np.amax(distances.flatten())

        # dilate race area using maximum filter, s.t. gaussian filter does not introduce values from outside the race area
        distances_extended = distances_blurred * drivable_area + ndimage.maximum_filter(distances_blurred, size=10) * (1-drivable_area)
        # split sections around finish line, s.t. gaussian filter does not introduce values from the other side of the finish line
        distances_extended_target_blank = np.full_like(self.binary_image, t_distance, np.float) * mask_target + distances_extended * (1-mask_target)
        distances_extended_start_blank = np.full_like(self.binary_image, 0, np.float) * mask_start_small + distances_extended * (1-mask_start_small)

        distances_blurred_target_blank = ndimage.gaussian_filter(distances_extended_target_blank, sigma=3) * drivable_area
        distances_blurred_start_blank = ndimage.gaussian_filter(distances_extended_start_blank, sigma=3) * drivable_area
        distances_blurred = distances_blurred_target_blank * mask_start + distances_blurred_start_blank * (1-mask_start)

        # dilate race area using maximum filter, s.t. gaussian filter does not introduce values from outside the race area
        distances_extended = distances_blurred * drivable_area + ndimage.maximum_filter(distances_blurred, size=10) * (1-drivable_area)
        # split sections arount finish line, s.t. gaussian filter does not introduce values from the other side of the finish line
        distances_extended_target_blank = np.full_like(self.binary_image, t_distance, np.float) * mask_target + distances_extended * (1-mask_target)
        distances_extended_start_blank = np.full_like(self.binary_image, 0, np.float) * mask_start_small + distances_extended * (1-mask_start_small)

        distances_blurred_target_blank = ndimage.gaussian_filter(distances_extended_target_blank, sigma=3) * drivable_area
        distances_blurred_start_blank = ndimage.gaussian_filter(distances_extended_start_blank, sigma=3) * drivable_area
        distances_blurred = distances_blurred_target_blank * mask_start + distances_blurred_start_blank * (1-mask_start)


        distances_extended = distances * drivable_area + np.full_like(self.binary_image, t_distance, np.float) * (1-drivable_area)
        # split sections arount finish line, s.t. gaussian filter does not introduce values from the other side of the finish line
        distances_extended_target_blank = np.full_like(self.binary_image, t_distance, np.float) * mask_target + distances_extended * (1-mask_target)
        distances_extended_start_blank = np.full_like(self.binary_image, 0, np.float) * mask_start_small + distances_extended * (1-mask_start_small)

        distances_blurred_border_target_blank = ndimage.gaussian_filter(distances_extended_target_blank, sigma=5) * drivable_area
        distances_blurred_border_start_blank = ndimage.gaussian_filter(distances_extended_start_blank, sigma=5) * drivable_area
        distances_blurred_border = distances_blurred_border_target_blank * mask_start + distances_blurred_border_start_blank * (1-mask_start)

        if self.use_blurred_factor:
            distances = distances + distances_blurred * 0.1 + distances_blurred_border * 0.1

        distances = distances * self.resolution
        normalized_distances = distances / np.amax(distances.flatten())

        #For debugging:
        if self.export_debug_images:
            io.imsave(self.output_filename_root + '.normalized_distances_blurred.png', img_as_ubyte(normalized_distances), check_contrast=False)

        return drivable_area, distances, normalized_distances, mask_start, mask_start_small, mask_target, mask_target_small

    # ===================================================================================================================

    def compute_raceline(self, starting_position, goal_position, current_drivable_area, full_drivable_area, normalized_target_distances, filename_part):
        race_line = np.zeros_like(self.binary_image, np.float)
        spline_line = np.zeros_like(self.binary_image, np.float)

        race_pos = [starting_position[1], starting_position[0]]; # start position

        race_line[race_pos[0], race_pos[1]] = 1;
        cv = [race_pos];

        a_distance = 0.0
        while True:
            a_distance = a_distance + 1.0

            best_val = 100000
            best_pos = race_pos
            better = False

            for [step_x, step_y] in [[0, 1],[0, -1],[1, 0],[-1, 0],[1, 1],[-1, 1],[1, -1],[-1, -1]]:
                candidate_pos = [race_pos[0] + step_x, race_pos[1] + step_y]
                candidate_val = normalized_target_distances[candidate_pos[0], candidate_pos[1]]
                if candidate_val < best_val and current_drivable_area[candidate_pos[0], candidate_pos[1]]:
                    best_val = candidate_val
                    best_pos = candidate_pos
                    better = True
#            if np.mod(a_distance,15) == 0:
            if np.mod(a_distance,self.sample_every) == 0:
                cv = np.append(cv, [race_pos], axis=0)
                rospy.logdebug("Add path point: {} {}".format(race_pos[0], race_pos[1]))
                
            #For debugging:
            #if self.export_debug_images:
            #    print("  best_pos {} {} val {} from current val {}".format(best_pos[0], best_pos[1], best_val, normalized_target_distances[race_pos[0], race_pos[1]]))

            race_pos = best_pos
            if race_line[race_pos[0], race_pos[1]] == 1: # cannot advance, car is stuck
                break

            race_line[race_pos[0], race_pos[1]] = 1;

            if np.mod(a_distance,50) == 0:
                rospy.logdebug("a_distance: {}".format(a_distance))
            if a_distance >= 5000:
                break

        cv = np.append(cv, [[goal_position[1], goal_position[0]]], axis=0)
        self.race_line_points = cv;

        #For debugging:
        if self.export_debug_images:
            race_line = morphology.binary_dilation(race_line, footprint=morphology.footprints.disk(radius=2, dtype=np.bool)) # dilate for thicker line
            io.imsave(self.output_filename_root + '.race_line_' + filename_part + '.png', img_as_ubyte(race_line * 255) - self.binary_image.astype(float) * 150)

        # optional: use spline interpolation
        #d = 35
        #d = self.d_value
        #p = si.BSpline(cv,1000,d,True)
        #p = scipy_bspline(cv,n=1000,degree=d,periodic=True)
        
        spline_line = race_line
        #for [spline_x, spline_y] in p.astype(int):
        #    spline_line[spline_x, spline_y] = 1;

        #For debugging:
        if self.export_debug_images:
            spline_line = morphology.binary_dilation(spline_line, footprint=morphology.footprints.disk(radius=2, dtype=np.bool)) # dilate for thicker line
            io.imsave(self.output_filename_root + '.spline_line.png', img_as_ubyte(spline_line * 255) - self.binary_image.astype(float) * 150)

        spline_line_blurred = ndimage.gaussian_filter(spline_line.astype(float) * 150, sigma=3) * full_drivable_area;
        for it in range(1,10):
            spline_line_blurred = ndimage.gaussian_filter(spline_line_blurred, sigma=3) * full_drivable_area;

        normalized_spline_line_blurred = spline_line_blurred / np.amax(spline_line_blurred.flatten())

        return normalized_spline_line_blurred

    # ===================================================================================================================

    def goal_callback(self, goal_msg):
        if(self.initpose.header.frame_id == "unknown"):
            rospy.loginfo("GlobalPlanner: inital pose was not set before")
        elif not self.map_init:
            rospy.loginfo("GlobalPlanner: map not yet received")
        else:
            rospy.loginfo("GlobalPlanner: goal_callback received")

            starting_position = self.tfBuffer.lookup_transform("map", "r0/base_link", rospy.Time(0))
            self.world_starting_position[0] = starting_position.transform.translation.x # self.initpose.pose.pose.position.x
            self.world_starting_position[1] = starting_position.transform.translation.y # self.initpose.pose.pose.position.y
            self.grid_starting_position = np.divide(self.world_starting_position - self.origin, self.resolution)
            self.grid_starting_position[1] = self.image_shape[1] - self.grid_starting_position[1] - 1
            self.grid_starting_position = self.grid_starting_position.astype(int)
            
            rospy.loginfo("Starting position (world coordinates): [x,y] = [{:.2f}, {:.2f}]".format(
                self.world_starting_position[0], self.world_starting_position[1]
            ))
            rospy.loginfo("Starting position (grid coordinates): [x,y] = [{:d}, {:d}]".format(
                self.grid_starting_position[0], self.grid_starting_position[1]
            ))
            
            self.world_goal_position[0] = goal_msg.pose.position.x
            self.world_goal_position[1] = goal_msg.pose.position.y
            self.grid_goal_position = np.divide(self.world_goal_position - self.origin, self.resolution)
            self.grid_goal_position[1] = self.image_shape[1] - self.grid_goal_position[1] - 1
            self.grid_goal_position = self.grid_goal_position.astype(int)
            
            rospy.loginfo("Goal position (world coordinates): [x,y] = [{:.2f}, {:.2f}]".format(
                self.world_goal_position[0], self.world_goal_position[1]
            ))
            rospy.loginfo("Goal position (grid coordinates): [x,y] = [{:d}, {:d}]".format(
                self.grid_goal_position[0], self.grid_goal_position[1]
            ))
            

            drivable_area_eroded, distance_to_target_eroded_smoothed, normalized_distance_to_target_eroded_smoothed, _, _, _, _ = \
                self.compute_distance_transform_smoothed(starting_position=self.grid_starting_position, goal_position=self.grid_goal_position, \
                erosion_value = self.erosion_value, forward_direction = False)

            drivable_area, distance_to_target_smoothed, normalized_distance_to_target_smoothed, _, _, _, _ = \
                self.compute_distance_transform_smoothed(starting_position=self.grid_starting_position, goal_position=self.grid_goal_position, forward_direction = False)

            #drivable_area, distance_from_start_line, normalized_distance_from_start_line, _, _, _, _ = \
            #    self.compute_distance_transform(starting_position=self.grid_starting_position, goal_position=self.grid_goal_position, forward_direction = True)

            self.compute_raceline(self.grid_starting_position, self.grid_goal_position, drivable_area, drivable_area, distance_to_target_smoothed, "full")
            normalized_spline_line = self.compute_raceline(self.grid_starting_position, self.grid_goal_position, drivable_area_eroded, drivable_area, distance_to_target_eroded_smoothed, "eroded")

            msg = Path()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.get_rostime()
            for line_point in self.race_line_points:
                world_line_point = line_point * self.resolution + self.origin

                pose = PoseStamped()
                pose.pose.position.x = world_line_point[1]
                pose.pose.position.y = -world_line_point[0]
                pose.pose.position.z = 0
                pose.pose.orientation = goal_msg.pose.orientation # only applied by local planner for target position.
                msg.poses.append(pose)
            
            self.path_publisher.publish(msg)
 

    def initialpose_callback(self, pose_msg):
        rospy.loginfo("GlobalPlanner: initialpose_callback received")
        self.initpose = pose_msg
        #rospy.loginfo(self.initpose)

    def map_callback(self, map_msg):
        rospy.loginfo("GlobalPlanner: map_callback received")
        if not self.map_init:
            res = map_msg.info.resolution
            width = map_msg.info.width
            height = map_msg.info.height
            map_data = np.array(map_msg.data).reshape((height, width))
            origin_x = map_msg.info.origin.position.x
            origin_y = map_msg.info.origin.position.y

            self.resolution = res
            self.image_shape = (height, width)
            self.origin = [origin_x, origin_y]

            rospy.loginfo("Map Resolution: " + str(res))
            rospy.loginfo("Map Width: " + str(width))
            rospy.loginfo("Map Height: " + str(height))
            rospy.loginfo("Map Origin: " + str((origin_x, origin_y)))
            
            self.map = Map(width, height, res, map_data, [origin_x, origin_y])
            self.map_init = True

            self.image = np.flip(map_data.astype(np.float), 0)
            self.normalized_image = self.image / np.amax(self.image.flatten()) # normalize image
            self.binary_image = self.normalized_image < (self.occupied_thresh)
            
            rospy.loginfo("Min. pixel value*: {:8.4f} (*) before normalization".format(np.amin(self.image)))
            rospy.loginfo("Max. pixel value*: {:8.4f}".format(np.amax(self.image)))
                        

            # map_uint8 = self.map.data.astype(np.uint8)
            # map_uint8 = map_uint8 * (255/100)
            # cv2.imshow("win", map_uint8)
            # cv2.waitKey()

def main():
    rospy.init_node('global_planner_node')
    gp = GlobalPlanner()
    rospy.spin()
if __name__ == '__main__':
    main()

