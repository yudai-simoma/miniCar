import numpy as np
import pandas as pd
from operator import itemgetter
from numpy.linalg import norm

import struct


fmt_full = ''


def pointcloud2_to_array(msg):
    global fmt_full
    if not fmt_full:
        fmt = 'fff' + 20*'x'  # xyz + NEVERMIND
        fmt_full = ('>' if msg.is_bigendian else '<') + fmt*msg.width*msg.height

    unpacker = struct.Struct(fmt_full)
    unpacked = np.asarray(unpacker.unpack_from(msg.data))
    return unpacked.reshape(msg.height*msg.width, 3)


def find_waypoints(
    avg_height,
    max_angle=30, num_angles=12, steps=7, step_len=3,
    sphere_radius=4, draw_waypoints=True,
    nan_coeff=20,
):
    frame = np.flipud(avg_height.values)
    middle = avg_height.columns.get_loc(0)
    starting_point = np.array([frame.shape[0], middle])

    max_angle = max_angle*np.pi/180

    waypoints = np.zeros((steps+1, 2))
    waypoints[0] = starting_point
    angle = 1.5 * np.pi  # "Go straight" angle
    point = starting_point
    for step in range(steps):
        losses = (2*num_angles+1) * [None]
        for loss_idx, angle_idx in enumerate(range(-num_angles, num_angles+1)):
            # We're iterating over candidate angles that for a cone of [-40, 40] degrees
            angle_i = angle + angle_idx * max_angle / num_angles
            vector = step_len * np.array([np.sin(angle_i), np.cos(angle_i)])
            new_position = point + vector

            lower_bound, upper_bound = (
                (new_position - sphere_radius).astype(int),
                (new_position + sphere_radius).astype(int),
            )
            x, y = np.meshgrid(
                np.arange(max(0, lower_bound[0]), min(frame.shape[0], upper_bound[0])),
                np.arange(max(0, lower_bound[1]), min(frame.shape[1], upper_bound[1])),
            )
            xy = np.array([x.flatten(), y.flatten()]).T
            which_in_sphere = (norm(xy - new_position, axis=1) < sphere_radius)

            x_idx, y_idx = np.round(new_position).astype(int)

            piece_of_frame = frame[xy[:, 0], xy[:, 1]]
            mean = np.nanmean(piece_of_frame)
            frac_nans = np.isnan(piece_of_frame).mean()

            loss = mean + nan_coeff * frac_nans
            losses[loss_idx] = [loss, abs(angle_idx), angle_i, new_position]

        # We choose the waypoint that has the best score, but if two waypoints
        #  have the same score, we choose the one that's closest to going the
        #  same direction as previously
        lowest_loss, best_angle_idx, best_angle, best_new_position = min(
            losses, key=itemgetter(0, 1)
        )
        point = best_new_position
        angle = best_angle
        waypoints[step+1] = point

        if draw_waypoints:
            x_idx, y_idx = np.round(best_new_position).astype(int)
            # FIXME: sometimes a waypoint goes around the frame
            if x_idx < frame.shape[0] and y_idx < frame.shape[1]:
                frame[x_idx, y_idx] = -5

    return waypoints, frame


def find_angle(waypoints, starting_waypoint=1):
    difference_quotient = (waypoints[-1, 1] - waypoints[starting_waypoint, 1]) / (waypoints[-1, 0] - waypoints[starting_waypoint, 0])
    return -np.arctan(difference_quotient)
