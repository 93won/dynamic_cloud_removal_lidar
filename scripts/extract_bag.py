#!/usr/bin/env python3
"""
Extract point clouds and poses from ROS bag file.
Works without ROS installation using rosbags library.

Usage:
    python extract_bag.py --bag /path/to/bag.bag --output /path/to/output --gt /path/to/indices.csv
"""

import argparse
import os
import sys
import struct
import csv
from pathlib import Path

try:
    from rosbags.rosbag1 import Reader as Rosbag1Reader
    from rosbags.serde import deserialize_cdr, ros1_to_cdr
except ImportError:
    print("Please install rosbags: pip install rosbags")
    sys.exit(1)

import numpy as np


def parse_pointcloud2(msg_data, msg_type):
    """Parse PointCloud2 message to numpy array of xyz points."""
    # Get fields from message
    height = msg_data.height
    width = msg_data.width
    point_step = msg_data.point_step
    row_step = msg_data.row_step
    data = bytes(msg_data.data)
    
    # Find x, y, z field offsets
    x_offset = y_offset = z_offset = None
    x_dtype = y_dtype = z_dtype = None
    
    for field in msg_data.fields:
        if field.name == 'x':
            x_offset = field.offset
            x_dtype = get_dtype(field.datatype)
        elif field.name == 'y':
            y_offset = field.offset
            y_dtype = get_dtype(field.datatype)
        elif field.name == 'z':
            z_offset = field.offset
            z_dtype = get_dtype(field.datatype)
    
    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError("PointCloud2 message missing x, y, or z fields")
    
    # Parse points
    num_points = height * width
    points = np.zeros((num_points, 3), dtype=np.float32)
    
    for i in range(num_points):
        offset = i * point_step
        points[i, 0] = struct.unpack_from('f', data, offset + x_offset)[0]
        points[i, 1] = struct.unpack_from('f', data, offset + y_offset)[0]
        points[i, 2] = struct.unpack_from('f', data, offset + z_offset)[0]
    
    # Filter out NaN and inf points
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    
    return points, valid_mask


def get_dtype(datatype):
    """Convert PointField datatype to numpy dtype."""
    type_map = {
        1: np.int8,
        2: np.uint8,
        3: np.int16,
        4: np.uint16,
        5: np.int32,
        6: np.uint32,
        7: np.float32,
        8: np.float64,
    }
    return type_map.get(datatype, np.float32)


def parse_transform(msg_data):
    """Parse TransformStamped or Odometry to 4x4 matrix."""
    if hasattr(msg_data, 'pose'):
        # Odometry message
        pos = msg_data.pose.pose.position
        ori = msg_data.pose.pose.orientation
    elif hasattr(msg_data, 'transform'):
        # TransformStamped message
        pos = msg_data.transform.translation
        ori = msg_data.transform.rotation
    else:
        raise ValueError("Unknown pose message type")
    
    # Quaternion to rotation matrix
    x, y, z, w = ori.x, ori.y, ori.z, ori.w
    
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pos.x, pos.y, pos.z]
    
    return T


def load_ground_truth(gt_file):
    """Load ground truth dynamic indices from CSV file.
    
    Format: Each line is timestamp,idx1,idx2,idx3,...
    """
    gt_data = {}
    
    with open(gt_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            parts = line.split(',')
            if len(parts) < 1:
                continue
            
            timestamp = int(parts[0])
            indices = [int(x) for x in parts[1:] if x.strip()]
            gt_data[timestamp] = indices
    
    return gt_data


def save_ply(filepath, points, colors=None):
    """Save point cloud to PLY file."""
    with open(filepath, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        if colors is not None:
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
        f.write("end_header\n")
        
        for i, pt in enumerate(points):
            if colors is not None:
                f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f} {int(colors[i,0])} {int(colors[i,1])} {int(colors[i,2])}\n")
            else:
                f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")


def save_bin(filepath, points):
    """Save point cloud to binary file (KITTI format)."""
    # Add dummy intensity/reflectance as 4th channel
    data = np.zeros((len(points), 4), dtype=np.float32)
    data[:, :3] = points
    data.tofile(filepath)


def main():
    parser = argparse.ArgumentParser(description='Extract point clouds from ROS bag')
    parser.add_argument('--bag', required=True, help='Path to ROS bag file')
    parser.add_argument('--output', required=True, help='Output directory')
    parser.add_argument('--gt', help='Ground truth indices.csv file')
    parser.add_argument('--pointcloud_topic', default='/os1_cloud_node/points', 
                        help='Point cloud topic name')
    parser.add_argument('--pose_topic', default='', 
                        help='Pose/odometry topic (optional, will use TF if empty)')
    parser.add_argument('--format', default='ply', choices=['ply', 'bin'],
                        help='Output format (ply or bin)')
    parser.add_argument('--max_frames', type=int, default=-1,
                        help='Maximum number of frames to extract (-1 for all)')
    args = parser.parse_args()
    
    # Create output directories
    output_dir = Path(args.output)
    pointcloud_dir = output_dir / 'pointclouds'
    pointcloud_dir.mkdir(parents=True, exist_ok=True)
    
    # Load ground truth if provided
    gt_data = None
    if args.gt:
        print(f"Loading ground truth from {args.gt}")
        gt_data = load_ground_truth(args.gt)
        print(f"Loaded {len(gt_data)} frames of ground truth")
        
        # Create GT labels directory
        labels_dir = output_dir / 'labels'
        labels_dir.mkdir(parents=True, exist_ok=True)
    
    # Open bag file
    print(f"Opening bag: {args.bag}")
    
    with Rosbag1Reader(args.bag) as reader:
        # List available topics
        print("\nAvailable topics:")
        for conn in reader.connections:
            print(f"  {conn.topic} [{conn.msgtype}]")
        
        # Find point cloud connection
        pc_conn = None
        for conn in reader.connections:
            if conn.topic == args.pointcloud_topic:
                pc_conn = conn
                break
        
        if pc_conn is None:
            print(f"Error: Topic {args.pointcloud_topic} not found in bag")
            return
        
        print(f"\nExtracting from topic: {args.pointcloud_topic}")
        
        # Collect TF transforms for pose interpolation
        tf_poses = {}
        print("Reading TF transforms...")
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic == '/tf':
                try:
                    msg = deserialize_cdr(ros1_to_cdr(rawdata, conn.msgtype), conn.msgtype)
                    for tf in msg.transforms:
                        if tf.child_frame_id == 'os1_lidar' or tf.child_frame_id == 'os_lidar':
                            T = parse_transform(tf)
                            tf_poses[timestamp] = T
                except Exception as e:
                    pass
        
        print(f"Found {len(tf_poses)} TF transforms")
        
        # Extract point clouds
        frame_idx = 0
        poses = []
        timestamps_list = []
        
        print("\nExtracting point clouds...")
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic != args.pointcloud_topic:
                continue
            
            if args.max_frames > 0 and frame_idx >= args.max_frames:
                break
            
            try:
                msg = deserialize_cdr(ros1_to_cdr(rawdata, conn.msgtype), conn.msgtype)
                points, valid_mask = parse_pointcloud2(msg, conn.msgtype)
                
                # Save point cloud
                filename = f"{frame_idx:06d}.{args.format}"
                filepath = pointcloud_dir / filename
                
                if args.format == 'ply':
                    save_ply(filepath, points)
                else:
                    save_bin(filepath, points)
                
                # Find closest TF pose
                if tf_poses:
                    closest_ts = min(tf_poses.keys(), key=lambda x: abs(x - timestamp))
                    pose = tf_poses[closest_ts]
                else:
                    pose = np.eye(4)
                
                poses.append(pose)
                timestamps_list.append(timestamp)
                
                # Save ground truth labels if available
                if gt_data is not None:
                    # Find matching GT by closest timestamp
                    gt_ts = min(gt_data.keys(), key=lambda x: abs(x - timestamp))
                    if abs(gt_ts - timestamp) < 1e9:  # Within 1 second
                        dynamic_indices = gt_data[gt_ts]
                        
                        # Adjust indices for valid points only
                        # Create mapping from original index to new index
                        original_indices = np.where(valid_mask)[0]
                        dynamic_set = set(dynamic_indices)
                        
                        # Find which valid points are dynamic
                        new_dynamic_indices = []
                        for new_idx, orig_idx in enumerate(original_indices):
                            if orig_idx in dynamic_set:
                                new_dynamic_indices.append(new_idx)
                        
                        # Save labels
                        label_file = labels_dir / f"{frame_idx:06d}.txt"
                        with open(label_file, 'w') as f:
                            for idx in new_dynamic_indices:
                                f.write(f"{idx}\n")
                
                frame_idx += 1
                if frame_idx % 100 == 0:
                    print(f"  Processed {frame_idx} frames...")
                    
            except Exception as e:
                print(f"Warning: Failed to process frame {frame_idx}: {e}")
                continue
        
        print(f"\nExtracted {frame_idx} point cloud frames")
        
        # Save poses
        poses_file = output_dir / 'poses.txt'
        with open(poses_file, 'w') as f:
            for pose in poses:
                # Save as 12 values (first 3 rows of 4x4 matrix)
                row = pose[:3, :].flatten()
                f.write(' '.join(f'{x:.10e}' for x in row) + '\n')
        
        print(f"Saved poses to {poses_file}")
        
        # Save timestamps
        timestamps_file = output_dir / 'timestamps.txt'
        with open(timestamps_file, 'w') as f:
            for ts in timestamps_list:
                f.write(f"{ts}\n")
        
        print(f"Saved timestamps to {timestamps_file}")
        
        print("\nDone!")
        print(f"Output directory: {output_dir}")
        print(f"  - pointclouds/: {frame_idx} {args.format} files")
        if gt_data:
            print(f"  - labels/: ground truth labels")
        print(f"  - poses.txt: camera poses")
        print(f"  - timestamps.txt: frame timestamps")


if __name__ == '__main__':
    main()
