#!/usr/bin/env python3
"""
Inspect PKL file structure and contents.

This script helps you understand the structure of pkl files from GMR retargeting.
"""

import argparse
import pickle
import sys
import os

try:
    import joblib
    HAS_JOBLIB = True
except ImportError:
    HAS_JOBLIB = False


def inspect_pkl(pkl_file):
    """Inspect and print pkl file structure."""
    if not os.path.exists(pkl_file):
        print(f"Error: File not found: {pkl_file}")
        return

    print(f"\n{'='*60}")
    print(f"Inspecting: {pkl_file}")
    print(f"{'='*60}\n")

    # Try to load with joblib first, then pickle
    try:
        if HAS_JOBLIB:
            data = joblib.load(pkl_file)
            print("✓ Loaded with joblib")
        else:
            with open(pkl_file, 'rb') as f:
                data = pickle.load(f)
            print("✓ Loaded with pickle")
    except Exception as e:
        print(f"✗ Failed to load: {e}")
        return

    # Check if it's a dict or single motion
    if isinstance(data, dict):
        print(f"\nType: Dictionary with {len(data)} keys")
        print(f"Keys: {list(data.keys())}\n")
        
        # Check if it's a motion dict (multiple motions) or single motion data
        first_key = list(data.keys())[0]
        first_value = data[first_key]
        
        if isinstance(first_value, dict):
            print(f"Structure: Dictionary of motion sequences")
            print(f"\nFirst motion '{first_key}' structure:")
            print(f"{'─'*60}")
            inspect_motion_data(first_value, indent="  ")
            
            # Show all motion names
            print(f"\nAll motion sequences:")
            for i, key in enumerate(data.keys(), 1):
                if isinstance(data[key], dict):
                    motion = data[key]
                    timesteps = None
                    if 'joint_pos' in motion:
                        timesteps = motion['joint_pos'].shape[0] if hasattr(motion['joint_pos'], 'shape') else len(motion['joint_pos'])
                    elif 'dof_pos' in motion:
                        timesteps = motion['dof_pos'].shape[0] if hasattr(motion['dof_pos'], 'shape') else len(motion['dof_pos'])
                    print(f"  {i}. '{key}': {timesteps} frames" if timesteps else f"  {i}. '{key}'")
        else:
            # It's a single motion data dict
            print(f"Structure: Single motion data dictionary")
            print(f"{'─'*60}")
            inspect_motion_data(data)
    else:
        print(f"\nType: {type(data).__name__}")
        if hasattr(data, '__dict__'):
            print(f"Attributes: {list(data.__dict__.keys())}")
        else:
            print(f"Value: {data}")


def inspect_motion_data(motion, indent=""):
    """Inspect a single motion data dictionary."""
    import numpy as np
    
    print(f"{indent}Motion data fields:")
    
    for key, value in motion.items():
        if isinstance(value, np.ndarray):
            print(f"{indent}  {key}:")
            print(f"{indent}    Type: numpy.ndarray")
            print(f"{indent}    Shape: {value.shape}")
            print(f"{indent}    Dtype: {value.dtype}")
            if value.size > 0:
                print(f"{indent}    Min: {np.min(value):.4f}, Max: {np.max(value):.4f}")
                if value.size <= 10:
                    print(f"{indent}    Sample: {value.flatten()[:10]}")
        elif isinstance(value, (list, tuple)):
            print(f"{indent}  {key}:")
            print(f"{indent}    Type: {type(value).__name__}")
            print(f"{indent}    Length: {len(value)}")
            if len(value) > 0 and isinstance(value[0], (int, float, str)):
                print(f"{indent}    Sample: {value[:5]}")
        elif isinstance(value, dict):
            print(f"{indent}  {key}:")
            print(f"{indent}    Type: dict with {len(value)} keys")
            print(f"{indent}    Keys: {list(value.keys())[:10]}")
        else:
            print(f"{indent}  {key}: {type(value).__name__} = {value}")
    
    # Check for GMR format
    print(f"\n{indent}Format detection:")
    has_gmr_format = all(k in motion for k in ['fps', 'root_pos', 'root_rot', 'dof_pos'])
    has_sonic_format = all(k in motion for k in ['joint_pos', 'joint_vel', 'body_quat_w'])
    
    if has_gmr_format:
        print(f"{indent}  ✓ GMR format detected (fps, root_pos, root_rot, dof_pos)")
        print(f"{indent}    - fps: {motion.get('fps', 'N/A')}")
        print(f"{indent}    - root_pos shape: {motion.get('root_pos', 'N/A')}")
        if 'root_rot' in motion:
            root_rot = motion['root_rot']
            if hasattr(root_rot, 'shape'):
                print(f"{indent}    - root_rot shape: {root_rot.shape} (format: xyzw)")
        if 'dof_pos' in motion:
            dof_pos = motion['dof_pos']
            if hasattr(dof_pos, 'shape'):
                print(f"{indent}    - dof_pos shape: {dof_pos.shape} (DOF = Degrees of Freedom)")
    
    if has_sonic_format:
        print(f"{indent}  ✓ SONIC format detected (joint_pos, joint_vel, body_quat_w)")
        if 'joint_pos' in motion:
            print(f"{indent}    - joint_pos shape: {motion['joint_pos'].shape}")
        if 'joint_vel' in motion:
            print(f"{indent}    - joint_vel shape: {motion['joint_vel'].shape}")
        if 'body_quat_w' in motion:
            print(f"{indent}    - body_quat_w shape: {motion['body_quat_w'].shape}")
    
    if not has_gmr_format and not has_sonic_format:
        print(f"{indent}  ⚠ Unknown format - may need conversion")


def main():
    parser = argparse.ArgumentParser(
        description="Inspect pkl file structure and contents"
    )
    parser.add_argument(
        "pkl_file",
        type=str,
        help="Path to pkl file to inspect",
    )
    
    args = parser.parse_args()
    inspect_pkl(args.pkl_file)


if __name__ == "__main__":
    main()
