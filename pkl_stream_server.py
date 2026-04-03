#!/usr/bin/env python3
"""
PKL Motion Stream Server for SONIC Robot Control

Streams motion data from GMR retargeting pkl files via ZMQ Protocol v1.
This script reads pre-recorded motion sequences and streams them to the SONIC
deployment system for robot control.

Usage:
    # Basic usage - stream a GMR pkl file (auto-detects format)
    python pkl_stream_server.py --pkl_file /home/qc/data/lafan1-GMR-retargeted/jumps1_subject1.pkl

    # Stream with custom settings
    python pkl_stream_server.py --pkl_file motion.pkl \
        --port 5556 \
        --target_fps 50 \
        --num_frames_to_send 5 \
        --loop

    # Stream multiple motion sequences from a dictionary pkl file
    python pkl_stream_server.py --pkl_file motions_dict.pkl --motion_name motion_1

    # Interactive mode with keyboard controls
    python pkl_stream_server.py --pkl_file motion.pkl --interactive

Supported Formats:
    - GMR format: fps, root_pos, root_rot (xyzw), dof_pos
    - SONIC format: joint_pos, joint_vel, body_quat_w
    
    The script automatically detects and converts GMR format to SONIC format.
"""

import argparse
import os
import pickle
import sys
import threading
import time
from typing import Dict, Optional

import numpy as np
import zmq

try:
    from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
        build_command_message,
        pack_pose_message,
    )
except ImportError:
    print("Warning: gear_sonic.utils.teleop.zmq.zmq_planner_sender not available.")
    print("Some features may not work correctly.")
    build_command_message = None
    pack_pose_message = None


class PKLMotionStreamer:
    """
    Streams motion data from pkl files via ZMQ Protocol v1.
    
    Supports both single motion sequences and dictionaries of multiple motions.
    """

    def __init__(
        self,
        pkl_file: str,
        motion_name: Optional[str] = None,
        port: int = 5556,
        num_frames_to_send: int = 5,
        target_fps: int = 50,
        root_body_index: int = 0,
        loop: bool = False,
        interactive: bool = False,
    ):
        """
        Initialize PKL motion streamer.

        Args:
            pkl_file: Path to pkl file containing motion data
            motion_name: Name of motion sequence to stream (if pkl contains dict)
            port: ZMQ publisher port
            num_frames_to_send: Number of frames per message batch
            target_fps: Target streaming frame rate
            root_body_index: Index of root body in body_quat_w (default: 0)
            loop: Whether to loop the motion sequence
            interactive: Enable keyboard controls
        """
        self.pkl_file = pkl_file
        self.motion_name = motion_name
        self.port = port
        self.num_frames_to_send = num_frames_to_send
        self.target_fps = target_fps
        self.root_body_index = root_body_index
        self.loop = loop
        self.interactive = interactive

        # Load motion data
        self.motion_data = self._load_motion_data()
        
        # Extract motion sequence
        if isinstance(self.motion_data, dict):
            # Check if this is a GMR format single motion (has GMR fields) or a dict of multiple motions
            gmr_fields = {'fps', 'root_pos', 'root_rot', 'dof_pos'}
            sonic_fields = {'joint_pos', 'joint_vel', 'body_quat_w'}
            
            # Check if dict keys match GMR or SONIC format (single motion)
            is_gmr_single = gmr_fields.issubset(self.motion_data.keys())
            is_sonic_single = sonic_fields.issubset(self.motion_data.keys())
            
            if is_gmr_single or is_sonic_single:
                # This is a single motion sequence in GMR or SONIC format
                self.motion_sequence = self.motion_data
                self.motion_name = "default"
                print(f"[PKLStreamer] Detected single motion sequence (GMR or SONIC format)")
            else:
                # This is a dictionary of multiple motion sequences
                if motion_name is None:
                    if len(self.motion_data) == 1:
                        # Auto-select if only one motion
                        self.motion_name = list(self.motion_data.keys())[0]
                        print(f"[PKLStreamer] Auto-selected motion: {self.motion_name}")
                    else:
                        print(f"[PKLStreamer] Available motions: {list(self.motion_data.keys())}")
                        raise ValueError(
                            f"pkl file contains {len(self.motion_data)} motions. "
                            f"Please specify --motion_name"
                        )
                if self.motion_name not in self.motion_data:
                    raise ValueError(
                        f"Motion '{self.motion_name}' not found in pkl file. "
                        f"Available: {list(self.motion_data.keys())}"
                    )
                self.motion_sequence = self.motion_data[self.motion_name]
        else:
            # Single motion sequence (not a dict)
            self.motion_sequence = self.motion_data
            self.motion_name = "default"

        # Validate and extract data
        self._validate_motion_data()
        
        # Initialize ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        time.sleep(0.1)  # Give socket time to bind
        print(f"[PKLStreamer] ZMQ socket bound to port {port}")

        # Streaming state
        self.is_streaming = False
        self.is_paused = False
        self.current_frame = 0
        self.frame_time = 1.0 / max(1, target_fps)
        self.last_batch_time = None  # Track last batch send time for smooth streaming
        self.total_frames_sent_global = 0  # Track total frames sent across loops (for continuous indexing)
        
        # Statistics
        self.fps_counter = 0
        self.last_fps_report = time.time()
        self.total_frames_sent = 0

    def _load_motion_data(self):
        """Load motion data from pkl file."""
        if not os.path.exists(self.pkl_file):
            raise FileNotFoundError(f"PKL file not found: {self.pkl_file}")

        print(f"[PKLStreamer] Loading motion data from: {self.pkl_file}")
        try:
            import joblib
            data = joblib.load(self.pkl_file)
            print("[PKLStreamer] ✓ Successfully loaded with joblib")
        except ImportError:
            print("[PKLStreamer] joblib not available, trying pickle...")
            try:
                with open(self.pkl_file, 'rb') as f:
                    data = pickle.load(f)
                print("[PKLStreamer] ✓ Successfully loaded with pickle")
            except Exception as e:
                raise RuntimeError(f"Failed to load pkl file: {e}")
        except Exception as e:
            raise RuntimeError(f"Failed to load pkl file with joblib: {e}")

        return data

    def _validate_motion_data(self):
        """Validate motion data structure and extract arrays."""
        data = self.motion_sequence

        # Check if it's GMR format (fps, root_pos, root_rot, dof_pos) or SONIC format (joint_pos, joint_vel, body_quat_w)
        has_gmr_format = all(k in data for k in ['fps', 'root_pos', 'root_rot', 'dof_pos'])
        has_sonic_format = all(k in data for k in ['joint_pos', 'joint_vel', 'body_quat_w'])

        if has_gmr_format:
            # GMR format: convert to SONIC format
            print("[PKLStreamer] Detected GMR format, converting to SONIC format...")
            self._convert_gmr_to_sonic(data)
        elif has_sonic_format:
            # SONIC format: use directly
            print("[PKLStreamer] Detected SONIC format")
            self._extract_sonic_format(data)
        else:
            available_fields = list(data.keys())
            raise ValueError(
                f"Unknown motion data format. Available fields: {available_fields}\n"
                f"Expected either:\n"
                f"  - GMR format: fps, root_pos, root_rot, dof_pos\n"
                f"  - SONIC format: joint_pos, joint_vel, body_quat_w"
            )

    def _convert_gmr_to_sonic(self, data):
        """Convert GMR format to SONIC format."""
        # Extract GMR data
        dof_pos = np.array(data['dof_pos'], dtype=np.float32)  # [T, num_dofs]
        root_pos = np.array(data['root_pos'], dtype=np.float32)  # [T, 3]
        root_rot_xyzw = np.array(data['root_rot'], dtype=np.float32)  # [T, 4] in xyzw format
        
        # Get fps from GMR data (required for velocity calculation)
        fps = data.get('fps', 30.0)
        if fps <= 0:
            fps = 30.0
            print(f"[PKLStreamer] Warning: Invalid fps in pkl file, using default {fps} Hz")
        
        # Convert root_rot from xyzw to wxyz
        root_rot_wxyz = root_rot_xyzw[:, [3, 0, 1, 2]]  # [T, 4] in wxyz format
        
        timesteps = dof_pos.shape[0]
        num_dofs = dof_pos.shape[1]
        
        print(f"[PKLStreamer] GMR data: {timesteps} frames, {num_dofs} DOFs, {fps} Hz")
        
        # Check if we have 29 DOFs (G1 robot)
        if num_dofs != 29:
            print(f"[PKLStreamer] Warning: Expected 29 DOFs for G1, got {num_dofs}")
            if num_dofs < 29:
                # Pad with zeros
                print(f"[PKLStreamer] Padding {29 - num_dofs} DOFs with zeros")
                padding = np.zeros((timesteps, 29 - num_dofs), dtype=np.float32)
                dof_pos = np.concatenate([dof_pos, padding], axis=1)
            else:
                # Truncate
                print(f"[PKLStreamer] Truncating to 29 DOFs")
                dof_pos = dof_pos[:, :29]
        
        # Convert from MuJoCo order (GMR output) to IsaacLab order (ZMQ Protocol v1 requirement)
        # MuJoCo order: groups by limb (left leg, right leg, waist, left arm, right arm)
        # IsaacLab order: interleaves left/right joints differently (used by RL policy and SDK)
        # Mapping from policy_parameters.hpp: mujoco_to_isaaclab
        MUJOCO_TO_ISAACLAB = np.array([
            0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
            16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28
        ], dtype=np.int32)
        
        # Convert joint positions from MuJoCo to IsaacLab order
        dof_pos_isaaclab = dof_pos[:, MUJOCO_TO_ISAACLAB]  # [T, 29]
        self.joint_pos = dof_pos_isaaclab
        print(f"[PKLStreamer] Converted joint positions from MuJoCo order to IsaacLab order (ZMQ Protocol v1 requirement)")
        
        # Calculate joint velocities using finite difference
        # Protocol v1 requires joint_vel, but GMR doesn't provide it
        # We compute it from joint positions: vel[t] = (pos[t+1] - pos[t]) * fps
        # Note: Compute velocities in IsaacLab order (already converted above)
        print(f"[PKLStreamer] Computing joint velocities from positions (fps={fps} Hz)...")
        dof_vel_isaaclab = np.zeros_like(dof_pos_isaaclab, dtype=np.float32)  # [T, 29]
        
        if timesteps > 1:
            # Forward difference: vel[t] = (pos[t+1] - pos[t]) * fps
            # Use IsaacLab-ordered positions
            dof_vel_isaaclab[:-1] = (dof_pos_isaaclab[1:] - dof_pos_isaaclab[:-1]) * fps
            
            # For the last frame, use the velocity from the previous frame
            dof_vel_isaaclab[-1] = dof_vel_isaaclab[-2]
            
            # Smooth the first frame velocity to avoid sudden jumps from rest
            # Use average of first two velocity calculations for smoother start
            if timesteps > 2:
                dof_vel_isaaclab[0] = (dof_vel_isaaclab[0] + dof_vel_isaaclab[1]) * 0.5
        else:
            # Single frame case: velocity is zero
            dof_vel_isaaclab[0] = 0.0
        
        # Clamp velocities to reasonable limits for G1 robot
        # G1 arm joints typically have limits around 6 rad/s, leg joints can be higher
        max_velocity_limit = 10.0  # rad/s (reasonable limit)
        vel_clamped = np.clip(dof_vel_isaaclab, -max_velocity_limit, max_velocity_limit)
        if not np.allclose(dof_vel_isaaclab, vel_clamped):
            num_clamped = np.sum(np.abs(dof_vel_isaaclab) > max_velocity_limit)
            print(f"[PKLStreamer] WARNING: Clamped {num_clamped} velocity values exceeding ±{max_velocity_limit} rad/s")
            dof_vel_isaaclab = vel_clamped
        
        self.joint_vel = dof_vel_isaaclab  # [T, 29] in IsaacLab order
        
        # Log velocity statistics for debugging
        max_vel = np.max(np.abs(dof_vel_isaaclab))
        mean_vel = np.mean(np.abs(dof_vel_isaaclab))
        print(f"[PKLStreamer] Velocity stats: max={max_vel:.3f} rad/s, mean={mean_vel:.3f} rad/s")
        
        # Create body_quat_w: we only have root, so shape is [T, 1, 4]
        # But SONIC expects [T, num_bodies, 4], so we'll use root only
        self.body_quat_w = root_rot_wxyz[:, np.newaxis, :]  # [T, 1, 4]
        
        # Extract root quaternion
        self.body_quat = root_rot_wxyz  # [T, 4] in wxyz format
        
        self.timesteps = timesteps
        print(f"[PKLStreamer] Converted: {timesteps} frames, root quaternion extracted, velocities computed")

    def _extract_sonic_format(self, data):
        """Extract data from SONIC format."""
        # Extract arrays
        self.joint_pos = np.array(data['joint_pos'], dtype=np.float32)  # [T, 29]
        self.joint_vel = np.array(data['joint_vel'], dtype=np.float32)  # [T, 29]
        self.body_quat_w = np.array(data['body_quat_w'], dtype=np.float32)  # [T, num_bodies, 4]

        # Validate shapes
        timesteps = self.joint_pos.shape[0]
        if self.joint_pos.shape != (timesteps, 29):
            raise ValueError(
                f"joint_pos shape mismatch: expected (T, 29), got {self.joint_pos.shape}"
            )
        if self.joint_vel.shape != (timesteps, 29):
            raise ValueError(
                f"joint_vel shape mismatch: expected (T, 29), got {self.joint_vel.shape}"
            )
        if len(self.body_quat_w.shape) != 3 or self.body_quat_w.shape[2] != 4:
            raise ValueError(
                f"body_quat_w shape mismatch: expected (T, num_bodies, 4), "
                f"got {self.body_quat_w.shape}"
            )
        if self.body_quat_w.shape[0] != timesteps:
            raise ValueError(
                f"Timestep mismatch: joint_pos has {timesteps} frames, "
                f"body_quat_w has {self.body_quat_w.shape[0]} frames"
            )

        # Extract root body quaternion
        num_bodies = self.body_quat_w.shape[1]
        if self.root_body_index >= num_bodies:
            raise ValueError(
                f"root_body_index {self.root_body_index} >= num_bodies {num_bodies}"
            )
        
        # Extract root quaternion: [T, 4] with format (w, x, y, z)
        self.body_quat = self.body_quat_w[:, self.root_body_index, :]  # [T, 4]

        self.timesteps = timesteps
        print(f"[PKLStreamer] Motion '{self.motion_name}': {timesteps} frames, "
              f"{num_bodies} body parts, root index: {self.root_body_index}")

    def _send_batch(self, start_frame: int, end_frame: int) -> bool:
        """
        Send a batch of frames via ZMQ Protocol v1.

        Args:
            start_frame: Start frame index (inclusive)
            end_frame: End frame index (exclusive)

        Returns:
            True if batch was sent successfully
        """
        if end_frame > self.timesteps:
            end_frame = self.timesteps

        if start_frame >= end_frame:
            return False

        batch_size = end_frame - start_frame

        # Extract batch data
        batch_joint_pos = self.joint_pos[start_frame:end_frame]  # [N, 29]
        batch_joint_vel = self.joint_vel[start_frame:end_frame]  # [N, 29]
        batch_body_quat = self.body_quat[start_frame:end_frame]  # [N, 4]
        
        # Frame indices must be monotonically increasing and continuous
        # For loop mode, we use global frame indices to maintain continuity
        # This prevents window confusion when looping
        if hasattr(self, 'total_frames_sent_global'):
            # Use global frame index for continuous indexing across loops
            batch_frame_index = np.arange(
                self.total_frames_sent_global,
                self.total_frames_sent_global + batch_size,
                dtype=np.int64
            )
        else:
            # Fallback: use local frame indices
            batch_frame_index = np.arange(start_frame, end_frame, dtype=np.int64)  # [N]

        # Prepare ZMQ message data (Protocol v1)
        numpy_data = {
            "joint_pos": batch_joint_pos.astype(np.float32),      # [N, 29]
            "joint_vel": batch_joint_vel.astype(np.float32),      # [N, 29]
            "body_quat": batch_body_quat.astype(np.float32),      # [N, 4] (w, x, y, z)
            "frame_index": batch_frame_index,                       # [N]
            "catch_up": np.array([False], dtype=bool),            # Disable catch-up to prevent frame resets
        }

        # Pack and send message (Protocol v1)
        if pack_pose_message is None:
            raise RuntimeError("pack_pose_message not available")
        
        packed_message = pack_pose_message(numpy_data, topic="pose", version=1)
        self.socket.send(packed_message)

        self.total_frames_sent += batch_size
        if hasattr(self, 'total_frames_sent_global'):
            self.total_frames_sent_global += batch_size
        
        return True

    def _keyboard_listener(self):
        """Keyboard input listener thread for interactive mode."""
        import select
        import tty
        import termios

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            print("\n[PKLStreamer] Interactive mode enabled. Controls:")
            print("  ENTER: Start/Pause streaming")
            print("  R: Restart from beginning")
            print("  Q: Quit")
            print("  Press ENTER to start...")

            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == '\r' or key == '\n':  # ENTER
                        if not self.is_streaming:
                            self.is_streaming = True
                            self.is_paused = False
                            print("[PKLStreamer] Streaming STARTED")
                        else:
                            self.is_paused = not self.is_paused
                            if self.is_paused:
                                print("[PKLStreamer] Streaming PAUSED")
                            else:
                                print("[PKLStreamer] Streaming RESUMED")
                    elif key == 'r' or key == 'R':
                        self.current_frame = 0
                        print(f"[PKLStreamer] Restarted to frame 0")
                    elif key == 'q' or key == 'Q':
                        print("[PKLStreamer] Quitting...")
                        self.is_streaming = False
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def run(self):
        """Main streaming loop."""
        print(f"[PKLStreamer] Starting motion streamer")
        print(f"  Motion: {self.motion_name}")
        print(f"  Frames: {self.timesteps}")
        print(f"  Target FPS: {self.target_fps}")
        print(f"  Batch size: {self.num_frames_to_send}")
        print(f"  Loop: {self.loop}")
        print(f"  Interactive: {self.interactive}")

        # Start keyboard listener if interactive
        keyboard_thread = None
        if self.interactive:
            keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
            keyboard_thread.start()
        else:
            # Non-interactive: start streaming immediately
            self.is_streaming = True
            print("[PKLStreamer] Streaming started (non-interactive mode)")

        # Send initial command message to start policy and switch to STREAMED_MOTION mode
        # For zmq_manager: planner=false means STREAMED_MOTION mode (pose data)
        if build_command_message is not None:
            try:
                # Start policy and switch to STREAMED_MOTION mode (planner=false)
                print("[PKLStreamer] Sending command: start=True, stop=False, planner=False (STREAMED_MOTION mode)")
                self.socket.send(build_command_message(start=True, stop=False, planner=False))
                time.sleep(0.1)  # Give time for command to be processed
            except Exception as e:
                print(f"[PKLStreamer] Warning: failed to send initial command: {e}")
        else:
            print("[PKLStreamer] Warning: build_command_message not available. "
                  "Policy may not start automatically. Use keyboard ']' in C++ terminal to start.")

        try:
            while True:
                if not self.is_streaming:
                    if self.interactive:
                        time.sleep(0.1)
                        continue
                    else:
                        break  # Exit if not interactive and streaming stopped

                if self.is_paused:
                    time.sleep(0.1)
                    continue

                # Calculate batch end frame
                end_frame = min(
                    self.current_frame + self.num_frames_to_send,
                    self.timesteps
                )

                # Send batch
                batch_send_start = time.time()
                if self._send_batch(self.current_frame, end_frame):
                    self.current_frame = end_frame

                    # Check if reached end
                    if self.current_frame >= self.timesteps:
                        if self.loop:
                            print(f"[PKLStreamer] Loop: restarting from frame 0 (global index continues: {self.total_frames_sent_global})")
                            self.current_frame = 0
                            # Note: total_frames_sent_global continues to increment for continuous indexing
                            # This helps the receiver maintain a consistent sliding window
                        else:
                            print(f"[PKLStreamer] Finished streaming {self.timesteps} frames")
                            if self.interactive:
                                self.is_streaming = False
                                print("[PKLStreamer] Press ENTER to restart, Q to quit")
                            else:
                                # Non-interactive: exit after one playthrough
                                break

                # Frame rate control - maintain consistent batch send rate
                # Each batch contains num_frames_to_send frames
                batch_duration = self.num_frames_to_send * self.frame_time
                batch_send_elapsed = time.time() - batch_send_start
                sleep_time = batch_duration - batch_send_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.1:  # Warn if significantly behind
                    print(f"[PKLStreamer] Warning: Batch send took {batch_send_elapsed:.3f}s, "
                          f"expected {batch_duration:.3f}s (behind by {abs(sleep_time):.3f}s)")

                # FPS reporting
                self.fps_counter += 1
                current_time = time.time()
                if current_time - self.last_fps_report >= 5.0:
                    fps = self.fps_counter / (current_time - self.last_fps_report)
                    print(f"[PKLStreamer] FPS: {fps:.2f}, Frame: {self.current_frame}/{self.timesteps}, "
                          f"Total sent: {self.total_frames_sent}")
                    self.fps_counter = 0
                    self.last_fps_report = current_time

        except KeyboardInterrupt:
            print("\n[PKLStreamer] Interrupted by user")
        finally:
            # Send stop command
            if build_command_message is not None:
                try:
                    print("[PKLStreamer] Sending stop command...")
                    self.socket.send(build_command_message(start=False, stop=True, planner=False))
                    time.sleep(0.1)  # Give time for command to be processed
                except Exception as e:
                    print(f"[PKLStreamer] Warning: failed to send stop command: {e}")
            
            self.socket.close()
            self.context.term()
            print(f"[PKLStreamer] Shutdown complete. Total frames sent: {self.total_frames_sent}")


def main():
    parser = argparse.ArgumentParser(
        description="Stream GMR retargeted motion data via ZMQ Protocol v1"
    )
    parser.add_argument(
        "--pkl_file",
        type=str,
        required=True,
        help="Path to pkl file containing motion data",
    )
    parser.add_argument(
        "--motion_name",
        type=str,
        default=None,
        help="Name of motion sequence to stream (required if pkl contains dict)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5556,
        help="ZMQ publisher port (default: 5556)",
    )
    parser.add_argument(
        "--num_frames_to_send",
        type=int,
        default=10,
        help="Number of frames per message batch (default: 5)",
    )
    parser.add_argument(
        "--target_fps",
        type=int,
        default=50,
        help="Target streaming frame rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--root_body_index",
        type=int,
        default=0,
        help="Index of root body in body_quat_w (default: 0)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop the motion sequence continuously",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Enable interactive keyboard controls (ENTER: start/pause, R: restart, Q: quit)",
    )

    args = parser.parse_args()

    try:
        streamer = PKLMotionStreamer(
            pkl_file=args.pkl_file,
            motion_name=args.motion_name,
            port=args.port,
            num_frames_to_send=args.num_frames_to_send,
            target_fps=args.target_fps,
            root_body_index=args.root_body_index,
            loop=args.loop,
            interactive=args.interactive,
        )
        streamer.run()
    except Exception as e:
        print(f"[PKLStreamer] Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
