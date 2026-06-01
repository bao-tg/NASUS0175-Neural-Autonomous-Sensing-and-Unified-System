#!/usr/bin/env python3
"""
Dingo Walk Phase Test
=====================
Alternates between stance and swing phases for a trot gait.
Uses GaitController to determine which legs are in stance vs swing,
StanceController for grounded legs, and SwingController for airborne legs.

Gait phases (trot):
  Phase 0: [FR=stance, FL=stance, BR=stance, BL=swing]   overlap
  Phase 1: [FR=stance, FL=swing,  BR=stance, BL=stance]   swing
  Phase 2: [FR=stance, FL=swing,  BR=stance, BL=stance]   overlap
  Phase 3: [FR=stance, FL=stance, BR=stance, BL=swing]   swing

Press Ctrl+C to stop.
"""
import sys
import time
import signal
import numpy as np

from dingo_control.Config import Configuration, Leg_linkage
from dingo_control.State import State
from dingo_control.Command import Command
from dingo_control.Kinematics import four_legs_inverse_kinematics
from dingo_control.StanceController import StanceController
from dingo_control.SwingLegController import SwingController
from dingo_control.Gaits import GaitController
from dingo_servo_interfacing.HardwareInterface import HardwareInterface

LEG_NAMES = ["FR", "FL", "BR", "BL"]

running = True

def signal_handler(sig, frame):
    global running
    running = False

def print_state(tick, foot_locations, joint_angles, contact_modes):
    print("\n  Tick {:4d}:".format(tick))
    print("  {:>5s}  {:>10s} {:>10s} {:>10s}  |  {:>8s} {:>8s} {:>8s}  |  {}".format(
        "Leg", "X(m)", "Y(m)", "Z(m)", "Hip(d)", "Upp(d)", "Low(d)", "Contact"))
    print("  " + "-" * 90)
    for i, name in enumerate(LEG_NAMES):
        x, y, z = foot_locations[:, i]
        h, u, l = np.degrees(joint_angles[:, i])
        contact = "STANCE" if contact_modes[i] == 1 else "SWING "
        print("  {:>5s}  {:10.5f} {:10.5f} {:10.5f}  |  {:8.2f} {:8.2f} {:8.2f}  |  {}".format(
            name, x, y, z, h, u, l, contact))


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("=" * 70)
    print("  Dingo Walk Phase Test (Trot Gait)")
    print("=" * 70)

    config = Configuration()
    stance_ctrl = StanceController(config)
    swing_ctrl = SwingController(config)
    gait_ctrl = GaitController(config)

    print("\n  Gait config:")
    print("    dt             = {:.4f} s".format(config.dt))
    print("    overlap_ticks  = {}".format(config.overlap_ticks))
    print("    swing_ticks    = {}".format(config.swing_ticks))
    print("    stance_ticks   = {}".format(config.stance_ticks))
    print("    phase_length   = {} ticks ({:.3f} s)".format(
        config.phase_length, config.phase_length * config.dt))
    print("    contact_phases =")
    for i in range(config.num_phases):
        print("      Phase {}: {}".format(i, config.contact_phases[:, i]))

    state = State()
    state.foot_locations = (
        config.default_stance
        + np.array([0, 0, config.default_z_ref])[:, np.newaxis]
    )

    command = Command()
    command.horizontal_velocity = np.array([0.3, 0.0])
    command.yaw_rate = 0.0
    command.height = config.default_z_ref

    print("\n  Command:")
    print("    horizontal_velocity = {} m/s".format(command.horizontal_velocity))
    print("    yaw_rate           = {} rad/s".format(command.yaw_rate))
    print("    body height        = {:.3f} m".format(command.height))

    joint_angles = four_legs_inverse_kinematics(state.foot_locations, config)
    print("\n  Initial standing state:")
    print_state(0, state.foot_locations, joint_angles, np.ones(4))

    linkage = Leg_linkage(config)
    hardware = HardwareInterface(linkage)
    if hardware.kit is None:
        print("\n  PCA9685 not found - dry run only (no servo movement)")
        print("  Press Ctrl+C to stop.\n")
        tick = 0
        print_every = max(1, config.phase_length)
        try:
            while running:
                contact_modes = gait_ctrl.contacts(state.ticks)
                new_foot_locations = np.zeros((3, 4))
                
                for leg_index in range(4):
                    
                    if contact_modes[leg_index] == 1:
                        
                        new_foot_locations[:, leg_index] = stance_ctrl.next_foot_location(
                            leg_index, state, command
                        )
                    else:
                        
                        swing_prop = gait_ctrl.subphase_ticks(state.ticks) / config.swing_ticks
                        swing_prop = np.clip(swing_prop, 0.0, 1.0)
                        new_foot_locations[:, leg_index] = swing_ctrl.next_foot_location(
                            swing_prop, leg_index, state, command
                        )
                state.foot_locations = new_foot_locations
                state.ticks += 1
                tick += 1

                if tick % print_every == 0:
                    joint_angles = four_legs_inverse_kinematics(state.foot_locations, config)
                    print_state(tick, state.foot_locations, joint_angles, contact_modes)
        except KeyboardInterrupt:
            pass
        print("\n  Dry run test complete.")
        return

    print("\n  PCA9685 found! Moving servos to standing pose in 3s...")
    time.sleep(3.0)

    standing_feet = config.default_stance + np.array([0, 0, config.default_z_ref])[:, np.newaxis]
    standing_angles = four_legs_inverse_kinematics(standing_feet, config)
    current = np.zeros((3, 4))
    for step in range(1, 41):
        t = step / 40.0
        angles = current + (standing_angles - current) * t
        hardware.set_actuator_postions(angles)
        time.sleep(0.04)

    print("  Standing. Starting trot gait on servos...")
    print("  Press Ctrl+C to stop.\n")

    try:
        while running:
            contact_modes = gait_ctrl.contacts(state.ticks)
            new_foot_locations = np.zeros((3, 4))
            for leg_index in range(4):
                if contact_modes[leg_index] == 1:
                    new_foot_locations[:, leg_index] = stance_ctrl.next_foot_location(
                        leg_index, state, command
                    )
                    if leg_index == 0:
                        print("   #################        stance phase      #################    ")
                        print("foot_locations[:, leg_index]: ", np.round(new_foot_locations[:, leg_index],3))
                else:
                    swing_prop = gait_ctrl.subphase_ticks(state.ticks) / config.swing_ticks
                    swing_prop = np.clip(swing_prop, 0.0, 1.0)
                    new_foot_locations[:, leg_index] = swing_ctrl.next_foot_location(
                        swing_prop, leg_index, state, command
                    )
                    if leg_index == 0:
                        print("   #################        swing phase      #################    ")
                        print("foot_locations[:, leg_index]: ", np.round(new_foot_locations[:, leg_index],3))
            state.foot_locations = new_foot_locations
            state.ticks += 1

            joint_angles = four_legs_inverse_kinematics(state.foot_locations, config)
            hardware.set_actuator_postions(joint_angles)
            time.sleep(config.dt)
    except KeyboardInterrupt:
        pass

    print("\n  Stopping... Relaxing servos...")
    hardware.relax_all_motors()
    print("  Done.")


if __name__ == "__main__":
    main()
# source /dingo_ws/devel/setup.bash && sudo env PYTHONPATH=$PYTHONPATH OPENBLAS_CORETYPE=ARMV8 BLINKA_JETSON_NANO=1 python3 /dingo_ws/src/dingo/scripts/dingo_driver.py