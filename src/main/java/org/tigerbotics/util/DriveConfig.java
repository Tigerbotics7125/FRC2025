/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.util;

public class DriveConfig {
    /** Defines which direction the robot assumes forwards to be. */
    public enum RobotOrientation {
        // Assumes that forwards is away from the driver station.
        kFieldOriented,
        // Assumes that forwards is towards the front of the robot.
        kRobotOriented;
    }

    private RobotOrientation robotOrientation = RobotOrientation.kRobotOriented;

    public DriveConfig() {}

    public DriveConfig withRobotOrientation(RobotOrientation robotOrientation) {
        this.robotOrientation = robotOrientation;
        return this;
    }

    public RobotOrientation getRobotOrientation() {
        return robotOrientation;
    }
}
