/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;

public class DriveConfig {
    /** Defines which direction the robot assumes forwards to be. */
    public enum RobotOrientation {
        // Assumes that forward is away from the driver station.
        kFieldOriented,
        // Assumes that forward is towards the front of the robot.
        kRobotOriented;
    }

    public class ReefAlignAssistConfig {
        private boolean assistanceEnabled = true;
        private double minDotProduct = 0.5;
        private Distance maxDistanceToReef = Meters.of(2);
        private PIDController translationPID = new PIDController(2.0, 0.0, 0.0);
        private PIDController rotationPID = new PIDController(0.01, 0.0, 0.0);

        /** @param assistanceEnabled Whether to enable this assistance or not. */
        public ReefAlignAssistConfig withAssistanceEnabled(boolean assistanceEnabled) {
            this.assistanceEnabled = assistanceEnabled;
            return this;
        }

        public boolean assistanceEnabled() {
            return assistanceEnabled;
        }

        /**
         * @param minDotProduct The minimum dot product value to ensure the driver is intending to
         *     go towards the reef.
         */
        public ReefAlignAssistConfig withMinDotProduct(double minDotProduct) {
            this.minDotProduct = minDotProduct;
            return this;
        }

        public double minDotProduct() {
            return minDotProduct;
        }

        /**
         * @param minDistanceToReef The robot must be this close to the reef before the assistance
         *     will enable.
         */
        public ReefAlignAssistConfig withMaxDistanceToReef(Distance minDistanceToReef) {
            this.maxDistanceToReef = minDistanceToReef;
            return this;
        }

        public Distance maxDistanceToReef() {
            return maxDistanceToReef;
        }

        /** @param translationPID The translational PIDController. */
        public ReefAlignAssistConfig withTranslationPID(PIDController translationPID) {
            this.translationPID = translationPID;
            return this;
        }

        public PIDController translationPID() {
            return translationPID;
        }

        /** @param rotationPID The rotational PIDController. */
        public ReefAlignAssistConfig withRotationPID(PIDController rotationPID) {
            this.rotationPID = rotationPID;
            return this;
        }

        public PIDController rotationPID() {
            return rotationPID;
        }
    }

    // Defines how the controllers forward direction should be interpreted by the drivetrain.
    private RobotOrientation robotOrientation = RobotOrientation.kRobotOriented;
    // Configurations related to the reef align assistance feature.
    private ReefAlignAssistConfig reefAlignAssist = new ReefAlignAssistConfig();

    public DriveConfig withRobotOrientation(RobotOrientation robotOrientation) {
        this.robotOrientation = robotOrientation;
        return this;
    }

    public RobotOrientation robotOrientation() {
        return robotOrientation;
    }

    public DriveConfig withReefAlignAssist(ReefAlignAssistConfig reefAlignAssist) {
        this.reefAlignAssist = reefAlignAssist;
        return this;
    }

    public ReefAlignAssistConfig reefAlignAssist() {
        return reefAlignAssist;
    }
}
