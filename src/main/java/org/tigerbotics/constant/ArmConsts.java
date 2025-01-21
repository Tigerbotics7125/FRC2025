/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Dimensionless;

public class ArmConsts {

    public static final int kMotorID = 21;

    public static final SparkMaxConfig kMotorConfig;

    // TODO: Find the actual value.
    public static final Dimensionless kGearRatio = Value.of(100);

    // Use a static block to alter the config here as opposed to in Drivetrain
    // class.
    static {
        // Create a new config object.
        SparkMaxConfig config = new SparkMaxConfig();

        // Limits the max current draw.
        config.smartCurrentLimit(80);

        // Set the conversion factors, converting motor position and velocity (rotations
        // & RPM) to linear wheel position & valocity (meters & meters/second)
        config.encoder.positionConversionFactor(
                Value.of(1).div(kGearRatio).times(2 * Math.PI).magnitude());
        config.encoder.velocityConversionFactor(
                kWheelCircumference
                        .div(kGearRatio)
                        .div(Seconds.of(60))
                        .div(Rotation.per(Minute).of(1))
                        .magnitude());

        // Set PID values for the spark's built-in PID controller.
        config.closedLoop.maxMotion.maxVelocity(kMaxLinearVelocity.in(MetersPerSecond));
        config.closedLoop.maxMotion.maxAcceleration(
                kMaxLinearVelocity.per(Second).times(100).in(MetersPerSecondPerSecond));
        // I tried tuning this in sim to no avail, good luck future jeff and seth.
        config.closedLoop.p(0.7);
        config.closedLoop.i(0);
        config.closedLoop.d(0.05);
        config.closedLoop.velocityFF(1 / 473); // from rev docs and neo 1.1 specs.

        // Set the config objects.
        kLeftMotorsConfig.apply(config);
        kRightMotorsConfig.apply(config);

        // Invert the right side. (This must be done after the apply otherwise it would
        // get overwritten.)
        kLeftMotorsConfig.inverted(false);
        kRightMotorsConfig.inverted(true);
    }
}
