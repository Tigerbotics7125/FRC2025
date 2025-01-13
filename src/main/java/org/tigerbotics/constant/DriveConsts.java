/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class DriveConsts {

    // Drivetrain real-world values.
    public static final Distance kWheelRadius = Inches.of(6).div(2);
    public static final Distance kWheelCircumference =
            Dimensionless.ofBaseUnits(2, Value).times(Math.PI).times(kWheelRadius);

    // TODO: GET REAL MEASUREMENTS.
    public static final Distance kTrackWidth = Inches.of(22);
    public static final Distance kTrackLength = Inches.of(20);
    // The translations calculated based on track & width.
    public static final Translation2d kTLTranslation =
            new Translation2d(kTrackLength.div(2), kTrackWidth.div(2).unaryMinus());
    public static final Translation2d kTRTranslation =
            new Translation2d(kTrackLength.div(2), kTrackWidth.div(2));
    public static final Translation2d kBLTranslation =
            new Translation2d(kTrackLength.div(2).unaryMinus(), kTrackWidth.div(2).unaryMinus());
    public static final Translation2d kBRTranslation =
            new Translation2d(kTrackLength.div(2).unaryMinus(), kTrackWidth.div(2));
    // Kinematics created from the translations.
    public static final MecanumDriveKinematics kKinematics =
            new MecanumDriveKinematics(
                    kTLTranslation, kTRTranslation, kBLTranslation, kBRTranslation);

    // TODO: Make sure this is the gear ratio we're using.
    // Output revolution per input revolution.
    public static final Dimensionless kGearRatio = Dimensionless.ofBaseUnits(10.71, Value);
    // TODO: Measure the max speed by just running the wheels full speed.
    // Looking at motor graph & max eff% free speed is roughly 5400 rpm.
    public static final AngularVelocity kMaxSpeed = Revolutions.of(5400).per(Minute);
    public static final Per<LinearVelocityUnit, AngularVelocityUnit> kVelocityConversion =
            kWheelCircumference.div(Seconds.of(60)).div(Rotation.per(Minute).of(1));
    public static final LinearVelocity kMaxLinearVelocity =
            kMaxSpeed.timesConversionFactor(kVelocityConversion);
    public static final AngularVelocity kMaxAngularVelocity =
            AngularVelocity.ofBaseUnits(
                    kMaxLinearVelocity.div(kTrackWidth.div(2)).magnitude(), RadiansPerSecond);

    // Motor values
    public static final int kFrontLeftID = 1; // Top Left ID
    public static final int kFrontRightID = 2; // Top Right ID
    public static final int kRearLeftID = 3; // Bottom Left ID
    public static final int kRearRightID = 4; // Bottom Right ID

    // Driving control values
    public static final IdleMode kDefaultIdleMode = IdleMode.kCoast;
    public static final boolean kDefaultFieldOriented = true;

    // Feedforward values
    public static final double kFF_ks = 0.0;
    public static final double kFF_kv = 0.0;
    public static final double kFF_ka = 0.0;

    // Closed loop values
    public static final double kMotorP = 0.1;
    public static final double kMotorI = 0.0;
    public static final double kMotorD = 0.0;

    // Motor config setup.
    public static final Current kStallCurrentLimit = Amps.of(80);
    // Convert from motor rotations to linear distance traveled by the wheel.
    public static final Per<DistanceUnit, AngleUnit> kPositionConversionFactor =
            kWheelCircumference.div(kGearRatio).div(Rotation.of(1));
    // Convert the encoder velocity from RPM to Meters per second.
    public static final Per<LinearVelocityUnit, AngularVelocityUnit> kVelocityConversionFactor =
            kWheelCircumference.div(kGearRatio).div(Seconds.of(60)).div(Rotation.per(Minute).of(1));

    public static final SparkMaxConfig kMotorConfig;

    static {
        kMotorConfig = new SparkMaxConfig();

        kMotorConfig.smartCurrentLimit((int) kStallCurrentLimit.magnitude()); // Amps

        kMotorConfig.encoder.positionConversionFactor(kPositionConversionFactor.magnitude());
        kMotorConfig.encoder.velocityConversionFactor(kVelocityConversionFactor.magnitude());

        kMotorConfig.closedLoop.p(kMotorP);
        kMotorConfig.closedLoop.i(kMotorI);
        kMotorConfig.closedLoop.d(kMotorD);
    }
}
