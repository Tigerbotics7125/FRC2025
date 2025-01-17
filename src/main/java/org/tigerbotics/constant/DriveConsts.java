/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class DriveConsts {

    // Default driving behavior control values
    public static final IdleMode kDefaultIdleMode = IdleMode.kCoast;
    public static final boolean kDefaultFieldOriented = true;

    public static final Alert kMotorConfigAlert =
            new Alert(
                    "Drivetrain: SparkMAX configure failed! (Performance may be affected!)",
                    AlertType.kError);
    public static final Alert kNavXConnectAlert =
            new Alert(
                    "Drivetrain: NavX failed to connect! (Odometry & Field-oriented affected!)",
                    AlertType.kError);

    // Motor ID values
    public static final int kFrontLeftID = 1; // Top Left ID
    public static final int kFrontRightID = 2; // Top Right ID
    public static final int kRearLeftID = 3; // Bottom Left ID
    public static final int kRearRightID = 4; // Bottom Right ID

    // Drivetrain real-world values.
    public static final Distance kWheelRadius = Inches.of(6).div(2);
    public static final Distance kWheelCircumference = kWheelRadius.times(2).times(Math.PI);

    // TODO: GET REAL MEASUREMENTS.
    public static final Distance kTrackWidth = Inches.of(22);
    public static final Distance kTrackLength = Inches.of(20);
    // The translations calculated based on track & width.
    public static final Translation2d kFLTranslation =
            new Translation2d(kTrackLength.div(2), kTrackWidth.div(2).unaryMinus());
    public static final Translation2d kFRTranslation =
            new Translation2d(kTrackLength.div(2), kTrackWidth.div(2));
    public static final Translation2d kRLTranslation =
            new Translation2d(kTrackLength.div(2).unaryMinus(), kTrackWidth.div(2).unaryMinus());
    public static final Translation2d kRRTranslation =
            new Translation2d(kTrackLength.div(2).unaryMinus(), kTrackWidth.div(2));
    // Kinematics created from the translations.
    public static final MecanumDriveKinematics kKinematics =
            new MecanumDriveKinematics(
                    kFLTranslation, kFRTranslation, kRLTranslation, kRRTranslation);

    // Values calculated from the AM Toughbox Micro S product page
    // TODO: Make sure right gear ratio.
    public static final Dimensionless kGearRatio = Dimensionless.ofBaseUnits(10.71, Value);
    public static final LinearVelocity kMaxLinearVelocity = Feet.of(13.88).per(Second);
    public static final AngularVelocity kMaxAngularVelocity =
            AngularVelocity.ofBaseUnits(
                    kMaxLinearVelocity.div(kTrackWidth.div(2)).magnitude(), RadiansPerSecond);

    // Feedforward values
    // TODO: Run SYSID so we can use feedforward.
    public static final double ks = 0.0;
    public static final double kv = 0.0;
    public static final double ka = 0.0;
    public static final SimpleMotorFeedforward kFeedforward =
            new SimpleMotorFeedforward(ks, kv, ka);

    // Configuration objects.
    public static final SparkMaxConfig kLeftMotorsConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightMotorsConfig = new SparkMaxConfig();

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
                kWheelCircumference.div(kGearRatio).div(Rotation.of(1)).magnitude());
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

    // Put all the simulation stuff in its own class.
    public static final class Simulation {
        public static final DCMotor kMotor = DCMotor.getNEO(1);
        // I calculated all of the MoI at one point and it came out to about 0.001, and 0.002 with
        // the wheel.
        public static final LinearSystem<N2, N1, N2> kMotorPlant =
                LinearSystemId.createDCMotorSystem(kMotor, 0.002, kGearRatio.magnitude());
    }
}
