/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static edu.wpi.first.units.Units.*;
import static org.tigerbotics.constant.DriveConsts.*;
import static org.tigerbotics.constant.DriveConsts.Simulation.*;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Logged
public class Drivetrain extends SubsystemBase {

    // Controllers & Sensors
    private final List<SparkMax> m_motors;
    private final List<SparkMaxSim> m_sparkSims = new ArrayList<>();
    private final List<DCMotorSim> m_motorSims = new ArrayList<>();
    private final AHRS m_navx;

    // Odometry
    private final MecanumDrivePoseEstimator m_poseEstimator;

    // Feedforward control
    private final SimpleMotorFeedforward m_feedForward = kFeedforward;

    // Field for simulation
    @Logged private final Field2d m_field = new Field2d();

    private final boolean m_fieldOriented = kDefaultFieldOriented;

    public Drivetrain() {
        // Motors will always be in order: FL, FR, RL, RR.
        m_motors =
                List.of(
                        new SparkMax(kFrontLeftID, MotorType.kBrushless),
                        new SparkMax(kFrontRightID, MotorType.kBrushless),
                        new SparkMax(kRearLeftID, MotorType.kBrushless),
                        new SparkMax(kRearRightID, MotorType.kBrushless));

        // Configure the motors
        m_motors.forEach(this::configureMotor);

        // Create the navX using the SPI on the rio.
        m_navx = new AHRS(NavXComType.kMXP_SPI);
        // Raise alert if navX is not connected.
        kNavXConnectAlert.set(!m_navx.isConnected());

        // Start the odometry / pose estimator.
        m_poseEstimator =
                new MecanumDrivePoseEstimator(
                        kKinematics, getRotation2d(), getWheelPositions(), new Pose2d());
    }

    /**
     * Configures a motor by applying the SparkMaxConfig object. This method will raise an Alert on
     * the dashboard if the config fails.
     *
     * <p>This method will also handle setting up the simulation objects for each motor.
     *
     * @param motor The motor to configure.
     */
    private void configureMotor(SparkMax motor) {
        // Setup simulation objects.
        m_sparkSims.add(new SparkMaxSim(motor, kMotor));
        m_motorSims.add(new DCMotorSim(kMotorPlant, kMotor));

        // Apply SparkMAX configurations.
        SparkMaxConfig config =
                motor.getDeviceId() == kFrontRightID || motor.getDeviceId() == kRearRightID
                        ? kRightMotorsConfig
                        : kLeftMotorsConfig;
        REVLibError response =
                motor.configure(
                        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // Raise alert if config failed.
        kMotorConfigAlert.set(response != REVLibError.kOk);
    }

    @Override
    public void periodic() {
        m_poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(), getRotation2d(), getWheelPositions());

        m_field.setRobotPose(getPose2d());
    }

    @Override
    public void simulationPeriodic() {
        double vIn = RoboRioSim.getVInVoltage();
        double accumCurrent = 0;

        for (int i = 0; i < 4; i++) {
            // Breaks sim, ask me how I know.
            if (vIn == 0) break;

            // Update the motor sim
            m_motorSims.get(i).setInputVoltage(m_motors.get(i).getAppliedOutput() * vIn);
            m_motorSims.get(i).update(0.02);
            // update the spark values from sim
            m_sparkSims
                    .get(i)
                    .iterate(
                            m_motorSims.get(i).getAngularVelocityRPM()
                                    / 60.0
                                    * kWheelCircumference.in(Meters),
                            vIn,
                            0.02);
            // Add the current draw to the sim.
            accumCurrent += m_sparkSims.get(i).getMotorCurrent();
        }

        // Set the calculated voltage to the system.
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(accumCurrent));

        // Get the change in angle (Euler's Method)
        double deltaOmega =
                Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond)
                        / kMaxAngularVelocity.in(DegreesPerSecond)
                        * 0.02;

        // Update the NavX angle
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(m_navx.getYaw() + deltaOmega);

        // Update the field.
        m_field.setRobotPose(getPose2d());
        SmartDashboard.putData(m_field);
    }

    public void drive(Translation2d translation, Rotation2d rotation, boolean openLoop) {

        // If field oriented, then rotate the translation to match robot orientation.
        if (m_fieldOriented) translation = translation.rotateBy(getRotation2d());

        // Currently not implementing a PID on heading, if/when that occurs the full rotation will
        // be useful,
        // See 2023 drivetrain code for info, implementation wasn't great but it should be a good
        // starting point.
        double thetaSpeed = rotation.getSin();
        // System.out.println(translation.getX() + " " + translation.getY() + " " + thetaSpeed);

        if (openLoop) {
            setOpenLoopSpeeds(
                    MecanumDrive.driveCartesianIK(
                            translation.getX(), translation.getY(), thetaSpeed));
        } else {
            // For closed loop, scale duty cycle values to velocity values.
            translation = translation.times(kMaxLinearVelocity.in(MetersPerSecond));
            thetaSpeed = thetaSpeed * kMaxAngularVelocity.in(RadiansPerSecond);

            setTargetChassisSpeeds(
                    new ChassisSpeeds(translation.getX(), translation.getY(), thetaSpeed));
        }
    }

    /**
     * Add vision pose estimations to the robot pose.
     *
     * @param visionPose The pose as seen by vision.
     * @param visionPoseTimestamp The timestamp that the vision pose was generated at.
     */
    public void addVisionEstimate(Pose2d visionPose, double visionPoseTimestamp) {
        m_poseEstimator.addVisionMeasurement(visionPose, visionPoseTimestamp);
    }

    // ~ Getters

    public Rotation2d getRotation2d() {
        return m_navx.getRotation2d();
    }

    public Pose2d getPose2d() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
                m_motors.get(0).getEncoder().getPosition(),
                m_motors.get(1).getEncoder().getPosition(),
                m_motors.get(2).getEncoder().getPosition(),
                m_motors.get(3).getEncoder().getPosition());
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                m_motors.get(0).getEncoder().getVelocity(),
                m_motors.get(1).getEncoder().getVelocity(),
                m_motors.get(2).getEncoder().getVelocity(),
                m_motors.get(3).getEncoder().getVelocity());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getWheelSpeeds());
    }

    // ~ Setters

    /** @param currentPose The pose the robot is located (Rotation does not matter). */
    public void setOdometryPose(Pose2d currentPose) {
        m_poseEstimator.resetPosition(getRotation2d(), getWheelPositions(), currentPose);
    }

    /** Set the current heading to zero. */
    public void resetHeading() {
        m_navx.reset();
    }

    @Logged ChassisSpeeds targetChassisSpeeds;

    /**
     * Set the target chassis speeds to be achieved by the chassis.
     *
     * @param targetSpeeds
     */
    public void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeed) {
        this.targetChassisSpeeds = targetChassisSpeed;
        setClosedLoopSpeeds(kKinematics.toWheelSpeeds(targetChassisSpeed));
    }

    @Logged MecanumDriveWheelSpeeds targetWheelSpeeds;

    /**
     * Set the target wheel speeds to be achieved by each wheel. Uses the built in PID controller on
     * the SparkMAX
     *
     * @param targetWheelSpeeds
     */
    public void setClosedLoopSpeeds(MecanumDriveWheelSpeeds targetWheelSpeeds) {
        targetWheelSpeeds.desaturate(kMaxLinearVelocity);

        this.targetWheelSpeeds = targetWheelSpeeds;

        List<Double> wheelSpeeds =
                List.of(
                        targetWheelSpeeds.frontLeftMetersPerSecond,
                        targetWheelSpeeds.frontRightMetersPerSecond,
                        targetWheelSpeeds.rearLeftMetersPerSecond,
                        targetWheelSpeeds.rearRightMetersPerSecond);

        for (int i = 0; i < 4; i++) {
            m_motors.get(i)
                    .getClosedLoopController()
                    .setReference(
                            wheelSpeeds.get(i),
                            ControlType.kMAXMotionVelocityControl,
                            ClosedLoopSlot.kSlot0,
                            m_feedForward.calculate(wheelSpeeds.get(i)),
                            ArbFFUnits.kVoltage);
        }
    }

    /**
     * Set the target wheel speeds to be achieved by each wheel. Uses duty cycle only.
     *
     * @param targetWheelSpeeds
     */
    public void setOpenLoopSpeeds(MecanumDrive.WheelSpeeds targetWheelSpeeds) {
        List<Double> dutyCycles =
                List.of(
                        targetWheelSpeeds.frontLeft,
                        targetWheelSpeeds.frontRight,
                        targetWheelSpeeds.rearLeft,
                        targetWheelSpeeds.rearRight);

        for (int i = 0; i < 4; i++) {
            m_motors.get(i).set(dutyCycles.get(i));
        }
    }

    // ~ Commands

    /** @return A Command which disables all motors. */
    public Command disable() {
        return runOnce(() -> m_motors.forEach(SparkMax::disable));
    }

    public Command driveCommand(
            DoubleSupplier xTranslation,
            DoubleSupplier yTranslation,
            DoubleSupplier cosRotation,
            DoubleSupplier sinRotation,
            BooleanSupplier openLoop) {
        return run(
                () ->
                        drive(
                                new Translation2d(
                                        xTranslation.getAsDouble(), yTranslation.getAsDouble()),
                                new Rotation2d(
                                        cosRotation.getAsDouble(), sinRotation.getAsDouble()),
                                openLoop.getAsBoolean()));
    }
}
