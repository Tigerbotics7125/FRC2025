/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.tigerbotics.constant.DriveConsts;
import org.tigerbotics.util.Elastic;

@Logged
public class Drivetrain extends SubsystemBase {

    // Controllers & Sensors
    private final List<SparkMax> m_motors;
    private final List<SparkMaxSim> m_motorSims = new ArrayList<>();
    private final AHRS m_navx;

    // Odometry
    private final MecanumDrivePoseEstimator m_poseEstimator;

    // Feedforward control
    private final SimpleMotorFeedforward m_feedForward;

    // Field for simulation
    @Logged
    private final Field2d m_field = new Field2d();

    private final boolean m_fieldOriented = DriveConsts.kDefaultFieldOriented;

    Alert m_motorConfigAlert = new Alert("Drivetrain: SparkMAX %d configure failed! (Performance likely affected!)", AlertType.kError);
    Alert m_navxConnectAlert = new Alert("Drivetrain: NavX failed to connect! (Odometry & Field-oriented affected!)", AlertType.kError);

    public Drivetrain() {
        // * DT motors will always be in front left, front right, rear left, rear right order.
        m_motors =
                List.of(
                        new SparkMax(DriveConsts.kFrontLeftID, MotorType.kBrushless),
                        new SparkMax(DriveConsts.kFrontRightID, MotorType.kBrushless),
                        new SparkMax(DriveConsts.kRearLeftID, MotorType.kBrushless),
                        new SparkMax(DriveConsts.kRearRightID, MotorType.kBrushless));

        m_motors.forEach(this::configureMotor);

        m_navx = new AHRS(NavXComType.kMXP_SPI);
        m_navxConnectAlert.set(m_navx.isConnected());

        m_poseEstimator =
                new MecanumDrivePoseEstimator(
                        DriveConsts.kKinematics,
                        getRotation2d(),
                        getWheelPositions(),
                        new Pose2d());

        m_feedForward =
                new SimpleMotorFeedforward(
                        DriveConsts.kFF_ks, DriveConsts.kFF_kv, DriveConsts.kFF_ka);
    }

    private void configureMotor(SparkMax motor) {
        m_motorSims.add(new SparkMaxSim(motor, DCMotor.getNEO(1)));

        // Invert the right side motors.
        /**
         * TODO: I don't like that these are mutating the "const" config object but you're supposed
         * to apply inversion via the config now so idk what to do.
         */
        if (motor.getDeviceId() == DriveConsts.kFrontRightID
                || motor.getDeviceId() == DriveConsts.kRearRightID) {
            DriveConsts.kMotorConfig.inverted(true);
        } else {
            DriveConsts.kMotorConfig.inverted(false);
        }

        // Apply configurations to the motor, this will write the configs to flash.
        REVLibError response =
                motor.configure(
                        DriveConsts.kMotorConfig,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kPersistParameters);

        // Alert if the config failed.
        if (response != REVLibError.kOk) {
            m_motorConfigAlert.setText(String.format(m_motorConfigAlert.getText(), motor.getDeviceId()));
            m_motorConfigAlert.set(true);
        }
    }

    @Override
    public void periodic() {
        m_poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(), getRotation2d(), getWheelPositions());

        m_field.setRobotPose(getPose2d());
    }

    @Override
    public void simulationPeriodic() {
        for (int i = 0; i < 4; i++) {
            m_motorSims.get(i).iterate(i, i, i);
        }

        // Get the angular velocity
        AngularVelocity omegaVel =
                AngularVelocity.ofBaseUnits(
                        getChassisSpeeds().omegaRadiansPerSecond, RadiansPerSecond);
        // Get the change in angle (Euler's Method)
        Angle deltaOmega = omegaVel.times(Milliseconds.of(20));

        // Update the NavX angle
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(m_navx.getFusedHeading() + deltaOmega.in(Degrees));

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
        double thetaSpeed = rotation.getCos();

        if (openLoop) {
            setOpenLoopSpeeds(
                    MecanumDrive.driveCartesianIK(
                            translation.getX(), translation.getY(), thetaSpeed));
        } else {
            // For closed loop, scale duty cycle values to velocity values.
            translation = translation.times(DriveConsts.kMaxLinearVelocity.in(MetersPerSecond));
            thetaSpeed = thetaSpeed * DriveConsts.kMaxAngularVelocity.in(RotationsPerSecond);

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
        return DriveConsts.kKinematics.toChassisSpeeds(getWheelSpeeds());
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

    /**
     * Set the target chassis speeds to be achieved by the chassis.
     *
     * @param targetSpeeds
     */
    public void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeed) {
        setClosedLoopSpeeds(DriveConsts.kKinematics.toWheelSpeeds(targetChassisSpeed));
    }

    /**
     * Set the target wheel speeds to be achieved by each wheel. Uses the built in PID controller on
     * the SparkMAX
     *
     * @param targetWheelSpeeds
     */
    public void setClosedLoopSpeeds(MecanumDriveWheelSpeeds targetWheelSpeeds) {
        targetWheelSpeeds.desaturate(DriveConsts.kMaxLinearVelocity);

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
                            ControlType.kVelocity,
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
