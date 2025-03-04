/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static edu.wpi.first.units.Units.*;
import static org.tigerbotics.constant.ElevConsts.*;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tigerbotics.Robot;

@Logged
public class Elevator extends SubsystemBase {
    private final SparkMax m_left = new SparkMax(kLeftID, MotorType.kBrushless);
    private final SparkMax m_right = new SparkMax(kRightID, MotorType.kBrushless);

    private final SparkMaxSim m_leftSim = new SparkMaxSim(m_left, DCMotor.getNEO(1));
    private final SparkMaxSim m_rightSim = new SparkMaxSim(m_right, DCMotor.getNEO(1));

    private final ElevatorSim m_elevSim = kElevSim;

    private final NetworkTable table =
            NetworkTableInstance.getDefault().getTable("Robot/m_elev/pid");
    private final DoubleEntry closedLoopPEntry = table.getDoubleTopic("/p").getEntry(0.0);
    private final DoubleEntry closedLoopIEntry = table.getDoubleTopic("/i").getEntry(0.0);
    private final DoubleEntry closedLoopDEntry = table.getDoubleTopic("/d").getEntry(0.0);
    private final DoubleEntry maxVelEntry = table.getDoubleTopic("/maxVel").getEntry(0.0);
    private final DoubleEntry maxAccelEntry = table.getDoubleTopic("/maxAccel").getEntry(0.0);
    private final DoubleEntry setpointEntry = table.getDoubleTopic("/setpointMeters").getEntry(0.0);

    public Elevator() {
        REVLibError leftConfigure =
                m_left.configure(
                        kLeftConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
        REVLibError rightConfigure =
                m_right.configure(
                        kRightConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        kConfigAlert.set(leftConfigure != REVLibError.kOk || rightConfigure != REVLibError.kOk);

        // publish the current values to NT.
        maxVelEntry.set(kPIDController.getConstraints().maxVelocity);
        maxAccelEntry.set(kPIDController.getConstraints().maxAcceleration);
        closedLoopPEntry.set(kPIDController.getP());
        closedLoopIEntry.set(kPIDController.getI());
        closedLoopDEntry.set(kPIDController.getD());
        setpointEntry.set(0.0);
    }

    @Override
    public void periodic() {
        System.out.println("Elavator periodic");
        // sim variable tuner.
        if (maxVelEntry.readQueue().length != 0)
            kPIDController.setConstraints(
                    new TrapezoidProfile.Constraints(
                            maxVelEntry.get(), kPIDController.getConstraints().maxAcceleration));
        if (maxAccelEntry.readQueue().length != 0)
            kPIDController.setConstraints(
                    new TrapezoidProfile.Constraints(
                            kPIDController.getConstraints().maxVelocity, maxAccelEntry.get()));

        if (closedLoopPEntry.readQueue().length != 0) kPIDController.setP(closedLoopPEntry.get());
        if (closedLoopIEntry.readQueue().length != 0) kPIDController.setI(closedLoopIEntry.get());
        if (closedLoopDEntry.readQueue().length != 0) kPIDController.setD(closedLoopDEntry.get());

        if (setpointEntry.readQueue().length != 0)
            setGoal(new TrapezoidProfile.State(setpointEntry.get(), 0));
        // kPIDController.calculate(m_left.getEncoder().getPosition());
        System.out.println("GOAL:" + kPIDController.getGoal().position);
        double voltage = kPIDController.calculate(getPosition().in(Meters));
        m_left.setVoltage(voltage);
        String voltageS = " " + voltage;
        System.out.println(voltageS);
    }

    @Override
    public void simulationPeriodic() {
        m_elevSim.setInputVoltage(m_leftSim.getAppliedOutput() * Robot.vInSim);
        m_elevSim.update(0.02);

        double mps = m_elevSim.getVelocityMetersPerSecond();

        m_leftSim.iterate(mps, Robot.vInSim, 0.02);
        m_rightSim.iterate(mps, Robot.vInSim, 0.02);

        Robot.currentDrawSim += m_elevSim.getCurrentDrawAmps();
    }

    /** @return The Distance from the starting point of the Elevator. */
    public Distance getPosition() {
        return Meters.of(m_left.getEncoder().getPosition());
    }

    /** @return The LinearVelocity of the Elevator. */
    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(m_left.getEncoder().getVelocity());
    }

    /**
     * Set a new PID Goal.
     *
     * @param goal The trapezoid profile state to go to. Units are in Meters, & Meters/Second for
     *     position and velocity respectively.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        kPIDController.setGoal(goal);
    }

    /**
     * @return The Distance of the setpoint. (Not the end goal, this is the moving target to keep
     *     the system smooth.)
     */
    public Distance getSetpointPosition() {
        return Meters.of(kPIDController.getSetpoint().position);
    }

    /**
     * @return The LinearVelocity of the setpoint. (Not the end goal, this is the moving target to
     *     keep the system smooth.)
     */
    public LinearVelocity getSetpointVelocity() {
        return MetersPerSecond.of(kPIDController.getSetpoint().velocity);
    }

    /** @return A Command which will run the PID controller and motor outputs. */
    public Command runPID() {
        return run(
                () -> {
                    double voltage = kPIDController.calculate(getPosition().in(Meters));
                    m_left.setVoltage(voltage);
                    String voltageS = " " + voltage;
                    Commands.print(voltageS);
                });
    }

    /** @return A Command which disables motor output. */
    public Command disable() {
        return run(
                () -> {
                    m_left.disable();
                    m_right.disable();
                });
    }
}
