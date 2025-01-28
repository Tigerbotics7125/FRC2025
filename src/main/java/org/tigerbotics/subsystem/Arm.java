/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static edu.wpi.first.units.Units.*;
import static org.tigerbotics.constant.ArmConsts.*;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tigerbotics.Robot;

@Logged
public class Arm extends SubsystemBase {

    // Motor & PID controller.
    private final SparkMax m_motor = new SparkMax(kMotorID, MotorType.kBrushless);

    private final NetworkTable table =
            NetworkTableInstance.getDefault().getTable("Robot/m_arm/pid");

    private final DoubleEntry closedLoopPEntry = table.getDoubleTopic("/p").getEntry(0.0);
    private final DoubleEntry closedLoopIEntry = table.getDoubleTopic("/i").getEntry(0.0);
    private final DoubleEntry closedLoopDEntry = table.getDoubleTopic("/d").getEntry(0.0);
    private final DoubleEntry maxVelEntry = table.getDoubleTopic("/maxVel").getEntry(0.0);
    private final DoubleEntry maxAccelEntry = table.getDoubleTopic("/maxAccel").getEntry(0.0);

    private final DoubleEntry setpointEntry =
            table.getDoubleTopic("/setpointDegrees").getEntry(0.0);

    // Sim variables
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));
    private final SingleJointedArmSim m_armSim = kArmSim;

    public Arm() {
        REVLibError result =
                m_motor.configure(
                        kConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Show error if config fails.
        kConfigAlert.set(result != REVLibError.kOk);

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
    }

    @Override
    public void simulationPeriodic() {
        m_armSim.setInputVoltage(m_motorSim.getAppliedOutput() * Robot.vInSim);
        m_armSim.update(0.02);

        double degreesPerSecond = Units.radiansToDegrees(m_armSim.getVelocityRadPerSec());

        m_motorSim.iterate(degreesPerSecond, Robot.vInSim, 0.02);

        Robot.currentDrawSim += m_armSim.getCurrentDrawAmps();
    }

    /** @return The Angle of the Arm. */
    public Angle getPosition() {
        return Degrees.of(m_motor.getEncoder().getPosition());
    }

    /** @return The AngularVelocity of the Arm. */
    public AngularVelocity getVelocity() {
        return DegreesPerSecond.of(m_motor.getEncoder().getVelocity());
    }

    /**
     * @return The Angle of the setpoint. (Not the end goal, this is the moving target to keep the
     *     system smooth.)
     */
    public Angle getSetpointPosition() {
        return Degrees.of(kPIDController.getSetpoint().position);
    }

    /**
     * @return The AngularVelocity of the setpoint. (Not the end goal, this is the moving target to
     *     keep the system smooth.)
     */
    public AngularVelocity getSetpointVelocity() {
        return DegreesPerSecond.of(kPIDController.getSetpoint().velocity);
    }

    /**
     * Set a new PID Goal.
     *
     * @param goal The trapezoid profile state to go to. Units are in Degrees, & Degrees/Second for
     *     position and velocity respectively.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        kPIDController.setGoal(goal);
    }

    /** @return A Command which will run the PID controller and control the motor. */
    public Command runPID() {
        return run(
                () -> {
                    // TODO: Add feedforward.
                    double voltage = kPIDController.calculate(m_motor.getEncoder().getPosition());
                    m_motor.setVoltage(voltage);
                });
    }

    /** @return A Command which will disable motor output. */
    public Command disable() {
        return run(() -> m_motor.disable());
    }
}
