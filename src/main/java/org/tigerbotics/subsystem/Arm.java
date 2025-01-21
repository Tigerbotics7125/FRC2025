/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static org.tigerbotics.constant.ArmConsts.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Arm extends SubsystemBase {

    private final SparkMax m_motor = new SparkMax(kMotorID, MotorType.kBrushless);

    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));

    private final SingleJointedArmSim m_armSim =
            new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    100,
                    .5,
                    Units.feetToMeters(1),
                    Units.degreesToRadians(-45),
                    Units.degreesToRadians(45),
                    true,
                    0);

    private final Mechanism2d m_mech = new Mechanism2d(1, 1);
    private final MechanismLigament2d m_armMechLig =
            new MechanismLigament2d(
                    "armLig",
                    Units.feetToMeters(1),
                    Units.radiansToDegrees(m_armSim.getAngleRads()));

    public Arm() {

        configureMotor(m_motor);


        var root = m_mech.getRoot("arm", 0.5, 0.5);
        root.append(m_armMechLig);

        SmartDashboard.putData("arm mech", m_mech);
    }

    private void configureMotor(SparkMax motor) {

    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {

        double vIn = RoboRioSim.getVInVoltage();

        m_armSim.setInputVoltage(m_motor.getAppliedOutput() * vIn);
        m_armSim.update(0.02);

        double rpm =
                Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()) * 100;

        m_motorSim.iterate(rpm, vIn, 0.02);

        m_armMechLig.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }
}
