/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tigerbotics.subsystem.Drivetrain;

@Logged
public class Robot extends TimedRobot {

    private final Drivetrain m_drivetrain = new Drivetrain();

    private final CommandXboxController m_driver = new CommandXboxController(0);

    @Override
    public void robotInit() {
        // Start logging.
        DataLogManager.start();
        Epilogue.bind(this);

        // Setup subsystem default commands
        m_drivetrain.setDefaultCommand(
                m_drivetrain.driveCommand(
                        () -> -m_driver.getLeftY(),
                        m_driver::getLeftX,
                        m_driver::getRightX,
                        m_driver::getRightY,
                        () -> true));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}
