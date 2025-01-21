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
import org.tigerbotics.subsystem.Odometry;
import org.tigerbotics.subsystem.Vision;

@Logged
public class Robot extends TimedRobot {

    // These are used for logs so we can track down issues if needed.
    private final String GIT_SHA = BuildConstants.GIT_SHA;
    private final String GIT_DATE = BuildConstants.GIT_DATE;
    private final String GIT_BRANCH = BuildConstants.GIT_BRANCH;
    private final String BUILD_DATE = BuildConstants.BUILD_DATE;

    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Vision m_vision = new Vision();

    private final Odometry m_odometry = new Odometry(m_drivetrain, m_vision);

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
