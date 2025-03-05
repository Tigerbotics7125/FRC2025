/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tigerbotics.command.DriveAssist;
import org.tigerbotics.constant.DriveConsts;
import org.tigerbotics.constant.SuperStructConsts.SuperStructState;
import org.tigerbotics.subsystem.*;

@Logged
public class Robot extends TimedRobot {

    // These are used for logs so we can track down issues if needed.
    @SuppressWarnings("unused")
    private final String kGitSHA = BuildConstants.GIT_SHA;

    @SuppressWarnings("unused")
    private final String kGitBranch = BuildConstants.GIT_BRANCH;

    @SuppressWarnings("unused")
    private final String kGitDate = BuildConstants.GIT_DATE;

    @SuppressWarnings("unused")
    private final String kBuildDate = BuildConstants.BUILD_DATE;

    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Vision m_vision = new Vision();

    private final Odometry m_odometry = new Odometry(m_drivetrain, m_vision);
    private final DriveAssist m_driveAssist =
            new DriveAssist(m_drivetrain, m_odometry, DriveConsts.kDefaultDriveConfig);

    private final Elevator m_elev = new Elevator();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();

    private final SuperStructure m_superStructure = new SuperStructure(m_elev, m_arm);

    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(1);
    private final Autonomous auto = new Autonomous(m_drivetrain, m_odometry);

    public static double currentDrawSim = 0.0;
    public static double vInSim = 12.0;

    public double elevPos = 0.0;
    public double armPos = 0.0;

    @Override
    public void robotInit() {
        // Start logging.
        DataLogManager.start();
        Epilogue.bind(this);

        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        configureButtonBindings();

        NamedCommands.registerCommand(
                "firstStage", m_superStructure.setState(SuperStructState.FIRST));

        // By default, we want to be driving the drivetrain lol.
        m_drivetrain.setDefaultCommand(
                m_driveAssist.driveAssistCmd(
                        () -> -m_driver.getLeftY(),
                        m_driver::getLeftX,
                        m_driver::getRightX,
                        m_driver.rightBumper().negate()));

        // By default, we want the arm to run PID control.
        m_arm.setDefaultCommand(m_arm.runPID());

        // By default, we want the elevator to run PID control.
        m_elev.setDefaultCommand(m_elev.runPID());

        m_intake.setDefaultCommand(m_intake.disable());

        m_operator.rightBumper().onTrue(m_superStructure.setState(SuperStructState.GROUND));
        m_operator.a().onTrue(m_superStructure.setState(SuperStructState.FIRST));
        // m_operator.a().onTrue(Commands.run(() -> m_elev.setGoal(
        // new TrapezoidProfile.State(0.5, 0)),m_elev));

        m_operator.a().onTrue(m_superStructure.setState(SuperStructState.FIRST));
        m_operator.x().onTrue(m_superStructure.setState(SuperStructState.SECOND));
        m_operator.y().onTrue(m_superStructure.setState(SuperStructState.THIRD));
        m_operator.b().onTrue(m_superStructure.setState(SuperStructState.FOURTH));

        m_operator.leftBumper().whileTrue(m_intake.intake());
        m_operator.leftTrigger().whileTrue(m_intake.kick());
        // pre-load pathplanner classes. stupid Java.
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureButtonBindings() {
        /*m_driver.a().onTrue(m_superStructure.setState(SuperStructState.START));
        m_driver.b().onTrue(m_superStructure.setState(SuperStructState.DEMO));*/
        // m_driver.leftBumper().whileTrue(m_intake.intake());
        // m_driver.leftTrigger().whileTrue(m_intake.kick());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // TODO: Bring subsystems online outside of sim, please reset all PID values to zero and
        // tune one subsystem at a time.
        if (Robot.isReal()) {
            m_arm.disable().schedule();
            // m_elev.disable().schedule();
        }
    }

    @Override
    public void autonomousInit() {
        Command autoCommand = auto.getAutonomousCommand();
        autoCommand.andThen(Commands.print("auto done")).schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // m_superStructure.setState(SuperStructState.FIRST);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
        Commands.waitTime(Seconds.of(5))
                .andThen(m_drivetrain.setIdleMode(IdleMode.kCoast))
                .schedule();
    }

    @Override
    public void disabledExit() {
        m_drivetrain.setIdleMode(IdleMode.kBrake).schedule();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {
        // Calculate new battery voltage.
        vInSim = BatterySim.calculateDefaultBatteryLoadedVoltage(currentDrawSim);
        // Set Rio voltage & current
        // (I think this makes brownout and wpilog work but I'm not sure)
        RoboRioSim.setVInVoltage(vInSim);
        RoboRioSim.setVInCurrent(currentDrawSim);
        // Reset current draw.
        currentDrawSim = 0.0;
    }
}
