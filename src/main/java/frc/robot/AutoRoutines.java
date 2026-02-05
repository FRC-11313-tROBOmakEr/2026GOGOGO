// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import static edu.wpi.first.units.Units.*;

//import java.lang.annotation.Target;
import frc.robot.subsystems.Target;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.Subsystem;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class AutoRoutines {
        AutoFactory m_factory;

        public AutoRoutines(AutoFactory factory) {
                m_factory = factory;
        }

        public AutoRoutine shoot2cycleAuto(Shooter shoot, Intake autoIntake) {
                final AutoRoutine routine = m_factory.newRoutine("shoot2CycleAuto");
                final AutoTrajectory shoot2cycleAuto = routine.trajectory("shoot2CycleAuto");

                routine.active().onTrue(
                                shoot2cycleAuto.resetOdometry()
                                                .andThen(shoot.shootAuto())
                                                .andThen(autoIntake.autointake1()));
                return routine;
        }

        public AutoRoutine shootclimbleft(Command shootCommand, Command climberCommand) {
                final AutoRoutine routine = m_factory.newRoutine("Shootclimbleft");
                final AutoTrajectory shootclimbleft = routine.trajectory("Shootclimbleft");
                routine.active().onTrue(
                                shootclimbleft.resetOdometry()
                                                .andThen(shootCommand)
                                                .andThen(climberCommand));
                return routine;
        }

        public AutoRoutine shootclimbright(Command shootCommand, Command climberCommand) {
                final AutoRoutine routine = m_factory.newRoutine("Shootclimbright");
                final AutoTrajectory shootclimbright = routine.trajectory("Shootclimbright");

                routine.active().onTrue(
                                shootclimbright.resetOdometry()
                                                .andThen(shootCommand)
                                                .andThen(climberCommand));
                return routine;
        }

        public AutoRoutine shootclimbmid(Command shootCommand, Command climberCommand) {
                final AutoRoutine routine = m_factory.newRoutine("Shootclimbmid");
                final AutoTrajectory shootclimbmid = routine.trajectory("Shootclimbmid");

                routine.active().onTrue(
                                shootclimbmid.resetOdometry()
                                                .andThen(shootCommand)
                                                .andThen(climberCommand));
                return routine;
        }

        }

        public RobotContainer() {
                NamedCommands.registerCommand("shoot", shooter.shootAuto());
                NamedCommands.registerCommand("intake", intake.autointake1());
                // NamedCommands.registerCommand("climbleft", Climber.climbCommand());
                // NamedCommands.registerCommand("climbmid", climber.climbCommand());
                // NamedCommands.registerCommand("climbright", climber.climbCommand());

                autoFactory = drivetrain.createAutoFactory();
                autoRoutines = new AutoRoutines(autoFactory);

                autoChooser.addRoutine("Shootclimbleft",
                                () -> autoRoutines.shootclimbleft(shooter.shoot(), climber.Climber_Out()));
                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();
        }

        public Command getAutonomousCommand() {
                /* Run the routine selected from the auto chooser */
                return autoChooser.selectedCommand();
        }

}

    public RobotContainer() {
        NamedCommands.registerCommand("shoot", shooter.shoot());
        NamedCommands.registerCommand("intake", );
        //NamedCommands.registerCommand("climbleft", Climber.climbCommand());
        //NamedCommands.registerCommand("climbmid", climber.climbCommand());
        //NamedCommands.registerCommand("climbright", climber.climbCommand());

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("Shootclimbleft",
                () -> autoRoutines.shootclimbleft(Shooter.shoot(), Command.climbLeft()));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        configureBindings();
    }

        public Command getAutonomousCommand() {
                /* Run the routine selected from the auto chooser */
                return autoChooser.selectedCommand();
        }
}
