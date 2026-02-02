// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
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

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xboxController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final Climber climber = new Climber();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private  AutoChooser autoChooser;

    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Intake intake = new Intake();

      
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed)   // Drive
                                                                                                   // forward
                                                                                                   // with
                                                                                                   // negative
                                                                                                   // Y
                                                                                                   // (forward)
                                           .withVelocityY(-xboxController.getLeftX() * MaxSpeed)   // Drive left with
                                                                                                   // negative X
                                                                                                   // (left)
                                           .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) 
                                                                                                   // Drive
                                                                                                   // counterclockwise
                                                                                                   // with
                                                                                                   // negative
                                                                                                   // X
                                                                                                   // (left)
                                ));

    // Idle while th10e robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
                      drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    xboxController.b().whileTrue(drivetrain.applyRequest(() -> point
                      .withModuleDirection(new Rotation2d(-xboxController.getLeftY(),
                                                          -xboxController.getLeftX()))));
    // 我加的
    xboxController.a().whileTrue(shooter.shoot()).onFalse(shooter.back());

    // intake按鍵
    xboxController.y().whileTrue(intake.intakeAndExtension())
                      .onFalse(intake.stopIntakeAndBack());

    // climber按鍵
    xboxController.leftBumper()
                  .whileTrue(climber.Climber_Out());// 執行的動作//抓住然後捲線
    xboxController.rightBumper()
                  .whileTrue(climber.Climber_Back());
    xboxController.leftTrigger()
                  .whileTrue(climber.Line_Out());
    xboxController.rightTrigger()
                .whileTrue(climber.Line_back());

    // .whileTrue(climber.run(()-> climber.extend()))
    // .onFalse((climber.runOnce)()->climber.stop());

    // pov是xboxcontroller十字按鈕(有上下左右)
    xboxController.povUp().whileTrue(drivetrain.applyRequest(
                  () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    xboxController.povDown().whileTrue(drivetrain.applyRequest(
                  () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // Run SysId routines when holdi ng back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.

    xboxController.back().and(xboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    xboxController.back().and(xboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    xboxController.start().and(xboxController.y())
                  .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    xboxController.start().and(xboxController.x())
                  .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    xboxController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    }

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
