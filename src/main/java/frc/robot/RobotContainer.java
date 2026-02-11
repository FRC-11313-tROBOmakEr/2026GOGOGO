
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.command.auto.Leftshoot2cycle;
import frc.robot.command.intake.Intakeback;
import frc.robot.command.intake.Intakeout;

import frc.robot.command.shoot.Shootout;
import frc.robot.command.shoot.Shootstop;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
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
        private final CommandXboxController joystic = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // private final Climber climber = new Climber();

        /* Path follower */
        private AutoFactory autoFactory;
        private AutoRoutine autoRoutines;
        private final Shooter shooter = new Shooter();
        // private final Climber climber = new Climber();
        private final Intake intake = new Intake();
        // private final Climber climber = new Climber();

        private final SwerveRequest.FieldCentricFacingAngle aimDrive = new SwerveRequest.FieldCentricFacingAngle();

        private Shootstop shootstop = new Shootstop(shooter);
        private Shootout shootout = new Shootout(shooter);
        private Intakeback intakeback = new Intakeback(intake);
        private Intakeout intakeout = new Intakeout(intake);
        private Leftshoot2cycle leftshoot2cycle = new Leftshoot2cycle(shooter, intake);

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                NamedCommands.registerCommand("leftshoot2cycle", leftshoot2cycle);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();
        }

        public Command getAutonomousCommand() {
                // return autoChooser.getSelected();
                return null;
        }

        public void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive
                                                                                                                         // forward
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // Y
                                                                                                                         // (forward)
                                                .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with
                                                                                                      // negative X
                                                                                                      // (left)
                                                .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive
                                                                                                                  // counterclockwise
                                                                                                                  // with
                                                                                                                  // negative
                                                                                                                  // X
                                                                                                                  // (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                xboxController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
                                new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                xboxController.back().and(xboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                xboxController.back().and(xboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                xboxController.start().and(xboxController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                xboxController.start().and(xboxController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> {
                        drivetrain.resetPose(new Pose2d());
                        drivetrain.seedFieldCentric();
                }));

                drivetrain.registerTelemetry(logger::telemeterize);

                // 我加的
                joystic.x().whileTrue(shootout).onFalse(shootstop);

                // intake按鍵
                joystic.y().whileTrue(new InstantCommand(() -> intake.out())).onFalse(new InstantCommand(() -> intake.back()));

                shooter.angle(joystic.getLeftY()*0.025);
        }

}
