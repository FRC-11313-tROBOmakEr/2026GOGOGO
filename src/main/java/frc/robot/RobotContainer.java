
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.command.Auto.Leftshoot2cycle;
import frc.robot.command.Intake.Intakein;
import frc.robot.command.Intake.Intakeout;
import frc.robot.command.Intake.Intakesuck;
import frc.robot.command.Shoot.Shooterout;
import frc.robot.command.Shoot.Shootin;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
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

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // private final Climber climber = new Climber();

        /* Path follower */
        private  AutoFactory autoFactory;
       // private final AutoRoutines autoRoutines;
        private final Shooter shooter = new Shooter();
        // private final Climber climber = new Climber();
        private final Intake intake = new Intake();
        //private final Climber climber = new Climber();

        // Target
       
        private final SwerveRequest.FieldCentricFacingAngle aimDrive = new SwerveRequest.FieldCentricFacingAngle();

        private  Shootin shootin = new Shootin(shooter);
        private  Shooterout shooterout = new Shooterout(shooter, drivetrain);
        private  Intakein intakein = new Intakein(intake);
        private  Intakeout intakeout= new Intakeout(intake);
        private Intakesuck intakesuck = new Intakesuck(intake);
        private  Leftshoot2cycle leftshoot2cycle = new Leftshoot2cycle(shooter, intake);

        private final SendableChooser<Command> autoChooser; 

    public RobotContainer() {
        NamedCommands.registerCommand("leftshoot2cycle", leftshoot2cycle);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }



        public  void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive
                                                   .withVelocityY(-xboxController.getLeftX() * MaxSpeed)                                                                          
                                                   .withRotationalRate(-xboxController.getRightX() * MaxAngularRate)));

                // Idle while th10e robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                xboxController.b().whileTrue(drivetrain.applyRequest(() -> point
                                            .withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));
                // 我加的
                xboxController.x().whileTrue(shooterout).onFalse(shootin);

                // pov是xboxcontroller十字按鈕(有上下左右)
                xboxController.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                xboxController.povDown().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

           

                // intake按鍵
                xboxController.x().whileTrue(intakeout).onFalse(intakein);
                xboxController.y().whileTrue(intakesuck);

                // xboxController.y().whileTrue(intake.intakeAndExtension(0.5))
                //                 .onFalse(intake.stopIntakeAndBack());
                //這是啥???
                // climber按鍵
                // xboxController.leftBumper()
                //                 .whileTrue(climber.Climber_Out(0.5));// 執行的動作//抓住然後捲線
                // xboxController.rightBumper()
                //                 .whileTrue(climber.Climber_Back(-0.5));
                // xboxController.leftTrigger()
                //                 .whileTrue(climber.Line_Out(0.5));
                // xboxController.rightTrigger()
                //                 .whileTrue(climber.Line_back(-0.333333));

                // .whileTrue(climber.run(()-> climber.extend()))
                // .onFalse((climber.runOnce)()->climber.stop());

                // pov是xboxcontroller十字按鈕(有上下左右)
               

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
                drivetrain.registerTelemetry(logger::telemeterize);
        }
}