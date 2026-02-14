package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.command.shoot.Shootout;
import frc.robot.command.intake.Intakeout;
import frc.robot.command.shoot.Shootstop;
import frc.robot.command.intake.Intakeback;
import frc.robot.command.intake.Deploy;
import frc.robot.command.auto.Newleftshoot2cycle;
import frc.robot.command.auto.Newmidshoot2cycle;
import frc.robot.command.auto.Newmidshoot2cycle;
import frc.robot.command.auto.Newleftshoot2cycle;
import choreo.auto.AutoChooser;

//import com.pathplanner.lib.path.PathPlannerPath;



import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); //最大速
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 最大角速

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% 死區(搖桿靈敏度下降)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors(開放迴圈控制)
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();//(X行煞車)內八
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();//方向控制

    private final Telemetry logger = new Telemetry(MaxSpeed);//即時數據傳送

    private final CommandXboxController joystick = new CommandXboxController(0);
   // private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();//子系統

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    private final Shootout shootout = new Shootout(shooter);
    private final Shootstop shootstop = new Shootstop(shooter);
    private final Intakeback intakeback = new Intakeback(intake);
    private final Intakeout intakeout = new Intakeout(intake);
    private final Deploy deploy = new Deploy(intake);
    private final Newmidshoot2cycle Newmidshoot2cycle = new Newmidshoot2cycle(shooter, intake);
    private final Newleftshoot2cycle Newleftshoot2cycle = new Newleftshoot2cycle(shooter, intake);
    private SendableChooser<Command> autoChooser;


    public RobotContainer() {
    NamedCommands.registerCommand("Newmidshoot2cycle", Newmidshoot2cycle);
    NamedCommands.registerCommand("Newleftshoot2cycle", Newleftshoot2cycle);
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    }

    private void configureBindings() {

            //    joystick2.x().whileTrue(shootout).onFalse(shootstop);

            //     joystick2.a().whileTrue(deploy);

            //     // intake按鍵
            //     joystick2.y().whileTrue(intakeout).onFalse(intakeback);

            //     shooter.angle(joystick2.getLeftY()*0.3);

                joystick.b().onTrue(new InstantCommand(()-> drivetrain.resetRotation()));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
            drivetrain.resetPose(new Pose2d());
            drivetrain.seedFieldCentric();
        }));
        
        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    
}

}
