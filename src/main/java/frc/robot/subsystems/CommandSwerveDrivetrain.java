package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController m_pathXController = new PIDController(10, 0, 0);
    private final PIDController m_pathYController = new PIDController(10, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds speedsApplier = new SwerveRequest.ApplyRobotSpeeds();

    private LimelightHelpers.PoseEstimate mt2;
    private boolean doRejectUpdate, badTagData;
  

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePPAuto();
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePPAuto();
    }

    private void configurePPAuto() {
        try {
            AutoBuilder.configure(
                    () -> getState().Pose,
                    (pose) -> resetPose(pose),
                    () -> getState().Speeds,
                    (speeds, feedForwards) -> setControl(
                            speedsApplier.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedForwards.robotRelativeForcesX())
                                    .withWheelForceFeedforwardsY(feedForwards.robotRelativeForcesY())),
                    new PPHolonomicDriveController(
                            new PIDConstants(5, 0, 0),
                            new PIDConstants(5, 0, 0)),
                    RobotConfig.fromGUISettings(),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception error) {
            DriverStation.reportError("Error occured when configuring AutoBuilder: ", error.getStackTrace());
        }
    }

    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {
        });
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                () -> getState().Pose,
                this::resetPose,
                this::followPath,
                true,
                this,
                trajLogger);
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
                pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
                pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading);

        setControl(
                m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // Auto align
    public final SwerveRequest.FieldCentric alignDrive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.maxSpeed * 0.1).withRotationalDeadband(DriveConstants.maxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public Command autoAlignCommand(CommandXboxController driverCtrl) {
        return applyRequest(() -> {
            double controllerVelX = -driverCtrl.getLeftX();
            double controllerVelY = -driverCtrl.getLeftY();

            Pose2d drivePose = getState().Pose;
            Pose2d targetPose = DriveConstants.getHubPose().toPose2d();
            
            Translation2d deltaDis = targetPose.relativeTo(drivePose).getTranslation();
            
            Rotation2d desiredAngle = deltaDis.getAngle();
            Rotation2d currentAngle = drivePose.getRotation();
            Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
            
            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180, 180);
            
            if ((Math.abs(wrappedAngleDeg) < DriveConstants.epsilonAngleToGoal.in(Degrees)) // if facing goal already
               && Math.hypot(controllerVelX, controllerVelY) < 0.1) {
                return new SwerveRequest.SwerveDriveBrake();
            } else {
                double vx = controllerVelY * DriveConstants.maxSpeed;
                double vy = controllerVelX * DriveConstants.maxSpeed;
                
                // feedforward
                double dx = deltaDis.getX(), dy = deltaDis.getY();
                double rSquare = (dx*dx + dy*dy);

                // if (rSquare < 1e-4) {
                    // rSquare = 0;
                // }
                
                double vw = ((dy*vx - dx*vy)) / rSquare ; // 先把PID設成0試試，可能需要在分子加負號
                    
                // feedback
                double feedback = DriveConstants.rotationController.calculate(currentAngle.getRadians(), desiredAngle.getRadians());

                vw += feedback;
                vw = MathUtil.clamp(vw, -DriveConstants.maxAngularRate, DriveConstants.maxAngularRate);
                // System.out.println(vw);
                // System.out.println(vx);
                // System.out.println(vx);
                // System.out.println(" ");

                return alignDrive
                        .withVelocityX(vx) // Drive forward with negative Y (forward)
                        .withVelocityY(vy) // Drive left with negative X (left)
                        .withRotationalRate(vw*1.151); // Use angular rate for rotation (rad/s)
            }
        });
    };

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.LLName, getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);

        doRejectUpdate = false;
        badTagData = false;

        if (LimelightHelpers.getFiducialID(Constants.VisionConstants.LLName) != -1) {
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.LLName);
        }
        else {
            badTagData = true;
        }


        //Pose Estimator
        if (!badTagData) {
            if (Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720){
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0){
                doRejectUpdate = true;
            }
            if (!doRejectUpdate){
                addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }

        // Hint: To get estimated Pose2d from built-in PoseEstimator, use getState().Pose (check L258)

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
    
}
