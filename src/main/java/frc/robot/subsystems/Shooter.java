package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkMax Intake_Roller = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax Intake_Ctrl = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

  private final SparkClosedLoopController intakePID = Intake_Roller.getClosedLoopController();
  private final SparkClosedLoopController conveyorPID = Intake_Ctrl.getClosedLoopController();

  public Intake() {
    SparkMaxConfig Rollerconfig = new SparkMaxConfig();
    // RollerConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    Rollerconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IntakeConstants.Roller_Out_P)
        .i(IntakeConstants.Roller_Out_I)
        .d(IntakeConstants.Roller_Out_D)
        .velocityFF(IntakeConstants.Roller_Out_F).maxMotion
        .maxVelocity(IntakeConstants.ROLLER_MAX_ACCEL) // RPM
        .maxAcceleration(IntakeConstants.ROLLER_MAX_VELOCITY)// RPM/s
        .allowedClosedLoopError(0.05);

    Rollerconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IntakeConstants.Roller_Back_P)
        .i(IntakeConstants.Roller_Back_I)
        .d(IntakeConstants.Roller_Back_D)
        .velocityFF(IntakeConstants.Roller_Back_F).maxMotion
        .maxVelocity(IntakeConstants.ROLLER_MAX_ACCEL) // RPM
        .maxAcceleration(IntakeConstants.ROLLER_MAX_VELOCITY)// RPM/s
        .allowedClosedLoopError(0.05);

    SparkMaxConfig CTRLconfig = new SparkMaxConfig();
    // CTRLConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    CTRLconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IntakeConstants.Intake_Out_P)
        .i(IntakeConstants.Intake_Out_I)
        .d(IntakeConstants.Intake_Out_D)
        .velocityFF(IntakeConstants.Intake_Out_F).maxMotion
        .maxVelocity(IntakeConstants.INTAKE_MAX_ACCEL) // RPM
        .maxAcceleration(IntakeConstants.INTAKE_MAX_VELOCITY) // RPM/s
        .allowedClosedLoopError(0.05);

    CTRLconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IntakeConstants.Intake_Back_P)
        .i(IntakeConstants.Intake_Back_I)
        .d(IntakeConstants.Intake_Back_D)
        .velocityFF(IntakeConstants.Intake_Back_F).maxMotion
        .maxVelocity(IntakeConstants.INTAKE_MAX_ACCEL) // RPM
        .maxAcceleration(IntakeConstants.INTAKE_MAX_VELOCITY) // RPM/s
        .allowedClosedLoopError(0.05);

    Intake_Roller.configure(CTRLconfig, SparkMax.ResetMode.kResetSafeParameters,

        SparkMax.PersistMode.kPersistParameters);
    Intake_Ctrl.configure(CTRLconfig, SparkMax.ResetMode.kResetSafeParameters,

        SparkMax.PersistMode.kPersistParameters);
  }
  // 清空原本設定，套用新的

  public void runIntake(double rpm) {
    intakePID.setSetpoint(rpm, SparkMax.ControlType.kVelocity);
  }

  public void runConveyor(double rpm) {
    conveyorPID.setSetpoint(rpm, SparkMax.ControlType.kVelocity);
  }

  public void Intake_Zero() {
    conveyorPID.setSetpoint(IntakeConstants.Intake_Zero, SparkMax.ControlType.kMAXMotionPositionControl);
  }

  public void Intake_out() {
    conveyorPID.setSetpoint(IntakeConstants.Intake_Out, SparkMax.ControlType.kMAXMotionPositionControl);
  }

  public void Intake_Back() {

    conveyorPID.setSetpoint(
        IntakeConstants.Intake_In,
        SparkMax.ControlType.kMAXMotionPositionControl,
        com.revrobotics.spark.ClosedLoopSlot.kSlot1);
  }

  public void Intake_back() {
    Intake_Ctrl.set(0.9);
  }

  public void Intake_Stop() {
    Intake_Ctrl.set(0);
  }

  public void suck() {
    Intake_Roller.set(1);
  }

  public void shoot() {
    Intake_Roller.set(-0.5);
  }

  public void Stop() {
    Intake_Ctrl.set(0);
    Intake_Roller.set(0);
  }

  public void stopAll() {
    Intake_Roller.stopMotor();
    Intake_Ctrl.stopMotor();
  }
}
