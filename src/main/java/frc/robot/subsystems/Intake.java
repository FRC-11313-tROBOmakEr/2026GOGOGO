package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SparkMax roller = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
    // TODO: 命名
    private final SparkMax Intake_Ctrl = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);

    private final SparkClosedLoopController rollerPID = roller.getClosedLoopController();
    private final SparkClosedLoopController ctrlPID = Intake_Ctrl.getClosedLoopController();

    public Intake() {

        SparkBaseConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
        rollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(IntakeConstants.Roller_Out_P, ClosedLoopSlot.kSlot0)
                .i(IntakeConstants.Roller_Out_I, ClosedLoopSlot.kSlot0)
                .d(IntakeConstants.Roller_Out_D, ClosedLoopSlot.kSlot0).maxMotion
                .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

        rollerConfig.closedLoop.feedForward
                .kV(IntakeConstants.Roller_Out_F, ClosedLoopSlot.kSlot0);

        rollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(IntakeConstants.Roller_Back_P, ClosedLoopSlot.kSlot1)
                .i(IntakeConstants.Roller_Back_I, ClosedLoopSlot.kSlot1)
                .d(IntakeConstants.Roller_Back_D, ClosedLoopSlot.kSlot1).maxMotion
                .allowedProfileError(0.05, ClosedLoopSlot.kSlot1);

        rollerConfig.closedLoop.feedForward
                .kV(IntakeConstants.Roller_Back_F, ClosedLoopSlot.kSlot1);

        rollerConfig.closedLoop.maxMotion
                .cruiseVelocity(IntakeConstants.ROLLER_MAX_VELOCITY)
                .maxAcceleration(IntakeConstants.ROLLER_MAX_ACCEL);

        // TODO: 跟馬達的命名一起改
        SparkMaxConfig ctrlConfig = new SparkMaxConfig();
        ctrlConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
        ctrlConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(IntakeConstants.Intake_Out_P, ClosedLoopSlot.kSlot0)
                .i(IntakeConstants.Intake_Out_I, ClosedLoopSlot.kSlot0)
                .d(IntakeConstants.Intake_Out_D, ClosedLoopSlot.kSlot0).maxMotion
                .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

        ctrlConfig.closedLoop.feedForward
                .kV(IntakeConstants.Intake_Out_F, ClosedLoopSlot.kSlot0);

        ctrlConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(IntakeConstants.Intake_Back_P, ClosedLoopSlot.kSlot1)
                .i(IntakeConstants.Intake_Back_I, ClosedLoopSlot.kSlot1)
                .d(IntakeConstants.Intake_Back_D, ClosedLoopSlot.kSlot1).maxMotion
                .allowedProfileError(0.05, ClosedLoopSlot.kSlot1);

        ctrlConfig.closedLoop.feedForward
                .kV(IntakeConstants.Intake_Back_F, ClosedLoopSlot.kSlot1);

        ctrlConfig.closedLoop.maxMotion
                .cruiseVelocity(IntakeConstants.INTAKE_MAX_VELOCITY)
                .maxAcceleration(IntakeConstants.INTAKE_MAX_ACCEL);

        // 清空原本設定，套用新的
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Intake_Ctrl.configure(ctrlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setRollerVelocity(double rpm) {
        rollerPID.setSetpoint(rpm, SparkMax.ControlType.kMAXMotionVelocityControl);
    }

    public void zero() {
        ctrlPID.setSetpoint(IntakeConstants.Intake_Zero, SparkMax.ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0);
    }

    public void out() {
        ctrlPID.setSetpoint(IntakeConstants.Intake_Out, SparkMax.ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0);
    }

    public void back() {
        ctrlPID.setSetpoint(IntakeConstants.Intake_In, SparkMax.ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot1);
    }

    public void intake_dont_do_that() { // TODO: 改名字
        Intake_Ctrl.stopMotor();
    }

    public void suck() {
        roller.set(1.0);
    }

    public void stopRoller() {
        roller.set(0);
    }

    public void stopAll() {
        roller.stopMotor();
        Intake_Ctrl.stopMotor();
    }
}
