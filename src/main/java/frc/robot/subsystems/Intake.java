package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
        private final SparkMax roller = new SparkMax(IntakeConstants.Roller_ID, SparkLowLevel.MotorType.kBrushless);
        // TODO: 命名
        private final SparkMax deploy = new SparkMax(IntakeConstants.Deploy_ID, SparkLowLevel.MotorType.kBrushless);

        private final SparkClosedLoopController rollerPID = roller.getClosedLoopController();
        private final SparkClosedLoopController deployPID = deploy.getClosedLoopController();
        private  SlewRateLimiter limiter;


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

                rollerConfig.closedLoop.maxMotion
                .cruiseVelocity(IntakeConstants.ROLLER_MAX_VELOCITY)
                .maxAcceleration(IntakeConstants.ROLLER_MAX_ACCEL);

                // TODO: 跟馬達的命名一起改
                SparkMaxConfig deployConfig = new SparkMaxConfig();
                deployConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
                deployConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(IntakeConstants.Intake_Out_P, ClosedLoopSlot.kSlot0)
                                .i(IntakeConstants.Intake_Out_I, ClosedLoopSlot.kSlot0)
                                .d(IntakeConstants.Intake_Out_D, ClosedLoopSlot.kSlot0).maxMotion
                                .allowedProfileError(0.05, ClosedLoopSlot.kSlot0);

                deployConfig.closedLoop.feedForward
                                .kV(IntakeConstants.Intake_Out_F, ClosedLoopSlot.kSlot0);

                deployConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(IntakeConstants.Intake_Back_P, ClosedLoopSlot.kSlot1)
                                .i(IntakeConstants.Intake_Back_I, ClosedLoopSlot.kSlot1)
                                .d(IntakeConstants.Intake_Back_D, ClosedLoopSlot.kSlot1).maxMotion
                                .allowedProfileError(0.05, ClosedLoopSlot.kSlot1);

                deployConfig.closedLoop.feedForward
                                .kV(IntakeConstants.Intake_Back_F, ClosedLoopSlot.kSlot1);

                deployConfig.closedLoop.maxMotion
                                .cruiseVelocity(IntakeConstants.INTAKE_MAX_VELOCITY)
                                .maxAcceleration(IntakeConstants.INTAKE_MAX_ACCEL);

                // 清空原本設定，套用新的
                // roller.configure(rollerConfig, ResetMode.kResetSafeParameters,
                // PersistMode.kPersistParameters);
                deploy.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void setRollerVelocity(double rpm) {
                rollerPID.setSetpoint(rpm, SparkMax.ControlType.kMAXMotionVelocityControl);
        }

        public void zero() {
                deployPID.setSetpoint(IntakeConstants.Intake_Zero, SparkMax.ControlType.kMAXMotionPositionControl,
                                ClosedLoopSlot.kSlot0);
        }

        public void out() {
                
                deployPID.setSetpoint(IntakeConstants.Intake_Out, SparkMax.ControlType.kDutyCycle,
                                ClosedLoopSlot.kSlot0);
        }

        public void back() {
                deployPID.setSetpoint(IntakeConstants.Intake_Back, SparkMax.ControlType.kDutyCycle,
                                ClosedLoopSlot.kSlot1);
        }

        public void stopDeploy() {
                deploy.stopMotor();
        }

        public void suck() {
                
        rollerPID.setSetpoint(IntakeConstants.Roller_Out, SparkMax.ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
                
        }

 


        public void stopRoller() {
                roller.stopMotor();
        }

        public void stopAll() {
                roller.stopMotor();
                deploy.stopMotor();
        }
}
