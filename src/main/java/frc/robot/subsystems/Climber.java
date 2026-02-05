package frc.robot.subsystems;


import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;


public class Climber extends SubsystemBase {


  private final TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotor_ID, Constants.CANIVORE_BUS);
  private final TalonFX tubeMotor1 = new TalonFX(ClimberConstants.tubeMotor1_ID, Constants.CANIVORE_BUS);
  private final TalonFX tubeMotor2 = new TalonFX(ClimberConstants.tubeMotor2_ID, Constants.CANIVORE_BUS);


  public Climber() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    TalonFXConfiguration tubeConfig = new TalonFXConfiguration();


    tubeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


    climberConfig.Slot0.kP = ClimberConstants.Climber_Angle_P;
    climberConfig.Slot0.kI = ClimberConstants.Climber_Angle_I;
    climberConfig.Slot0.kD = ClimberConstants.Climber_Angle_D;
    climberConfig.Slot0.kV = ClimberConstants.Climber_Angle_F;


    climberConfig.Slot1.kP = ClimberConstants.Line_Out_P;
    climberConfig.Slot1.kI = ClimberConstants.Line_Out_I;
    climberConfig.Slot1.kD = ClimberConstants.Line_Out_D;
    climberConfig.Slot1.kV = ClimberConstants.Line_Out_F;


    climberMotor.getConfigurator().apply(climberConfig);
    tubeMotor1.getConfigurator().apply(tubeConfig);
    tubeMotor2.getConfigurator().apply(tubeConfig);
    
    tubeMotor2.setControl(new Follower(tubeMotor1.getDeviceID(), MotorAlignmentValue.Aligned));


    if (DriverStation.isEnabled()) {
      climberMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Zero));
    }
  }


  public void Climb_Zero() {
    climberMotor.setControl(new MotionMagicVelocityVoltage(ClimberConstants.Climb_Zero).withSlot(0));
    tubeMotor1.setControl(new MotionMagicVelocityVoltage(ClimberConstants.Climb_Zero).withSlot(0));
  }


  public void Climb_Angle() {
    climberMotor.setControl(new MotionMagicDutyCycle(IntakeConstants.Intake_Out));
  }


  public void Line_out() {
    climberMotor.setControl(new MotionMagicDutyCycle(IntakeConstants.Intake_Out).withSlot(1));
  }


  public void Line_Back() {
    tubeMotor1.set(-0.6);
  }


  public void stopTube() {
    tubeMotor1.set(0);
  }


  public void stopClimber() {
    climberMotor.set(0);
  }


  public void stopAll() {
    climberMotor.stopMotor();
    tubeMotor1.stopMotor();
  }
}



