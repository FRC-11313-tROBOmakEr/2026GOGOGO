// package frc.robot.subsystems;


// import frc.robot.Constants.ClimberConstants;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants;


// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;


// public class Climber extends SubsystemBase {
//   private final TalonFX angleMotor = new TalonFX(ClimberConstants.climberMotor_ID, Constants.CANIVORE_BUS);
//   private final TalonFX tubeMotor1 = new TalonFX(ClimberConstants.tubeMotor1_ID, Constants.CANIVORE_BUS);
//   private final TalonFX tubeMotor2 = new TalonFX(ClimberConstants.tubeMotor2_ID, Constants.CANIVORE_BUS);

//   public Climber() {
//     TalonFXConfiguration climberConfig = new TalonFXConfiguration();
//     TalonFXConfiguration tubeConfig = new TalonFXConfiguration();


//     tubeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
//     //tubeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


//     climberConfig.Slot0.kP = ClimberConstants.Climber_Angle_P;
//     climberConfig.Slot0.kI = ClimberConstants.Climber_Angle_I;
//     climberConfig.Slot0.kD = ClimberConstants.Climber_Angle_D;
//     climberConfig.Slot0.kV = ClimberConstants.Climber_Angle_F;


//     tubeConfig.Slot1.kP = ClimberConstants.Line_Out_P;
//     tubeConfig.Slot1.kI = ClimberConstants.Line_Out_I;
//     tubeConfig.Slot1.kD = ClimberConstants.Line_Out_D;
//     tubeConfig.Slot1.kV = ClimberConstants.Line_Out_F;


//     angleMotor.getConfigurator().apply(climberConfig);
//     tubeMotor1.getConfigurator().apply(tubeConfig);
//     tubeMotor2.getConfigurator().apply(tubeConfig);
    
//     tubeMotor2.setControl(new Follower(tubeMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
//   }


//   public void Climb_Zero() {
//     angleMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Zero).withSlot(0));
//     tubeMotor1.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Zero).withSlot(0));
//   }


//   public void Climb_Angle() {
//     angleMotor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climber_Angle));
//   }

//   // TODO: 改成用 MotionMagic 的 Velocity 而非一般 DutyCycle (會變成位置)
//   public void Line_out() {//1 2s
//     tubeMotor1.setControl(new MotionMagicVelocityDutyCycle(ClimberConstants.Line_Out).withSlot(1));
//   }

//   // TODO: 改成 MotionMagic
//   public void Line_Back() {//2
//     tubeMotor1.setControl(new MotionMagicDutyCycle(ClimberConstants.Line_Back));
//   }


//   public void stopTube() {
//     tubeMotor1.stopMotor();
//   }


//   public void stopClimber() {
//     angleMotor.stopMotor();
//   }


//   public void stopAll() {
//     angleMotor.stopMotor();
//     tubeMotor1.stopMotor();
//   }
// }
