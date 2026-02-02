package frc.robot;

public final class Constants{
    
  public static class Intakecontants{
    public static final int Intake_Ctrl_ID = 21;

    public static final double Intake_Zero = 0;
    public static final double Intake_Out = 0;
    public static final double Intake_In = 0;

    public static final double MAX_ACCEL=500;
    public static final double MAX_VELOCITY=200;

    public static final double Intake_Out_P = 0.4;
    public static final double Intake_Out_I = 0;
    public static final double Intake_Out_D = 0;
    public static final double Intake_Out_F = 0;

    public static final double Intake_Back_P = 0.4;
    public static final double Intake_Back_I = 0;
    public static final double Intake_Back_D = 0;
    public static final double Intake_Back_F = 0;
  
  }

   public static class ClimberConstants {
        // Climber ID
        public static final int Climb_Motor = 41;

   
        // Climber Config
        public static final boolean climberMotor_Inverted = false;
        public static final boolean tubeMotor_ = false;
        public static final double Climb_Angle = -30;
        public static final double Climb_Zero = 178;
        public static final double Climb_StartUp = -24;

        // Climber PIDF
        public static final double P = 1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;

        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;
    }
}

