package frc.robot;

public final class Constants {

    public static class IntakeContants {
        public static final int Intake_Roller_ID = 21;
        public static final int Intake_Ctrl_ID = 22;

        public static final double Intake_Zero = 0;
        public static final double Intake_Out = 0;
        public static final double Intake_In = 0;

        public static final double MAX_ACCEL = 500;
        public static final double MAX_VELOCITY = 200;

        // RollerMT的PID
        public static final double Roller_Out_P = 0.4;
        public static final double Roller_Out_I = 0;
        public static final double Roller_Out_D = 0;
        public static final double Roller_Out_F = 0;

        public static final double Roller_Back_P = 0.4;
        public static final double Roller_Back_I = 0;
        public static final double Roller_Back_D = 0;
        public static final double Roller_Back_F = 0;

        //
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
        public static final int climberMotor_ID = 41; // 待改
        public static final int tubeMotor1_ID = 42; // 待改
        public static final int tubeMotor2_ID = 43; // 待改

        // Climber Config
        public static final boolean climberMotor_Inverted = false;
        public static final boolean tubeMotor1_Inverted = false;
        public static final boolean tubeMotor2_Inverted = true;
        // public static final double Climb_Angle = -30;
        public static final double Climb_Zero = 178;
        public static final double Climb_StartUp = -24;

        // Climber PIDF
        public static final double Climber_Out_P = 0;
        public static final double Climber_Out_I = 0;
        public static final double Climber_Out_D = 0;
        public static final double Climber_Out_F = 0;

        public static final double Climber_Back_P = 0;
        public static final double Climber_Back_I = 0;
        public static final double Climber_Back_D = 0;
        public static final double Climber_Back_F = 0;

        // Line PIDF
        public static final double Line_Out_P = 0;
        public static final double Line_Out_I = 0;
        public static final double Line_Out_D = 0;
        public static final double Line_Out_F = 0;

        public static final double Line_Back_P = 0;
        public static final double Line_Back_I = 0;
        public static final double Line_Back_D = 0;
        public static final double Line_Back_F = 0;

        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;

    }

    public static class ShooterConstants {
        // Shooter ID
        public static final int BigFlyWheel_ID = 44; // 待改
        public static final int SmallFlyWheel_ID = 45; // 待改
        public static final int superneo_ID = 46; // 待改

        // Shooter Config
        public static final boolean BigFlyWheel_Inverted = false;
        public static final boolean SmallFlyWheel_Inverted = false;
        public static final boolean superneo_Inverted = false;
        // public static final double Climb_Angle = -30;
        public static final double Shooter_Zero = 178;
        public static final double Shooter_StartUp = -24;

        // Shooter BigFlyWheel PIDF
        public static final double Shooter_Out_P = 0;
        public static final double Shooter_Out_I = 0;
        public static final double Shooter_Out_D = 0;
        public static final double Shooter_Out_F = 0;

        public static final double Shooter_Back_P = 0;
        public static final double Shooter_Back_I = 0;
        public static final double Shooter_Back_D = 0;
        public static final double Shooter_Back_F = 0;

        // Shooter SmallFlyWheel PIDF
        public static final double Shooters_Out_P = 0;
        public static final double Shooters_Out_I = 0;
        public static final double Shooters_Out_D = 0;
        public static final double Shooters_Out_F = 0;

        public static final double Shooters_Back_P = 0;
        public static final double Shooters_Back_I = 0;
        public static final double Shooters_Back_D = 0;
        public static final double Shooters_Back_F = 0;

        // Shooter superneo PIDF
        public static final double superneo_Out_P = 0;
        public static final double superneo_Out_I = 0;
        public static final double superneo_Out_D = 0;
        public static final double superneo_Out_F = 0;

        public static final double superneo_Back_P = 0;
        public static final double superneo_Back_I = 0;
        public static final double suppernet_Back_D = 0;
        public static final double supperneo_Back_F = 0;


        //最大加速度,最大速度
        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;

    }

    public static class IndexerConstants {
        // IndexerMT ID
        public static final int indexerMT_ID = 47; // 待改

        // IndexerMT Config
        public static final boolean indexerMT_Inverted = false;
        // public static final double Climb_Angle = -30;
        public static final double indexerMT_Zero = 178;
        public static final double indexerMT_StartUp = -24;

        // IndexerMT PIDF
        public static final double indexerMT_Out_P = 0;
        public static final double indexerMT_Out_I = 0;
        public static final double indexerMT_Out_D = 0;
        public static final double indexerMT_Out_F = 0;

        public static final double indexerMT_Back_P = 0;
        public static final double indexerMT_Back_I = 0;
        public static final double indexerMT_Back_D = 0;
        public static final double indexerMT_Back_F = 0;

        //最大加速度,最大速度
        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;

    }
}
