package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Constants {

    public static final CANBus CANIVORE_BUS = new CANBus("888");

    public static class IntakeConstants {

        public static final int Roller_ID = 11; // 待改
        public static final int Deploy_ID = 12;

        public static final double Intake_Zero = 0;
        public static final double Intake_Out = -0.3;
        public static final double Intake_Back = 0.7;

        public static final double Roller_Zero = 0;
        public static final double Roller_Out = 0.7;

        public static final double Roller_Out_P = 0.1;
        public static final double Roller_Out_I = 0;
        public static final double Roller_Out_D = 0.001;
        public static final double Roller_Out_F = 0;

        public static final double Intake_Out_P = 0.1;
        public static final double Intake_Out_I = 0;
        public static final double Intake_Out_D = 0.001;
        public static final double Intake_Out_F = 0;

        public static final double Intake_Back_P = 0.1;
        public static final double Intake_Back_I = 0;
        public static final double Intake_Back_D = 0.001;
        public static final double Intake_Back_F = 0;

        public static final double INTAKE_MAX_ACCEL = 100;
        public static final double INTAKE_MAX_VELOCITY = 50;

        public static final double ROLLER_MAX_ACCEL = 100;
        public static final double ROLLER_MAX_VELOCITY = 50;

    }

    public static class ClimberConstants {
        // Climber ID
        public static final int climberMotor_ID = 41; // 待改
        public static final int tubeMotor1_ID = 42; // 待改
        public static final int tubeMotor2_ID = 43; // 待改

        // Climber Confi
        public static final double Climb_Zero = 178;
        public static final double Climber_Angle = 0;
        public static final double Line_Out = 0;
        public static final double Line_Back = 0;

        // Climber PIDF
        public static final double Climber_Angle_P = 0;
        public static final double Climber_Angle_I = 0;
        public static final double Climber_Angle_D = 0;
        public static final double Climber_Angle_F = 0;

        // Line PIDF
        public static final double Line_Out_P = 0;
        public static final double Line_Out_I = 0;
        public static final double Line_Out_D = 0;
        public static final double Line_Out_F = 0;

        public static final double Line_Back_P = 0;
        public static final double Line_Back_I = 0;
        public static final double Line_Back_D = 0;
        public static final double Line_Back_F = 0;

        public static final double MAX_ACCEL = 100;
        public static final double MAX_VELOCITY = 400;

        public static final double floor = 0; // 我還不知道怎麼填 先亂填
        public static final double L1 = 0;
        public static final double L2 = 0;
        public static final double L3 = 0;

    }

    public static class ShooterConstants {
        // Shooter ID
        public static final int BigFlyWheel_ID = 13;
        public static final int SmallFlyWheel_ID = 11;
        public static final int angleMotor_ID = 2;

        // Shooter Config
        public static final boolean BigFlyWheel_Inverted = false;
        public static final boolean SmallFlyWheel_Inverted = false;
        public static final boolean superneo_Inverted = false;
        // public static final double Climb_Angle = -30;
        public static final double ShooterB_Out = 0.5;
        public static final double ShooterS_Out = 0.5;
        public static final double ShooterB_Back = 0;
        public static final double ShooterS_Back = 0;

        // Shooter BigFlyWheel PIDF
        public static final double ShooterB_Out_P = 0.1;
        public static final double ShooterB_Out_I = 0;
        public static final double ShooterB_Out_D = 0.001;
        public static final double ShooterB_Out_F = 0;

        public static final double ShooterB_MAX_ACCEL = 100;
        public static final double ShooterB_MAX_VELOCITY = 50;


        public static final double superneo_Zero = 0;
        public static final double superneo_Out = 3;
        public static final double superneo_Back = 0;

        public static final double superneo_Out_P = 0.1;
        public static final double superneo_Out_I = 0.0;
        public static final double superneo_Out_D = 0.001;
        public static final double superneo_Out_F = 0;

        public static final double superneo_Back_P = 0.1;
        public static final double superneo_Back_I = 0;
        public static final double superneo_Back_D = 0.001;
        public static final double superneo_Back_F = 0;
        public static final double superneo_MAX_ACCEL = 100;
        public static final double superneo_MAX_VELOCITY = 50;

    }

    public static class IndexerConstants {
        // IndexerMT ID
        public static final int Indexer_ID = 12;
        public static final int Conveyor_ID = 1;

        public static final double indexer_Run = 0.2;
        public static final double indexer_Zero = 0;
        public static final double conveyor_Run = -0.4;
        public static final double conveyor_Zero = 0;

        // IndexerMT Config
        public static final boolean indexer_Inverted = false;
        // public static final double Climb_Angle = -30;

        // IndexerMT PIDF
        public static final double conveyor_Run_P = 0.1;
        public static final double conveyor_Run_I = 0;
        public static final double conveyor_Run_D = 0.001;
        public static final double conveyor_Run_F = 0.;

        public static final double indexer_Run_P = 0.08;
        public static final double indexer_Run_I = 0;
        public static final double indexer_Run_D = 0.0095;
        public static final double indexer_Run_F = 0;

        public static final double MAX_ACCEL = 100;
        public static final double MAX_VELOCITY = 50;

    }

    public static class VisionConstants {
        public static final String LLName = "light";
    }
}