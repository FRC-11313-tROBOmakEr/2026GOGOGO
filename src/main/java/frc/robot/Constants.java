package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Constants {
    public static final CANBus CANIVORE_BUS = new CANBus("canbus");

    public static class IntakeConstants {
        public static final int Intake_Roller_ID = 21;
        public static final int Intake_Ctrl_ID = 22;

        public static final double Intake_Zero = 0;
        public static final double Intake_Out = 0;
        public static final double Intake_In = 0;

        // RollerMT的PID
        public static final double Roller_Out_P = 0.0001;
        public static final double Roller_Out_I = 0.0001;
        public static final double Roller_Out_D = 0.01;
        public static final double Roller_Out_F = 0.00017;

        public static final double Roller_Back_P = 0.0001;
        public static final double Roller_Back_I = 0.0001;
        public static final double Roller_Back_D = 0.01;
        public static final double Roller_Back_F = 0.00017;

        public static final double ROLLER_MAX_ACCEL = 2000;
        public static final double ROLLER_MAX_VELOCITY = 1500;

        //
        public static final double Intake_Out_P = 0.0001;
        public static final double Intake_Out_I = 0.0001;
        public static final double Intake_Out_D = 0.01;
        public static final double Intake_Out_F = 0.00017;

        public static final double Intake_Back_P = 0.0001;
        public static final double Intake_Back_I = 0.0001;
        public static final double Intake_Back_D = 0.01;
        public static final double Intake_Back_F = 0.00017;

        public static final double INTAKE_MAX_ACCEL = 2000;
        public static final double INTAKE_MAX_VELOCITY = 1500;

    }

    public static class ClimberConstants {
        // Climber ID
        public static final int climberMotor_ID = 41; // 待改
        public static final int tubeMotor1_ID = 42; // 待改
        public static final int tubeMotor2_ID = 43; // 待改

        // Climber Confi

        public static final double Climb_Angle = -30;
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

        public static final double floor = 0; // 我還不知道怎麼填 先亂填
        public static final double L1 = 0;
        public static final double L2 = 0;
        public static final double L3 = 0;

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

        // 最大加速度,最大速度
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

        // 最大加速度,最大速度
        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;

    }

    public static class VisionConstants {
        public static final String LLName = "light";
    }

<<<<<<< HEAD
}
=======
}
>>>>>>> f9b88e8e84ece603c2f32ca0ed0dd259893a1080
