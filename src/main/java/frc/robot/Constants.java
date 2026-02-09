package frc.robot;


import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;

public final class Constants {

    public static final CANBus CANIVORE_BUS = new CANBus("canbus");

    public static class IntakeConstants {

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

        // Shooter Config
        public static final boolean BigFlyWheel_Inverted = false;
        public static final boolean SmallFlyWheel_Inverted = false;
        public static final boolean superneo_Inverted = false;
        // public static final double Climb_Angle = -30;
        public static final double ShooterB_Zero = 0;
        public static final double ShooterS_Zero = 0;
        public static final double ShooterB_Out = 3;
        public static final double ShooterS_Out = 3;
        public static final double ShooterB_Back = 0;
        public static final double ShooterS_Back = 0;

        // Shooter BigFlyWheel PIDF
        public static final double ShooterB_Out_P = 0;
        public static final double ShooterB_Out_I = 0;
        public static final double ShooterB_Out_D = 0;
        public static final double ShooterB_Out_F = 0;

        public static final double ShooterB_MAX_ACCEL = 1000;
        public static final double ShooterB_MAX_VELOCITY = 400;

        // Shooter SmallFlyWheel PIDF
        public static final double ShooterS_Out_P = 0;
        public static final double ShooterS_Out_I = 0;
        public static final double ShooterS_Out_D = 0;
        public static final double ShooterS_Out_F = 0;

        public static final double ShooterS_MAX_ACCEL = 1000;
        public static final double ShooterS_MAX_VELOCITY = 400;

        //Indexer IndexerMT2 PID
        public static final double conveyorrun_P = 0;
        public static final double conveyorrun_I = 0;
        public static final double conveyorrun_D = 0;
        public static final double conveyorrun_F = 0;
        public static final double conveyorrun = 3;

        // Shooter superneo PIDF

        public static final double superneo_Zero = 0;
        public static final double superneo_Out = 3;
        public static final double superneo_Back = 0;
        
        public static final double superneo_Out_P = 0;
        public static final double superneo_Out_I = 0;
        public static final double superneo_Out_D = 0;
        public static final double superneo_Out_F = 0;

        public static final double superneo_Back_P = 0;
        public static final double superneo_Back_I = 0;
        public static final double superneo_Back_D = 0;
        public static final double superneo_Back_F = 0;
        public static final double superneo_MAX_ACCEL = 1000;
        public static final double superneo_MAX_VELOCITY = 400;

        public static final double indexer_Out_P = 0;
        public static final double indexer_Out_I = 0;
        public static final double indexer_Out_D = 0;
        public static final double indexer_Out_F = 0;

        public static final double indexer_MAX_ACCEL = 1000;
        public static final double indexer_MAX_VELOCITY = 400;

    }

    public static class IndexerConstants {
        // IndexerMT ID
        public static final int Indexer_ID = 47; // 待改
        public static final int Conveyor_ID = 46;

        public static final double indexer_Run = 3;
        public static final double indexer_Zero = 0;        
        public static final double conveyor_Run = 3;
        public static final double conveyor_Zero = 0;

        

        // IndexerMT Config
        public static final boolean indexer_Inverted = false;
        // public static final double Climb_Angle = -30;
        public static final double indexer_StartUp = -24;


        // IndexerMT PIDF
        public static final double conveyor_Run_P = 0;
        public static final double conveyor_Run_I = 0;
        public static final double conveyor_Run_D = 0;
        public static final double conveyor_Run_F = 0;


        public static final double indexer_Run_P = 0;
        public static final double indexer_Run_I = 0;
        public static final double indexer_Run_D = 0;
        public static final double indexer_Run_F = 0;




        public static final double MAX_ACCEL = 1000;
        public static final double MAX_VELOCITY = 400;

    }



    public static class VisionConstants {
        public static final String LLName = "light";
    }

    public static class driveConstants {
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final Pose3d redHubPose = new Pose3d(Units.Inches.of(468.56), Units.Inches.of(158.32), Units.Inches.of(72.0), new Rotation3d());
        public static final Pose3d blueHubPose = new Pose3d(Units.Inches.of(152.56), Units.Inches.of(158.32),  Units.Inches.of(72.0), new Rotation3d());

        public static final Angle epsilonAngleToGoal = Degree.of(0.1); //robot will stop if it's in range of 5deg

        public static final PIDController rotationController = getRotationController();

        private static final PIDController getRotationController() {
            PIDController controller = new PIDController(1.5, 0.0, 0.00514); 
            controller.enableContinuousInput(-Math.PI, Math.PI);
            return controller;
        }

        public static final Pose3d getHubPose() {
            Pose3d hubPose = DriverStation.getAlliance().equals(Optional.of(Alliance.Blue)) ? blueHubPose : redHubPose;
            return hubPose;
        }
    }
}