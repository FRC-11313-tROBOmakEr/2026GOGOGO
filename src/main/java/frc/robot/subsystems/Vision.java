package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.VisionModule;
import com.ctre.phoenix6.swerve.SwerveModule;

public class Vision extends SubsystemBase {
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.32, 0.385), // FL
            new Translation2d(0.32, -0.385), // FR
            new Translation2d(-0.32, 0.385), // BL
            new Translation2d(-0.32, -0.385) // BR
    );

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field = new Field2d();
    private final Pigeon2 pigeon = new Pigeon2(12);

    private final VisionModule m_frontLeft = new VisionModule(1, 2, 13);
    private final VisionModule m_frontRight = new VisionModule(3, 4, 14);
    private final VisionModule m_backLeft = new VisionModule(5, 6, 15);
    private final VisionModule m_backRight = new VisionModule(7, 8, 16);

    // private double m_yawRate = 0;

    public Vision() {

        // 初始化 PoseEstimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
                kDriveKinematics,
                getGyroRotation(),
                getModulePositions(),
                new Pose2d());

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {

        // 更新 Odometry (底盤編碼器部分)
        m_poseEstimator.update(getGyroRotation(), getModulePositions());// 反應快 有累積誤差(Drift)

        // 取得目前的陀螺儀 Yaw 角度
        double m_yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();

        // 從 LimelightHelpers 取得 MT2 結果
        LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("light");

        // 檢查是否有看到 Target 且數據可信
        if (mt2Estimate != null && mt2Estimate.tagCount > 0) {// 至少看到一個標籤
            // 加入視覺測量值到 PoseEstimator
            if (Math.abs(m_yawRate) < 720) {
                if (mt2Estimate.avgTagDist < 4.0) { // 忽略太遠的數據
                    m_poseEstimator.addVisionMeasurement( // 估計器(融合位置的數據)
                            mt2Estimate.pose,
                            mt2Estimate.timestampSeconds);
                }
            }
        }

        // 從 PoseEstimator 取得融合後的最終結果傳到 SmartDashboard 上
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
    }

    // 輔助方法：獲取陀螺儀 Rotation2d
    public Rotation2d getGyroRotation() {
        return pigeon.getRotation2d();
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
}
