package robot.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Ports;

public class Drive extends SubsystemBase {
    private final SparkMax leftLeader = new SparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
    private final DifferentialDriveOdometry odometry;

    public Drive() {
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        globalConfig.idleMode(IdleMode.kBrake);

        leftLeaderConfig.apply(globalConfig).inverted(true).encoder.positionConversionFactor(DriveConstants.POSITION_FACTOR).velocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        rightLeaderConfig.apply(globalConfig).inverted(false).encoder.positionConversionFactor(DriveConstants.POSITION_FACTOR).velocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        leftFollowerConfig.apply(globalConfig).follow(Ports.Drive.LEFT_LEADER);
        rightFollowerConfig.apply(globalConfig).follow(Ports.Drive.RIGHT_LEADER);

        leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        gyro.reset();
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(), 
            0, 
            0, 
            new Pose2d());
    }
    
    private void drive(double leftSpeed, double rightSpeed) {
        leftLeader.set(leftSpeed);
        rightLeader.set(rightSpeed);
    }

    public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
        return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
    }

    private void updateOdometry(Rotation2d rotation) {
        odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    @Override 
    public void periodic() {
        updateOdometry(gyro.getRotation2d());
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
    }
}