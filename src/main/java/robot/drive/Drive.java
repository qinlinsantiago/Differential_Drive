package robot.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Constants;
import robot.Ports;
import robot.Robot;
import robot.drive.DriveConstants.FF;
import robot.drive.DriveConstants.PID;

@Logged
public class Drive extends SubsystemBase {
    private final SparkMax leftLeader = new SparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
    private final DifferentialDriveOdometry odometry;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);
    private final PIDController leftPIDController =
        new PIDController(PID.kP, PID.kI, PID.kD);
    private final PIDController rightPIDController =
        new PIDController(PID.kP, PID.kI, PID.kD);
    private final DifferentialDrivetrainSim driveSim;
    @Logged
    private final Field2d field2d = new Field2d();
    
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
    
        driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getMiniCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.DRIVE_MASS,
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            DriveConstants.STD_DEVS);
    }
    
    private void drive(double leftSpeed, double rightSpeed) {
        leftLeader.set(leftSpeed);
        rightLeader.set(rightSpeed);
      
    final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
    final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;

    final double leftFeedforward = feedforward.calculate(realLeftSpeed);
    final double rightFeedforward = feedforward.calculate(realRightSpeed);

    final double leftPID = 
      leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
    final double rightPID = 
      rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);
      
      double leftVoltage = leftPID + leftFeedforward;
      double rightVoltage = rightPID + rightFeedforward;

      leftLeader.setVoltage(leftVoltage);
      rightLeader.setVoltage(rightVoltage);
      driveSim.setInputs(leftVoltage, rightVoltage);
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
        updateOdometry(Robot.isReal() ? gyro.getRotation2d() :  
                      driveSim.getHeading());
        field2d.setRobotPose(pose());
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
    }
    @Override
    public void simulationPeriodic() {
    // sim.update() tells the simulation how much time has passed
        driveSim.update(Constants.PERIOD.in(Seconds));
        leftEncoder.setPosition(driveSim.getLeftPositionMeters());
        rightEncoder.setPosition(driveSim.getRightPositionMeters());
    }
}