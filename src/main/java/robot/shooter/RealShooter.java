package robot.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RealShooter implements ShooterIO {

    private final SparkMax leader;
    private final SparkMax follower;
    private final RelativeEncoder encoder;

    public RealShooter() {

        leader = new SparkMax(
            ShooterConstants.LEADER,
            MotorType.kBrushless
        );

        follower = new SparkMax(
            ShooterConstants.FOLLOWER,
            MotorType.kBrushless
        );

        encoder = leader.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .smartCurrentLimit((int) ShooterConstants.CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);

        leader.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        follower.configure(
            config.inverted(true).follow(leader),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }


    @Override
    public void setVoltage(double voltage) {
        leader.setVoltage(voltage);
    }


    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
}