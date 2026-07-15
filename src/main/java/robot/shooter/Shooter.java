package robot.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Robot;

public final class Shooter extends SubsystemBase {

    private final ShooterIO hardware;

    private final PIDController pid =
        new PIDController(0,0,0);

    private final SimpleMotorFeedforward ff =
        new SimpleMotorFeedforward(0,0,0);


    private Shooter(ShooterIO hardware){
        this.hardware = hardware;
    }


    public static Shooter create(){

        return Robot.isReal()
            ? new Shooter(new RealShooter())
            : new Shooter(new SimShooter());
    }


    public static Shooter none(){
        return new Shooter(new NoShooter());
    }


    public double getVelocity(){
        return hardware.getVelocity();
    }


    public void update(double velocitySetpoint){

        double velocity = MathUtil.clamp(
            velocitySetpoint,
            -ShooterConstants.MAX_VELOCITY,
            ShooterConstants.MAX_VELOCITY
        );


        double ffVolts = ff.calculate(velocity);
        double pidVolts = pid.calculate(getVelocity(), velocity);


        hardware.setVoltage(
            MathUtil.clamp(
                pidVolts + ffVolts,
                -ShooterConstants.MAX_VOLTAGE,
                ShooterConstants.MAX_VOLTAGE
            )
        );
    }


    public Command runShooter(DoubleSupplier velocity){

        return run(
            () -> update(velocity.getAsDouble())
        );
    }


    public Command runShooter(double velocity){

        return runShooter(() -> velocity);
    }
}