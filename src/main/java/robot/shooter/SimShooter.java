package robot.shooter;


import com.google.flatbuffers.Constants;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class SimShooter implements ShooterIO {
   private final FlywheelSim shooter;
        public SimShooter() {
            shooter = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), ShooterConstants.MOI, ShooterConstants.GEARING), DCMotor.getNeoVortex(2));
        }


        public void setVoltage(double voltage) {
            shooter.setInputVoltage(voltage);
            shooter.update(Constants.SIZE_PREFIX_LENGTH);
        }
        public double getVelocity() {
            return shooter.getAngularVelocityRadPerSec();
    }
}


