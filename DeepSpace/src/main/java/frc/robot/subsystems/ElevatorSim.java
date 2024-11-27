package frc.robot.subsystems;
import java.util.Date;
public class ElevatorSim {
    double height = 0;
    double voltage = 0;
    Date updateTime = new Date();

    public ElevatorSim() {
    }

    public double getHeight() {
        return height;
    }
    public void setVoltage(double incomingVoltage) {
        voltage = incomingVoltage;
    }

    public void periodic() {
        Date timePeriodic = new Date();
        double deltaT = timePeriodic.getTime() - updateTime.getTime();
        if (voltage > 0) {

        }
        
        else if (voltage == 0) {
            height = (0.5*9.8)*Math.pow(deltaT,2) ;
        }

        else {
            System.out.println("WRONG >:(");
        }
    }
}
