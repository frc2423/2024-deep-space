package frc.robot.subsystems;
import java.util.Date;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSim{
    double initialHeight = 100;
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
        if (voltage > 0) {
            //vrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
            height+=1;
        }
        
        else if (voltage == 0) {
            Date timePeriodic = new Date();
            double deltaT = timePeriodic.getTime() - updateTime.getTime();
            if (height >= 0) {
                height = initialHeight -(0.5*9.8)*Math.pow(deltaT/1000,2);
                System.out.println(height);
                //System.out.println(deltaT / 1000);
            }
            //height = -(0.5*9.8)*Math.pow(deltaT/1000,2);
        }

        else {
            System.out.println("WRONG >:(");
        }
    }
}
