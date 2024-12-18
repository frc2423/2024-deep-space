package frc.robot.subsystems.Intake;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import au.grapplerobotics.LaserCan;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkFlex motor = new CANSparkFlex(23 , MotorType.kBrushless);
    private LaserCan intakeDist = new LaserCan(26);
    private Servo servo = new Servo(0);
    

    public IntakeSubsystem() {
        
    }

    public void intake(double speed){
        motor.set(speed); 
    }

    public void outtake(double speed){
        motor.set(speed);
    }

    public void hatchDrop(){
        servo.setAngle(0);
    }

    public void hatchReset(){
        servo.setAngle(180);
    }

    public void stop(){
        motor.set(0);
    }

    public double distMm(){
        var dist = intakeDist.getMeasurement();
        if(dist == null){
            return 10000;
        } else{
            return dist.distance_mm;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("laserCan distance", () -> distMm(), null);
        // builder.addDoubleProperty("servo Angle", () -> servo.getAngle(), null);
    }
}