import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ElevatorSubsystem extends SubsystemBase {
    ProfiledPIDController elevator_PID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));// noice
    ProfiledPIDController elevator_PID_Manual = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));// for the goUp and goDown for manual (slower maxVel)
    private double elevatorCurrentPose = 0;
    private double setpoint = 0;
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0, 0, 0, 0);
    private CANSparkFlex motor1 = new CANSparkFlex(24 , MotorType.kBrushless);
    private CANSparkFlex motor2 = new CANSparkFlex(26 , MotorType.kBrushless);
    private double highestPoint = 5.4;
    private double lowestPoint = 0;



    public ElevatorSubsystem() {
        
    }

    @Override
    public void periodic() {
        double calculatedPID = calculatePid(setpoint);
        motor1.set(calculatedPID);
        motor2.set(-calculatedPID); //ONE OF THEM IS NEGITIVE

      

    }

    private double calculatePid(double position) {
        // updatePivotAngle();
        double pid = elevator_PID.calculate(elevatorCurrentPose, position);
        var setpoint = elevator_PID.getSetpoint();
        double feedforward = m_feedforward.calculate(0, 0);
        // return feedforward + pid;
        return (feedforward + pid) / RobotController.getBatteryVoltage();
    }

    public void goUp() { // for manual control
        setpoint = highestPoint;


    }

    public void goDown() { // for manual control
        setpoint = lowestPoint;
    }

    public void stopElevator() {
        setpoint = elevatorCurrentPose;
    }

    public double getElevatorVelocity() {
        return motor1.getEncoder().getVelocity();
    }

}
