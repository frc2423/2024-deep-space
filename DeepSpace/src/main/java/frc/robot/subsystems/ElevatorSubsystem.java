import com.revrobotics.CANSparkMax;

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
    private Translation2d elevatorPose = new Translation2d(0, 0);
    private Translation2d setpoint = new Translation2d(0, 0);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0, 0, 0, 0);
    private CANSparkMax motor;

    public ElevatorSubsystem() {

    }

    private double calculatePid(Translation2d position) {
        // updatePivotAngle();
        double pid = elevator_PID.calculate(elevatorPose.getY(), position.getY());
        var setpoint = elevator_PID.getSetpoint();
        double feedforward = m_feedforward.calculate(0, 0);
        // return feedforward + pid;
        return (feedforward + pid) / RobotController.getBatteryVoltage();
    }

    public void goUp() { // for manual control

    }

    public void goDown() { // for manual control

    }

    public void stopElevator() {

    }

    public double getElevatorVelocity() {
        return motor.getEncoder().getVelocity();
    }

}
