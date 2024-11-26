package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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

    private final MechanismLigament2d elevator;
    private final MechanismLigament2d bottom;



    public ElevatorSubsystem() {
    // the main mechanism object
        Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
        MechanismRoot2d root = mech.getRoot("bottom", 2, 2);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
        elevator = root.append(new MechanismLigament2d("elevator", lowestPoint, 90));
        bottom =
        elevator.append(
            new MechanismLigament2d("bottom", 0.5, 180, 6, new Color8Bit(Color.kBlanchedAlmond)));

    // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);
    }
        

    @Override
    public void periodic() {
        double calculatedPID = calculatePid(setpoint);
        motor1.set(calculatedPID);
        motor2.set(-calculatedPID); //ONE OF THEM IS NEGITIVE, NICE ONE

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

    public Command goDown() { // for manual control, sick
        return runOnce(()-> {
            setpoint = lowestPoint;
        } );
    }

    public void stopElevator() {
        setpoint = elevatorCurrentPose;
    }

    public double getElevatorVelocity() {
        return motor1.getEncoder().getVelocity();
    }

    public void resetElevatorPosition() {
        setpoint = 0;
    }
}
