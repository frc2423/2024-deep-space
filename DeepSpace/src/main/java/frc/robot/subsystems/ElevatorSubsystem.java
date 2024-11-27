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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorSubsystem extends SubsystemBase {
    ProfiledPIDController elevator_PID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));// noice
    ProfiledPIDController elevator_PID_Manual = new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(0, 0));// for the goUp and goDown for manual (slower maxVel)
    private double elevatorCurrentPose = 0;
    private double setpoint = 0;
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0, 0, 0, 0);
    private CANSparkFlex motor1 = new CANSparkFlex(24, MotorType.kBrushless);
    private CANSparkFlex motor2 = new CANSparkFlex(26, MotorType.kBrushless);
    private double highestPoint = 5.4;
    private double lowestPoint = 0;

    private ElevatorSim elevatorSim = new ElevatorSim();

    // the main mechanism object
    private Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
    private MechanismRoot2d root = mech.getRoot("bottom", 1.55, 0);

    //private final FlywheelSim elevatorSimMotor = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

    // MechanismLigament2d objects represent each "section"/"stage" of the
    // mechanism, and are based
    // off the root node or another ligament object
    MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", lowestPoint, 90));
    MechanismLigament2d bottom = elevator.append(
            new MechanismLigament2d("bottom", 5, 180, 6, new Color8Bit(Color.kBlanchedAlmond)));

    public ElevatorSubsystem() {

        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);
        //elevatorSimMotor.setInput(0);
    }

    @Override
    public void periodic() {
        double calculatedPID = calculatePid(setpoint);
        motor1.set(calculatedPID);
        motor2.set(-calculatedPID); // ONE OF THEM IS NEGITIVE, NICE O

        if (Robot.isSimulation()) {
            elevatorSim.periodic();
            bottom.setLength(elevatorSim.getHeight());
        }
    }

    private double calculatePid(double position) {
        // updatePivotAngle();
        double pid = elevator_PID.calculate(elevatorCurrentPose, position);
        var setpoint = elevator_PID.getSetpoint();
        double feedforward = m_feedforward.calculate(1, 2);
        // return feedforward + pid;
        return (feedforward + pid) / RobotController.getBatteryVoltage();
    }

    public Command goDown() { // for manual control, sick
        return runOnce(() -> {
            setpoint = lowestPoint;
        });
    }

    public Command goUp() { // for manual control, sick
        return runOnce(() -> {
            setpoint = highestPoint;
        });
    }

    public Command stopElevator() { // for manual control, sick
        return runOnce(() -> {
            setpoint = elevatorCurrentPose;
        });
    }

    public Command getElevatorVelocity() { // for manual control, sick
        return runOnce(() -> {
            motor1.getEncoder().getVelocity();
        });
    }

    public void resetElevatorPosition() {
        setpoint = 0;
    }

    public double getHeight() {
        return motor1.getAbsoluteEncoder().getPosition();
    }

    //public double getHeightSim() {

    //}
}
