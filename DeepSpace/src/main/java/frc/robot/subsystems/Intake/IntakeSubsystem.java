package frc.robot.subsystems.Intake;
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

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkFlex motor = new CANSparkFlex(26 , MotorType.kBrushless);



    public IntakeSubsystem() {
        
    }

    public void intake(double speed){
        motor.set(speed);
    }

    public void outtake(double speed){
        motor.set(speed);
    }

    public void stop(){
        motor.set(0);
    }
}