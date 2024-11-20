package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.devices.NeoMotor;

public class IntakeSubsystem {    
    NeoMotor intakeMotor = new NeoMotor(23,true);
    Timer intakeTime = new Timer();
    public IntakeSubsystem() {
        
    }
    public void intake(){
        intakeMotor.setPercent(0.4);
    }

    public void outtake() {
        intakeMotor.setPercent(0.4);
    }

    public Command intakeIn() {
        var command = Commands.run(() -> intake());
        command.setName("Intake");
        return command;    
    }

    public Command intakeOut() {
        var command = Commands.run(() -> outtake());
        command.setName("Outtake");
        return command;    
    }
}
