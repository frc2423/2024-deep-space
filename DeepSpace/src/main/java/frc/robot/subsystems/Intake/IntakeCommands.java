package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class IntakeCommands {
    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }
    public Command intakeIn() {
        var command = Commands.run(() -> intake.intake(0.5));
        command.setName("Intake In");
        return command;
}
    public Command intakeOut() {
        var command = Commands.run(() -> intake.outtake(-0.5));
        command.setName("Intake Out");
        return command;
}
    public Command intakeStop() {
        var command = Commands.run(() -> intake.stop());
        command.setName("Intake Stop");
        return command;
}
}