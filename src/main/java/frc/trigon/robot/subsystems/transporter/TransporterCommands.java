package frc.trigon.robot.subsystems.transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class TransporterCommands {
    public Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.TRANSPORTER::setTargetVoltage,
                false,
                Set.of(RobotContainer.TRANSPORTER),
                "Debugging/TransporterTargetVoltage"
        );
    }

    public Command getSetTargetStateCommand(TransporterConstants.TransporterState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetState(targetState),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }

    public Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetVoltage(targetVoltage),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }
}