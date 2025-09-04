package frc.trigon.robot.subsystems.transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class TransporterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.TRANSPORTER::setTargetState,
                false,
                Set.of(RobotContainer.TRANSPORTER),
                "Debugging/TransporterTargetRightMotorVoltage",
                "Debugging/TransporterTargetLeftMotorVoltage"
        );
    }

    public static Command getSetTargetStateCommand(TransporterConstants.TransporterState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetState(targetState),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }

    public static Command getSetTargetStateCommand(double rightMotorVoltage, double leftMotorVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetState(rightMotorVoltage, leftMotorVoltage),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }
}