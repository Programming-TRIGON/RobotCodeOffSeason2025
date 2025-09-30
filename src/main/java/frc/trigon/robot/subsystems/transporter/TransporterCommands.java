package frc.trigon.robot.subsystems.transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.trigon.robot.RobotContainer;
import lib.commands.NetworkTablesCommand;

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

    public static Command getSetTargetStateWithPulsesCommand(TransporterConstants.TransporterState targetState) {
        return new SequentialCommandGroup(
                getSetTargetStateCommand(targetState).withTimeout(TransporterConstants.PULSE_VOLTAGE_APPLIED_TIME_SECONDS),
                new WaitCommand(TransporterConstants.PULSE_WAIT_TIME_SECONDS)
        ).repeatedly();
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