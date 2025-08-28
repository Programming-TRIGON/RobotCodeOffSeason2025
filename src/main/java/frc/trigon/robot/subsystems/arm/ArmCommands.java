package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.trigon.robot.RobotContainer;

public class ArmCommands {
    public static Command setTargetPositionCommand(ArmConstants.ArmState targetState) {
        return new FunctionalCommand(
                () -> RobotContainer.ARM.setTargetState(targetState),
                () -> RobotContainer.ARM.setTargetState(targetState),
                (interrupted) -> RobotContainer.ARM.setTargetState(ArmConstants.ArmState.REST),
                () -> false,
                RobotContainer.ARM
        );
    }
}
