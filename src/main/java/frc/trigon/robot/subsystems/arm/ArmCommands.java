package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;

public class ArmCommands {
    public static Command setTargetPositionCommand(ArmConstants.ArmState targetState){
        return new RunCommand(
                () -> RobotContainer.ARM.setTargetState(targetState),
                RobotContainer.ARM
        );
    }
}
