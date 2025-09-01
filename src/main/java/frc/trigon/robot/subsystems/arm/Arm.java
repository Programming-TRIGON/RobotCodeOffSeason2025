package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Arm extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = ArmConstants.MASTER_MOTOR;
    private final CANcoderEncoder encoder = ArmConstants.ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ArmConstants.DEFAULT_MAXIMUM_VELOCITY,
            ArmConstants.DEFAULT_MAXIMUM_ACCELERATION,
            ArmConstants.DEFAULT_MAXIMUM_JERK
    );
    private ArmConstants.ArmState targetState = ArmConstants.ArmState.REST;

    public Arm() {
        setName("Arm");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ArmConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .angularPosition(Units.Degrees.of(encoder.getSignal(CANcoderSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        Logger.recordOutput("Poses/Components/ArmPose", calculateComponentPose());
        ArmConstants.MECHANISM.update(
                getAngle(),
                Rotation2d.fromRotations(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + ArmConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET)
        );
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        encoder.update();
        Logger.recordOutput("Arm/CurrentPositionMeters", getAngle());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public boolean atState(ArmConstants.ArmState targetState) {
        return this.targetState == targetState && atTargetState();
    }

    public boolean atTargetState() {
        final double currentToTargetStateDifferenceDegrees = Math.abs(targetState.targetAngle.minus(getAngle()).getDegrees());
        return currentToTargetStateDifferenceDegrees < ArmConstants.ANGLE_TOLERANCE.getDegrees();
    }

    void setTargetState(ArmConstants.ArmState targetState) {
        this.targetState = targetState;
        setTargetState(
                targetState.targetAngle,
                targetState.targetVoltage);
    }

    void setTargetState(Rotation2d targetAngle, double targetVoltage) {
        setTargetAngleWithSkibidyLogic(targetAngle);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }

    private void setTargetAngleWithSkibidyLogic(Rotation2d targetAngle) {
        if (Math.abs(getAngle().getDegrees() - targetAngle.getDegrees()) > 180) {
            if (getAngle().getDegrees() < targetAngle.getDegrees())
                setTargetAngle(Rotation2d.fromDegrees(getAngle().getDegrees() - (360 + getAngle().getDegrees()) - targetAngle.getDegrees()));
            else if (getAngle().getDegrees() > targetAngle.getDegrees())
                setTargetAngle(Rotation2d.fromDegrees(getAngle().getDegrees() + (360 + getAngle().getDegrees()) - targetAngle.getDegrees()));
            else
                System.out.println("something wrong is with the logic");
        }

    }

    private void setTargetAngle(Rotation2d targetAngle) {
        masterMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Pose3d calculateComponentPose() {
        final Transform3d armTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, getAngle().getRadians(), 0)
        );
        return ArmConstants.ARM_VISUALIZATION_ORIGIN_POINT.transformBy(armTransform);
    }
}
