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
            armMasterMotor = ArmConstants.ARM_MASTER_MOTOR,
            endEffectorMotor = ArmConstants.END_EFFECTOR_MOTOR;
    private final CANcoderEncoder encoder = ArmConstants.ENCODER;
    private final VoltageOut armVoltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final VoltageOut endEffectorVoltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
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
        armMasterMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        armMasterMotor.stopMotor();
        endEffectorMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .angularPosition(Units.Rotations.of(encoder.getSignal(CANcoderSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(armMasterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(armMasterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        Logger.recordOutput("Poses/Components/ArmPose", calculateComponentPose());
        ArmConstants.ARM_MECHANISM.update(
                Rotation2d.fromRotations(getAngle().getRotations() + ArmConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET),
                Rotation2d.fromRotations(armMasterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + ArmConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET)
        );
        ArmConstants.END_EFFECTOR_MECHANISM.update(
                endEffectorMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)
        );
    }

    @Override
    public void updatePeriodically() {
        armMasterMotor.update();
        endEffectorMotor.update();
        encoder.update();
        Logger.recordOutput("Arm/CurrentPositionMeters", getAngle());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        armMasterMotor.setControl(armVoltageRequest.withOutput(targetVoltage));
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
        setTargetAngle(targetAngle);
        setTargetVoltage(targetVoltage);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        armMasterMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private void setTargetVoltage(double targetVoltage) {
        endEffectorMotor.setControl(endEffectorVoltageRequest.withOutput(targetVoltage));
    }

    private Pose3d calculateComponentPose() {
        final Transform3d armTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, getAngle().getRadians(), 0)
        );
        return ArmConstants.ARM_VISUALIZATION_ORIGIN_POINT.transformBy(armTransform);
    }
}