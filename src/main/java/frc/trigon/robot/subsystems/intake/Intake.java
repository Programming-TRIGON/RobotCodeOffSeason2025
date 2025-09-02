package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor
            intakeMotor = IntakeConstants.INTAKE_MOTOR,
            angleMotor = IntakeConstants.ANGLE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private IntakeConstants.IntakeState targetState = IntakeConstants.IntakeState.REST;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void sysIDDrive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("CoralAngleMotor")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return IntakeConstants.ANGLE_SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        intakeMotor.update();
        angleMotor.update();
        IntakeConstants.FORWARD_LIMIT_SENSOR.updateSensor();
        IntakeConstants.REVERSE_LIMIT_SENSOR.updateSensor();
        IntakeConstants.DISTANCE_SENSOR.updateSensor();
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.INTAKE_MECHANISM.update(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        IntakeConstants.ANGLE_MECHANISM.update(
                Rotation2d.fromRotations(getCurrentAngle().getRotations()),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        Transform3d transform = new Transform3d(new Translation3d(), new Rotation3d(0, getCurrentAngle().getRadians(), 0));
        Logger.recordOutput("Poses/Components/IntakePose", IntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(transform));
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public IntakeConstants.IntakeState getTargetState() {
        return targetState;
    }

    public boolean atState(IntakeConstants.IntakeState targetState) {
        return targetState == this.targetState && atTargetAngle();
    }

    @AutoLogOutput(key = "CoralIntake/AtTargetAngle")
    public boolean atTargetAngle() {
        final double angleDifferenceFromTargetAngleDegrees = Math.abs(getCurrentAngle().minus(targetState.targetAngle).getDegrees());
        return angleDifferenceFromTargetAngleDegrees < IntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetAngle, targetState.targetVoltage);
    }

    void setTargetState(Rotation2d targetAngle, double targetVoltage) {
        setTargetVoltage(targetVoltage);
        setTargetAngle(targetAngle);
    }

    private void setTargetVoltage(double voltage) {
        intakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.POSITION));
    }
}