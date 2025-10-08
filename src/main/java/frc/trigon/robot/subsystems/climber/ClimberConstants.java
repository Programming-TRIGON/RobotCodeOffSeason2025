package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.hardware.RobotHardwareStats;
import lib.hardware.misc.servo.Servo;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import lib.hardware.simulation.SimpleMotorSimulation;
import lib.utilities.mechanisms.SingleJointedArmMechanism2d;

public class ClimberConstants {
    private static final int
            MOTOR_ID = 18,
            RIGHT_SERVO_CHANNEL = 0,
            LEFT_SERVO_CHANNEL = 1;
    private static final String
            MOTOR_NAME = "ClimberMotor",
            RIGHT_SERVO_NAME = "ClimberRightServo",
            LEFT_SERVO_NAME = "ClimberLeftServo";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final Servo
            RIGHT_SERVO = new Servo(RIGHT_SERVO_CHANNEL, RIGHT_SERVO_NAME),
            LEFT_SERVO = new Servo(LEFT_SERVO_CHANNEL, LEFT_SERVO_NAME);

    static final int
            GROUNDED_PID_SLOT = 0,
            ON_CAGE_PID_SLOT = 1;
    private static final double GEAR_RATIO = 37.5;
//    private static final int
//            SERVO_PULSE_WIDTH_MICROSECONDS = 20000,
//            SERVO_MAXIMUM_DEADBAND_RANGE_MICROSECONDS = 0,
//            SERVO_CENTER_PULSE_WIDTH_MICROSECONDS = 1500,
//            SERVO_MINIMUM_DEADBAND_RANGE_MICROSECONDS = 1000,
//            SERVO_MAXIMUM_PULSE_WIDTH_MICROSECONDS = 2000;

    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor
            GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation MOTOR_SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Second),
            Units.Volts.of(5),
            null
    );

    static final Pose3d CLIMBER_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );
    static final double CLIMBER_VISUALIZATION_POSITION_SCALAR = 0.1;
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "ClimberMechanism",
            Color.kGreen
    );

    static final double MAXIMUM_MANUAL_CONTROL_VOLTAGE = 4;
    static final double CLIMBER_TOLERANCE_ROTATIONS = 0.02;

    static {
        configureMotor();
        configureServos();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 22.373 : 70;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.33014 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.016057 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 4.3932 : 2.2;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0 : -0.2001953125;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.074561 : 0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.Slot1.kP = RobotHardwareStats.isSimulation() ? 22.373 : 80;
        config.Slot1.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kD = RobotHardwareStats.isSimulation() ? 0.33014 : 0;
        config.Slot1.kS = RobotHardwareStats.isSimulation() ? 0.016057 : 0;
        config.Slot1.kV = RobotHardwareStats.isSimulation() ? 4.3932 : 2.2;
        config.Slot1.kG = RobotHardwareStats.isSimulation() ? 0 : -1;
        config.Slot1.kA = RobotHardwareStats.isSimulation() ? 0.074561 : 0;
        config.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 2 : 1.5;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 10 : 3;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(MOTOR_SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    private static void configureServos() {
//        RIGHT_SERVO.setPWMBoundaries(
//                SERVO_PULSE_WIDTH_MICROSECONDS,
//                SERVO_MAXIMUM_DEADBAND_RANGE_MICROSECONDS,
//                SERVO_CENTER_PULSE_WIDTH_MICROSECONDS,
//                SERVO_MINIMUM_DEADBAND_RANGE_MICROSECONDS,
//                SERVO_MAXIMUM_PULSE_WIDTH_MICROSECONDS
//        );
//        LEFT_SERVO.setPWMBoundaries(
//                SERVO_PULSE_WIDTH_MICROSECONDS,
//                SERVO_MAXIMUM_DEADBAND_RANGE_MICROSECONDS,
//                SERVO_CENTER_PULSE_WIDTH_MICROSECONDS,
//                SERVO_MINIMUM_DEADBAND_RANGE_MICROSECONDS,
//                SERVO_MAXIMUM_PULSE_WIDTH_MICROSECONDS
//        );
    }

    public enum ClimberState {
        REST(0, 0, false),
        BREAK_ZIP_TIE(-0.32, 0, false),
        PREPARE_FOR_CLIMB(0, 1, false),
        CLIMB(-1.6, 0, true);

        public final double targetPositionRotations;
        public final double targetServoPower;
        public final boolean isAffectedByRobotWeight;

        ClimberState(double targetPositionRotations, double targetServoPower, boolean isAffectedByRobotWeight) {
            this.targetPositionRotations = targetPositionRotations;
            this.targetServoPower = targetServoPower;
            this.isAffectedByRobotWeight = isAffectedByRobotWeight;
        }
    }
}