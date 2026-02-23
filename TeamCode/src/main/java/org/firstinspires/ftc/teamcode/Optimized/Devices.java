package org.firstinspires.ftc.teamcode.Optimized;

import static org.firstinspires.ftc.teamcode.Optimized.Utilities.degreesToTicks;
import static org.firstinspires.ftc.teamcode.Optimized.Utilities.radiansToTicks;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Optimized.DataTypes.PIDFFCoefficients;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import kotlin.Unit;

public class Devices {
    public static class DcMotorEx {
        /** The motor object **/
        private com.qualcomm.robotcore.hardware.DcMotorEx motor;
        /** Currently running OpMode **/
        private final OpMode opMode;
        /** Motor's name in configuration **/
        private final String deviceName;
        /** Rotation direction of the motor **/
        public Direction direction;
        /** Motor encoder resolution in ticks/rotation **/
        public double encoderResolution;
        /** Offset of the motor's zero position from the actual zero position in encoder ticks **/
        public int offset;
        /** Zero power behavior of the motor **/
        public ZeroPowerBehavior zeroPowerBehavior;
        /** Current PIDF Coefficients for the motor**/
        public PIDFFCoefficients pidffCoefficients = new PIDFFCoefficients();
        /** PIDF coefficients of the motor during the last code loop,
         *  used for detecting coefficient changes to rebuild the PIDFs
         **/
        private PIDFFCoefficients old_pidfCoefficients = null;
        /** Position PIDF control system **/
        public ControlSystem posController = null;
        /** Velocity PIDF control system **/
        public ControlSystem velController = null;
        /** Target position of the motor in encoder ticks **/
        public int targetPos = 0;
        /** Target velocity of the motor in encoder ticks per second **/
        public double targetVel = 0;
        /** Current Run Mode of the motor **/
        public RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;


        public enum RunMode {
            RUN_TO_POSITION {
                @Override
                public boolean isPIDMode() {return false;}
            },
            RUN_USING_ENCODER {
                @Override
                public boolean isPIDMode() {return false;}
            },
            RUN_WITHOUT_ENCODER {
                @Override
                public boolean isPIDMode() {return false;}
            },
            RUN_USING_POS_PID {
                @Override
                public boolean isPIDMode() {return true;}
            },
            RUN_USING_VEL_PID {
                @Override
                public boolean isPIDMode() {return true;}
            };

            public abstract boolean isPIDMode();
        }


        public DcMotorEx (String deviceName){
            opMode = OpModeManager.getActiveOpMode();
            this.deviceName = deviceName;
        }
        public DcMotorEx (String deviceName, Direction direction){
            this(deviceName);
            this.direction = direction;
        }
        public DcMotorEx (String deviceName, Direction direction, double encoderResolution){
            this(deviceName, direction);
            this.encoderResolution = encoderResolution;
        }
        public DcMotorEx (String deviceName, Direction direction, double encoderResolution, int offset){
            this(deviceName, direction, encoderResolution);
            this.offset = offset;
        }
        public DcMotorEx (String deviceName, Direction direction, double encoderResolution, int offset, ZeroPowerBehavior zeroPowerBehavior){
            this(deviceName, direction, encoderResolution, offset);
            this.zeroPowerBehavior = zeroPowerBehavior;
        }
        public DcMotorEx (String deviceName, Direction direction, double encoderResolution, int offset, ZeroPowerBehavior zeroPowerBehavior, RunMode runMode){
            this(deviceName, direction, encoderResolution, offset, zeroPowerBehavior);
            this.runMode = runMode;
        }

        /** Must be called during initialization of the OpMode **/
        public void init(){
            motor = opMode.hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, deviceName);
            motor.setDirection(direction);
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            buildPIDFs();
        }

        // TODO add custom run mode to motor to switch between velocity and position PIDs
        public void update(){
            if (old_pidfCoefficients != pidffCoefficients){
                buildPIDFs();
                old_pidfCoefficients = pidffCoefficients;
            }

            if (runMode.isPIDMode() && motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }else{
                switch (runMode){
                    case RUN_TO_POSITION:
                        motor.setTargetPosition(targetPos);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;
                    case RUN_USING_ENCODER:
                        if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                        break;
                    case RUN_WITHOUT_ENCODER:
                        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }
                        break;
                }
            }

            KineticState currentKineticState = new KineticState(getAngleAbsolute(), getVelocity());
            if (runMode == RunMode.RUN_USING_POS_PID){
                if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                posController.setGoal(new KineticState(targetPos, targetVel));
            }
        }

        private void buildPIDFs(){
            PIDCoefficients c = pidffCoefficients.getPIDCoefficients();
            posController = ControlSystem.builder()
                    .posPid(pidffCoefficients.getPIDCoefficients())
                    .build();
            velController = ControlSystem.builder()
                    .velPid(pidffCoefficients.getPIDCoefficients())
                    .basicFF(pidffCoefficients.getFFCoefficients())
                    .build();
        }


        /**
         * Sets the target position of the motor
         * @param angle The target angle in the specified units.
         * @param velocity The target velocity in units/sec.
         * @param units The units to be used for the angle & velocity.
         */
        public void setAngle(double angle, double velocity, @NonNull DataTypes.AngleUnit units){
            switch (units) {
                case TICKS:
                    targetPos = (int) angle;
                    targetVel = (int) velocity;
                    break;
                case DEGREES:
                    targetPos = (int) degreesToTicks(angle, encoderResolution);
                    targetVel = (int) degreesToTicks(velocity, encoderResolution);
                    break;
                case RADIANS:
                    targetPos = (int) radiansToTicks(angle, encoderResolution);
                    targetVel = (int) radiansToTicks(velocity, encoderResolution);
                    break;
            }
        }
        /**
         * Sets the target position of the motor
         * @param angle The target angle in the specified units.
         * @param units The units to be used for the angle.
         */
        public void setAngle(double angle, DataTypes.AngleUnit units){
            setAngle(angle, 0, units);
        }
        /**
         * Sets the target position of the motor
         * @param angle The target angle in encoder ticks.
         */
        public void setAngle(double angle){
            setAngle(angle, 0, DataTypes.AngleUnit.TICKS);
        }
        /**
         * Sets the target position of the motor
         * @param angle The target angle in encoder ticks.
         * @param velocity The target velocity in encoder ticks/sec.
         */
        public void setAngle(double angle, double velocity){
            setAngle(angle, velocity, DataTypes.AngleUnit.TICKS);
        }


        /**
         * @param power Ranges between -1 and 1. The power to apply to the motor.
         */
        public void setPower(double power){
            motor.setPower(power);
        }


        /**
         * @param units The units to be used in the returned angle.
         * @return The angle of the motor in the specified units.
         */
        public double getAngleAbsolute(@NonNull DataTypes.AngleUnit units){
            double ticks = motor.getCurrentPosition() + offset;
            switch (units) {
                case DEGREES:
                    return (ticks)/encoderResolution*360;
                case RADIANS:
                    return (ticks)/encoderResolution*(2*Math.PI);
            }
            return ticks;
        }
        /**
         * @return The angle of the motor in encoder ticks.
         */
        public double getAngleAbsolute(){
            return getAngleAbsolute(DataTypes.AngleUnit.TICKS);
        }


        /**
         * Gets the current velocity of the motor in the specified units.
         * @param units The units of the returned value.
         * @return The current velocity of the motor.
         */
        public double getVelocity(DataTypes.AngleUnit units){
            double ticks = motor.getVelocity();
            switch (units){
                case DEGREES:
                    return Utilities.ticksToDegrees(ticks, encoderResolution);
                case RADIANS:
                    return Utilities.ticksToRadians(ticks, encoderResolution);
            }
            return ticks;
        }
        /**
         * @return The current velocity of the motor in encoder ticks per second.
         */
        public double getVelocity(){
            return getVelocity(DataTypes.AngleUnit.TICKS);
        }
    }
}