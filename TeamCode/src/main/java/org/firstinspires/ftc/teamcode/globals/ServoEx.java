package org.firstinspires.ftc.teamcode.globals;

import static com.qualcomm.robotcore.util.Range.scale;

import com.qualcomm.robotcore.hardware.Servo;


/**
 * A class that provides a convenient way to control servos in terms of degrees.
 * This class allows for calibration of a servo to reach any desired angle, from negative infinity to infinity.
 * Definitions:
 * <ul>
 *  <li> Range: Commonly used ranges in the servo world are 180˚, 270˚, 360˚ etc. The range depends on the servo and restrictions from the control hub.</li>
 *  <li>Calibration: The servo can be calibrated through the use of an angle of calibration and a corresponding servo position:
 * </li>
 * <ul>
 * <li>
 *     Angle Of Calibration: The angle defined by the user the specified servo position.
 *                           This could be used to define x angle as y servo position
 * </li>
 * <li>
 *     Servo Position At Calibration: The servoPosition that corresponds to the calibration angle
 * </li>
 * <li> To determine the correct calibration values for a specific servo setup, use the ServoCalibration example program located in the opmodes/examples/ directory. This tool helps to interactively find the angle and corresponding position value through manual testing.</li> </li>
 * </ul>
 */
public class ServoEx {
    private final Servo servo;
    private final double RANGE;
    private final double MINIMUM_ALLOWED_ANGLE;
    private final double MAXIMUM_ALLOWED_ANGLE;

    private double angle;

    /**
     * Constructs a ServoEx object for a servo that requires calibration.
     * This constructor allows the servo to be calibrated so that a specified
     * servo position corresponds to a specified angle.
     *
     * @param servo   - The servo to be controlled.
     * @param range   - The total range of the servo (Typically 180°, 270°, etc.).
     * @param reverse - The direction of the servo, with counterclockwise meaning
     *                decreasing servoPos with increasing angle
     *                <br>
     *                <br>
     */
    public ServoEx(Servo servo, double range, double servoPosAtCalibration, double angleAtCalibration, boolean reverse) {
        this.servo = servo;

        if (reverse) {
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }

        if (range < 0) {
            this.RANGE = 180;
        } else {
            this.RANGE = range;
        }


        if (!RobotUtils.isWithinRange(servoPosAtCalibration, 0, 1)) {
            MINIMUM_ALLOWED_ANGLE = 0;
            MAXIMUM_ALLOWED_ANGLE = this.RANGE;
            return;
        }

        MINIMUM_ALLOWED_ANGLE = angleAtCalibration - servoPosAtCalibration * this.RANGE;
        MAXIMUM_ALLOWED_ANGLE = angleAtCalibration + (1 - servoPosAtCalibration) * this.RANGE;
    }


    /**
     * Constructs a ServoEx object for a servo that does not require calibration.
     *
     * @param servo - The servo to be controlled.
     * @param range - The total number of degree positions the servo can move (e.g., 180°, 270°).
     */
    public ServoEx(Servo servo, double range) {
        this(servo, range, 0, 0, false);
    }


    /**
     * Moves the servo to a specific desired angle.
     * The desired angle will be mapped to a servo position
     *
     * @param degrees - The desired angle to move the servo to.
     *                If the desired angle causes the servo to exceed its minimum or maximum positions (namely 0 and 1),
     *                the method will return false.
     */
    public boolean setAngle(double degrees) {
        if (degrees >= MINIMUM_ALLOWED_ANGLE && degrees <= MAXIMUM_ALLOWED_ANGLE) {
            double targetServoPosition = scale(degrees, MINIMUM_ALLOWED_ANGLE, MAXIMUM_ALLOWED_ANGLE, 0, 1);
            servo.setPosition(targetServoPosition);
            this.angle = degrees;
            return true;
        }
        return false;
    }


    /**
     * Returns the current angle of the servo (the last angle the servo moved to)
     */
    public double getAngle() {
        return angle;
    }


    /**
     * Returns the current position of the servo (the last position the servo moved to)
     */
    public double getPosition() {
        return servo.getPosition();
    }

    public double getMaximumAllowedAngle() {
        return MAXIMUM_ALLOWED_ANGLE;
    }

    public double getMinimumAllowedAngle() {
        return MINIMUM_ALLOWED_ANGLE;
    }
}
