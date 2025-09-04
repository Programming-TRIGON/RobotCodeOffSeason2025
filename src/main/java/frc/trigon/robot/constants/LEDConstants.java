package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.util.Color;
import lib.hardware.misc.leds.LEDStripAnimationSettings;

public class LEDConstants {
    //TODO: Implement LEDConstants
    public static final LEDStripAnimationSettings.BlinkSettings CLIMB_ANIMATION_SETTINGS = new LEDStripAnimationSettings.BlinkSettings(Color.kYellow, 0.1);//TODO: Add fade animation

    /**
     * Initializes LEDConstants. Needed to be called for the LED strips to be initialized before being used.
     */
    public static void init() {
    }
}