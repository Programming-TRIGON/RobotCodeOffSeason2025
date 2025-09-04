package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.util.Color;
import lib.hardware.misc.leds.LEDStripAnimationSettings;

public class LEDConstants {
    public static LEDStripAnimationSettings.ColorFlowSettings COLLECTION_CONFIRMATION_ANIMATION_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(
            Color.kGreen,
            0.5,
            false
    );

    /**
     * Initializes LEDConstants. Needed to be called for the LED strips to be initialized before being used.
     */
    public static void init() {
    }
}