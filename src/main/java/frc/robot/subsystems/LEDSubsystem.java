package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LEDSubsystem {
    private static final int NUM_LEDS = 60; // adjust as needed
    private final AddressableLED ledLeft;
    private final AddressableLED ledRight;
    private final AddressableLEDBuffer ledBuffer;

    // Define indices for the forward portion of each LED strip (adjust as needed)
    private static final int LEFT_FORWARD_START = 0;
    private static final int LEFT_FORWARD_END = 20;
    private static final int RIGHT_FORWARD_START = 0;
    private static final int RIGHT_FORWARD_END = 20;
    private static final int LEFT_BACK_START = 20;
    private static final int LEFT_BACK_END = 60;
    private static final int RIGHT_BACK_START = 20;
    private static final int RIGHT_BACK_END = 60;
    public LEDSubsystem() {
        ledLeft = new AddressableLED(0); // PWM port 0; change if necessary
        ledRight = new AddressableLED(1); // PWM port 1; change if necessary
        ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        ledLeft.setLength(ledBuffer.getLength());
        ledRight.setLength(ledBuffer.getLength());
        ledLeft.start();
        ledRight.start();
    }
    
    // Set LED color (R, G, B).
    // For an "RRGB" LED, assume both red channels use the same red value.
    public void setColorLeft(int red, int green, int blue) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        ledLeft.setData(ledBuffer);
    }
    public void setColorRight(int red, int green, int blue) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        ledRight.setData(ledBuffer);
    }
    
    public void turnOffLeft() {
        setColorLeft(0, 0, 0);
    }
    public void turnOffRight() {
        setColorRight(0, 0, 0);
    }
    
    // New method to update LED color based on error from target pose.
    public void updateLEDColor(Pose2d currentPose, Pose2d targetPose) {
        double error = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        int red, green, blue;
        if (error < 0.3) {         // close: green
            red = 0; green = 255; blue = 0;
        } else if (error < 1.0) {    // moderate: yellow
            red = 255; green = 255; blue = 0;
        } else {                   // far: red
            red = 255; green = 0; blue = 0;
        }
        setColorLeft(red, green, blue);
        setColorRight(red, green, blue);
    }
    
    // New helper: flashes the forward segment on the left LED strip.
    public void flashLeftForwardSegment(boolean flashOn) {
        for (int i = LEFT_FORWARD_START; i < LEFT_FORWARD_END && i < ledBuffer.getLength(); i++) {
            // If flashing, set to white; otherwise, turn off.
            if (flashOn) {
                ledBuffer.setRGB(i, 255, 255, 255);
            } else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        ledLeft.setData(ledBuffer);
    }
    
    // New helper: flashes the forward segment on the right LED strip.
    public void flashRightForwardSegment(boolean flashOn) {
        for (int i = RIGHT_FORWARD_START; i < RIGHT_FORWARD_END && i < ledBuffer.getLength(); i++) {
            if (flashOn) {
                ledBuffer.setRGB(i, 255, 255, 255);
            } else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        ledRight.setData(ledBuffer);
    }
    
    // New method to flash directional cue.
    // Pass a negative value for left, positive for right. Zero disables flashing.
    public void flashDirectionalCue(int direction, boolean flashOn) {
        if (direction < 0) {
            flashLeftForwardSegment(flashOn);
        } else if (direction > 0) {
            flashRightForwardSegment(flashOn);
        }
    }
    public void xAxis(Pose2d currentPose, Pose2d targetPose){
        double xError = currentPose.getX() - targetPose.getX();
        if (xError > 0){
            for (int i = LEFT_BACK_START; i < LEFT_BACK_END; i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
            ledLeft.setData(ledBuffer);
        }
        else if (xError < 0){
            for (int i = RIGHT_BACK_START; i < RIGHT_BACK_END; i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
            ledRight.setData(ledBuffer);
        }
        else{
            for (int i = LEFT_BACK_START; i < LEFT_BACK_END; i++) {
                ledBuffer.setRGB(i, 0, 255, 0);
            }
            for (int i = RIGHT_BACK_START; i < RIGHT_BACK_END; i++) {
                ledBuffer.setRGB(i, 0, 255, 0);
            }
            ledLeft.setData(ledBuffer);
            ledRight.setData(ledBuffer);
        }
    }
    public void yAxis(Pose2d currentPose, Pose2d targetPose){
        double yError = currentPose.getY() - targetPose.getY();
        if (yError > 0){
            for (int i = LEFT_BACK_START; i < LEFT_BACK_END; i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
            ledLeft.setData(ledBuffer);
        }
        else if (yError < 0){
            for (int i = RIGHT_BACK_START; i < RIGHT_BACK_END; i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
            ledRight.setData(ledBuffer);
        }
        else{
            for (int i = LEFT_BACK_START; i < LEFT_BACK_END; i++) {
                ledBuffer.setRGB(i, 0, 255, 0);
            }
            for (int i = RIGHT_BACK_START; i < RIGHT_BACK_END; i++) {
                ledBuffer.setRGB(i, 0, 255, 0);
            }
            ledLeft.setData(ledBuffer);
            ledRight.setData(ledBuffer);
        }
    }
}
