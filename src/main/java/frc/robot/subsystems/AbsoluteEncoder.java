// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.loopController.Looper;
import frc.loopController.Loop;

/** Add your docs here. */
public class AbsoluteEncoder implements com.revrobotics.AbsoluteEncoder, Loop {
    DutyCycleEncoder m_encoder;
    boolean inverted = false;
    double velocityConversionFactor = 1.0;
    int depth;
    private final Timer m_timer = new Timer();
    private Looper looper = new Looper(0.01); // Update velocity @ 100hz
    private double lastPosition;
    private double lastTime;
    private double currentVelocity;

    AbsoluteEncoder(int channel) {
        m_encoder = new DutyCycleEncoder(channel);
        m_timer.reset();
        m_timer.start();

        lastPosition = m_encoder.getDistance();
        lastTime = m_timer.get();

        looper.register(this);
        looper.start();
    }

    public void reset() {
        m_encoder.reset();
        m_timer.reset();
        
        lastPosition = m_encoder.getDistance();
        lastTime = m_timer.get();

        // Needed???
        currentVelocity = 0;
  }
    
    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
        // Get current Position and Time
        double currentPosition = m_encoder.getDistance();
        double currentTime = m_timer.get();
        
        // Calculate current Velocity
        currentVelocity = (currentPosition - lastPosition)
          / (currentTime - lastTime);
        
        // Set previous Position and Time to current Position and Time
        lastPosition = currentPosition;
        lastTime = currentTime;
    }

    @Override
    public void onStop() {
    }

    /**
        * Get the position of the motor. This returns the native units of 'rotations' by default, and can
        * be changed by a scale factor using setPositionConversionFactor().
        *
        * @return Number of rotations of the motor
        */
    public double getPosition() {
        double output = m_encoder.getAbsolutePosition();
        if(inverted) output = -output;
        return output;
    }

    /**
        * Get the velocity of the motor. This returns the native units of 'rotations per second' by
        * default, and can be changed by a scale factor using setVelocityConversionFactor().
        *
        * @return Number of rotations per second of the motor
        */
    public double getVelocity() {
        double output = currentVelocity*velocityConversionFactor;
        if(inverted) output = -output;
        return output;
    }

    /**
        * Set the conversion factor for position of the encoder. Multiplied by the native output units to
        * give you position
        *
        * @param factor The conversion factor to multiply the native units by
        * @return {@link REVLibError#kOk} if successful
        */
    public REVLibError setPositionConversionFactor(double factor) {
        try {
            m_encoder.setDistancePerRotation(factor);
        } catch (Error e) {
            e.printStackTrace();
            return REVLibError.kUnknown;
        }
        return REVLibError.kOk;
    }

    /**
         * Get the conversion factor for position of the encoder. Multiplied by the native output units to
         * give you position
         *
         * @return The conversion factor for position
         */
    public double getPositionConversionFactor() {
        return m_encoder.getDistancePerRotation();
    }
    
    /**
         * Set the conversion factor for velocity of the encoder. Multiplied by the native output units to
         * give you velocity
         *
         * @param factor The conversion factor to multiply the native units by
         * @return {@link REVLibError#kOk} if successful
         */
    public REVLibError setVelocityConversionFactor(double factor) {
        velocityConversionFactor = factor;
        return REVLibError.kOk;
    }

    /**
        * Get the conversion factor for velocity of the encoder. Multiplied by the native output units to
        * give you velocity
        *
        * @return The conversion factor for velocity
        */
    public double getVelocityConversionFactor() {
        return velocityConversionFactor;
    }

    /**
        * Set the phase of the AbsoluteEncoder so that it is set to be in phase with the motor itself
        *
        * @param inverted The phase of the encoder
        * @return {@link REVLibError#kOk} if successful
        */
    public REVLibError setInverted(boolean inverted) {
        this.inverted = inverted;
        return REVLibError.kOk;
    }

    /**
        * Get the phase of the AbsoluteEncoder
        *
        * @return The phase of the encoder
        */
    public boolean getInverted() {
        return this.inverted;
    }

    /**
        * Set the average sampling depth for an absolute encoder. This is a bit size and should be either
        * 1, 2, 4, 8, 16, 32, 64, or 128
        *
        * @param depth The average sampling depth of 1, 2, 4, 8, 16, 32, 64, or 128
        * @return {@link REVLibError#kOk} if successful
        */
    public REVLibError setAverageDepth(int depth) {
        this.depth = depth;
        return REVLibError.kOk;
    }

    /**
        * Get the average sampling depth for an absolute encoder
        *
        * @return The average sampling depth
        */
    public int getAverageDepth() {
        return this.depth;
    }

    /**
        * Sets the zero offset of an absolute encoder (the position that is reported as zero).
        *
        * <p>The zero offset is specified as the reported position of the encoder in the desired zero
        * position, if the zero offset was set to 0. It is influenced by the absolute encoder's position
        * conversion factor, and whether it is inverted.
        *
        * <p>Always call SetPositionConversionFactor() and SetInverted() before calling this function.
        *
        * @param offset The zero offset with the position conversion factor applied
        * @return {@link REVLibError#kOk} if successful
        */
    public REVLibError setZeroOffset(double offset) {
        m_encoder.setPositionOffset(offset);
        return REVLibError.kOk;
    }

    /**
        * Gets the zero offset for an absolute encoder (the position that is reported as zero).
        *
        * <p>The zero offset is specified as the reported position of the encoder in the desired zero
        * position, if the zero offset was set to 0. It is influenced by the absolute encoder's position
        * conversion factor, and whether it is inverted.
        *
        * @return The zero offset of the absolute encoder with the position conversion factor applied
        */
    public double getZeroOffset() {
        return m_encoder.getPositionOffset();
    }
}