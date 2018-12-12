
/**
 * The PX4Flow class provides an interface to PX4Flow optical flow camera capabilities
 * via I2C on the Android-based FTC robotics control system, where communications occur via the
 * "Core Device Interface Module" produced by Modern Robotics, inc.
 */

/*
Adapted from Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due
For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

// As described in the documentation
// http://pixhawk.org/modules/px4flow

package org.firstinspires.ftc.teamcode._Libs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;


class px4_frame
{
    public short frame_count;// counts created I2C frames
    public short pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame
    public short pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame
    public short flow_comp_m_x;// x velocity*1000 in meters / timestep
    public short flow_comp_m_y;// y velocity*1000 in meters / timestep
    public short quality;// Optical flow quality / confidence 0: bad, 255: maximum quality
    public short gyro_x_rate; //gyro x rate
    public short gyro_y_rate; //gyro y rate
    public short gyro_z_rate; //gyro z rate
    public byte  gyro_range; // gyro range
    public byte  sonar_timestamp;// timestep in milliseconds between I2C frames
    public short ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance

    public px4_frame(byte[] b) {
        ByteBuffer buf = ByteBuffer.wrap(b);
        buf.order(ByteOrder.LITTLE_ENDIAN);         // registers are read LSB, MSB
        frame_count = buf.getShort(0);
        pixel_flow_x_sum = buf.getShort(2);
        pixel_flow_y_sum = buf.getShort(4);
        byte q = buf.get(10);
        quality = (short)(q & 0xff);     // treat as unsigned byte
    }
}

class px4_integral_frame
{
    public short frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    public short pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    public short pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    public short gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000]
    public short gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000]
    public short gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000]
    public int   integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    public int   sonar_timestamp;// time since last sonar update [microseconds]
    public short ground_distance;// Ground distance in meters*1000 [meters*1000]
    public short gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    public short quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]

    public px4_integral_frame(byte[] b) {
        ByteBuffer buf = ByteBuffer.wrap(b);
        buf.order(ByteOrder.LITTLE_ENDIAN);         // registers are read LSB, MSB
        frame_count_since_last_readout = buf.getShort(0);
        pixel_flow_x_integral = buf.getShort(2);
        pixel_flow_y_integral = buf.getShort(4);
        gyro_x_rate_integral = buf.getShort(6);
        gyro_y_rate_integral = buf.getShort(8);
        gyro_z_rate_integral = buf.getShort(10);
        integration_timespan = buf.getInt(12);
        byte q = buf.get(24);
        quality = (short)(q & 0xff);     // treat as unsigned byte
    }
}

@I2cDeviceType() @DeviceProperties(name = "PX4Flow", description = "PX4Flow Camera", xmlTag = "PX4Flow")
public class PX4Flow extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // 7 Bit I2C Address of the Flow Module: Default 0x42 (user selectable bits 0,1,2) 
    final byte PX4FLOW_ADDRESS = 0x42;

    // timeout in milliseconds for PX4Flow read
    final byte PX4FLOW_TIMEOUT = 10;

    // If set to true, will print error messages on Serial
    final boolean PX4FLOW_DEBUG = true;

    private px4_frame frame;
    private px4_integral_frame iframe;

    private byte[] mCache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr mAddress = new I2cAddr(PX4FLOW_ADDRESS); //Default I2C address for PX4Flow (7-bit)

    final int mFrameReg = 0x00;     //Register to start reading for frame data
    final int mFrameLen = 22;       //Number of bytes to read
    final int mIntegralReg = 0x16;  //Register to start reading for integral data
    final int mIntegralLen = 26;    //Number of bytes to read (25 + pad)

    public I2cDevice mDevice;
    public I2cDeviceSynch mDeviceReader;

    public void init() {
        mDeviceReader = new I2cDeviceSynchImpl(mDevice, mAddress, false);
        mDeviceReader.engage();
    }

    private I2cDeviceSynch.ReadWindow mFrameReadWindow;
    private I2cDeviceSynch.ReadWindow mIntegralReadWindow;


    public PX4Flow(I2cDeviceSynch deviceSynch)
    {
        super(deviceSynch, true);

        // are these used?
        this.mFrameReadWindow = new I2cDeviceSynch.ReadWindow(mFrameReg, mFrameLen, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.mIntegralReadWindow = new I2cDeviceSynch.ReadWindow(mIntegralReg, mIntegralLen, I2cDeviceSynch.ReadMode.ONLY_ONCE);

        super.registerArmingStateCallback(false);
        this.deviceClient.setI2cAddress(mAddress);      // redundant?
        this.deviceClient.engage();
    }

    public boolean readFrame() {
        // read frame data from the PX4FLOW module
        mCache = mDeviceReader.read(mFrameReg, mFrameLen);
        frame = new px4_frame(mCache);
        return (frame != null);
    }

    public boolean readIntegral() {
        // read integral data from the PX4FLOW module
        mCache = mDeviceReader.read(mIntegralReg, mIntegralLen);
        iframe = new px4_integral_frame(mCache);
        return (frame != null);
    }


    // Simple frame
    public short frame_count() {
        return frame.frame_count;
    }
    public short pixel_flow_x_sum() {
        return frame.pixel_flow_x_sum;
    }
    public short pixel_flow_y_sum() {
        return frame.pixel_flow_y_sum;
    }
    public short flow_comp_m_x() {
        return frame.flow_comp_m_x;
    }
    public short flow_comp_m_y() {
        return frame.flow_comp_m_y;
    }
    public short gyro_x_rate() {
        return frame.gyro_x_rate;
    }
    public short gyro_y_rate() {
        return frame.gyro_y_rate;
    }
    public short gyro_z_rate() {
        return frame.gyro_z_rate;
    }
    public short quality() {
        return frame.quality;
    }
    public short sonar_timestamp() {
        return frame.sonar_timestamp;
    }
    public short ground_distance() {
        return frame.ground_distance;
    }

    // Integral frame
    public short frame_count_since_last_readout() {
        return iframe.frame_count_since_last_readout;
    }
    public short pixel_flow_x_integral() {
        return iframe.pixel_flow_x_integral;
    }
    public short pixel_flow_y_integral() {
        return iframe.pixel_flow_y_integral;
    }
    public short gyro_x_rate_integral() {
        return iframe.gyro_x_rate_integral;
    }
    public short gyro_y_rate_integral() {
        return iframe.gyro_y_rate_integral;
    }
    public short gyro_z_rate_integral() {
        return iframe.gyro_z_rate_integral;
    }
    public int integration_timespan() { return iframe.integration_timespan; }
    public int sonar_timestamp_integral() {
        return iframe.sonar_timestamp;
    }
    public short ground_distance_integral() {
        return iframe.ground_distance;
    }
    public short gyro_temperature() {
        return iframe.gyro_temperature;
    }
    public short quality_integral() { return iframe.quality; }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "PixyCam";
    }

}
