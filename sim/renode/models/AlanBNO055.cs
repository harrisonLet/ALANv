// Virtual Bosch BNO055 IMU (I2C, address 0x28) for the ALAN Renode board.
// Answers the registers CM7/App/Src/sensorMagnetometer.c reads.
// Monitor: i2c2.bno055 Heading 90   (deg, 0-360, what the firmware's yaw shows)
//          i2c2.bno055 YawRate 5    (deg/s, gyro Z)
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AlanBNO055 : II2CPeripheral
    {
        public AlanBNO055()
        {
            Reset();
        }

        public void Reset()
        {
            regs = new byte[0x80];
            regs[0x00] = 0xA0; // CHIP_ID
            regs[0x01] = 0xFB; // ACC_ID
            regs[0x02] = 0x32; // MAG_ID
            regs[0x03] = 0x0F; // GYR_ID
            regs[0x35] = 0xFF; // CALIB_STAT: sys/gyro/acc/mag all 3
            regs[0x36] = 0x0F; // ST_RESULT: all self tests pass
            regs[0x39] = 0x05; // SYS_STATUS: fusion running
            pointer = 0;
            Heading = 0;
            YawRate = 0;
        }

        public double Heading { get; set; }
        public double YawRate { get; set; }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            for(var i = 1; i < data.Length; i++)
            {
                var reg = (pointer + i - 1) & 0x7F;
                // Keep the ID, status and calibration-status registers read-only
                if(reg > 0x03 && reg != 0x35 && reg != 0x36 && reg != 0x39)
                {
                    regs[reg] = data[i];
                }
            }
        }

        public byte[] Read(int count = 1)
        {
            UpdateMeasurements();
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = regs[(pointer + i) & 0x7F];
            }
            pointer = (byte)((pointer + count) & 0x7F);
            return result;
        }

        public void FinishTransmission()
        {
        }

        private void UpdateMeasurements()
        {
            var h = ((Heading % 360) + 360) % 360;

            // Accelerometer: boat level, 1 g on Z (1 m/s^2 = 100 LSB)
            Put16(0x08, 0); Put16(0x0A, 0); Put16(0x0C, 981);

            // Magnetometer (1 uT = 16 LSB): horizontal field pointing to magnetic north
            var rad = h * Math.PI / 180.0;
            Put16(0x0E, (short)(16 * 22 * Math.Cos(rad)));
            Put16(0x10, (short)(-16 * 22 * Math.Sin(rad)));
            Put16(0x12, (short)(16 * -40));

            // Gyroscope (1 dps = 16 LSB)
            Put16(0x14, 0); Put16(0x16, 0); Put16(0x18, (short)(16 * YawRate));

            // Euler angles (1 deg = 16 LSB): heading, roll, pitch
            Put16(0x1A, (short)(16 * h)); Put16(0x1C, 0); Put16(0x1E, 0);

            // Quaternion (1 = 16384 LSB), rotation about Z so the firmware's yaw = Heading
            var half = rad / 2.0;
            Put16(0x20, (short)(16384 * Math.Cos(half))); // W
            Put16(0x22, 0);                                // X
            Put16(0x24, 0);                                // Y
            Put16(0x26, (short)(16384 * Math.Sin(half))); // Z
        }

        private void Put16(int reg, short value)
        {
            regs[reg] = (byte)(value & 0xFF);
            regs[reg + 1] = (byte)((value >> 8) & 0xFF);
        }

        private byte[] regs;
        private byte pointer;
    }
}
