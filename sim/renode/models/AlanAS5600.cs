// Virtual AMS AS5600 magnetic encoder (I2C, address 0x36) for the ALAN Renode board.
// Answers the registers CM7/App/Src/sensorEncoder.c reads.
// Monitor: i2c1.as5600 Angle 45   (deg, 0-360)
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AlanAS5600 : II2CPeripheral
    {
        public AlanAS5600()
        {
            Reset();
        }

        public void Reset()
        {
            regs = new byte[0x100];
            pointer = 0;
            Angle = 0;
        }

        public double Angle { get; set; }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            for(var i = 1; i < data.Length; i++)
            {
                var reg = (pointer + i - 1) & 0xFF;
                if(reg <= 0x08) // only the configuration registers are writable
                {
                    regs[reg] = data[i];
                }
            }
        }

        public byte[] Read(int count = 1)
        {
            var raw = (int)Math.Round((((Angle % 360) + 360) % 360) * 4096.0 / 360.0) & 0x0FFF;
            Put12(0x0C, raw);   // RAW ANGLE
            Put12(0x0E, raw);   // ANGLE (no ZPOS/MPOS programmed)
            regs[0x0B] = 0x20;  // STATUS: MD (magnet detected)
            regs[0x1A] = 64;    // AGC: mid-range at 3.3 V
            Put12(0x1B, 2048);  // MAGNITUDE

            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = regs[(pointer + i) & 0xFF];
            }
            pointer = (byte)((pointer + count) & 0xFF);
            return result;
        }

        public void FinishTransmission()
        {
        }

        private void Put12(int regH, int value)
        {
            regs[regH] = (byte)((value >> 8) & 0x0F);
            regs[regH + 1] = (byte)(value & 0xFF);
        }

        private byte[] regs;
        private byte pointer;
    }
}
