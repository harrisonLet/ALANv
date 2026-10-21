// Virtual DFRobot SEN0482 RS485 wind vane (Modbus RTU, address 0x02) for the ALAN Renode board.
// Answers "read holding register" requests from CM7/App/Src/sensorWind.c on UART4.
// Monitor: windvane Angle 270   (deg, 0-360)   windvane Enabled false   (stops answering, like a cut cable)
// Sits at a dummy sysbus address only so the monitor can reach it; the firmware never touches it.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AlanWindVane : IDoubleWordPeripheral, IKnownSize
    {
        public AlanWindVane(IUART uart)
        {
            this.uart = uart;
            uart.CharReceived += OnMcuByte;
            Reset();
        }

        public void Reset()
        {
            request.Clear();
            Angle = 0;
            Address = 0x02;
            Enabled = true;
        }

        public double Angle { get; set; }
        public byte Address { get; set; }
        public bool Enabled { get; set; }

        private void OnMcuByte(byte value)
        {
            request.Add(value);
            if(request.Count > 8)
            {
                request.RemoveAt(0);
            }
            if(request.Count == 8 && request[0] == Address && request[1] == 0x03 && Crc(request, 6) == (request[6] | (request[7] << 8)))
            {
                var register = (request[2] << 8) | request[3];
                request.Clear();
                if(Enabled)
                {
                    Respond(register);
                }
            }
        }

        public long Size => 0x10;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        private void Respond(int register)
        {
            var a = ((Angle % 360) + 360) % 360;
            int value = register == 0x0000 ? (int)Math.Round(a * 10)            // angle x10
                      : register == 0x0001 ? (int)Math.Round(a / 22.5) % 16     // 16-point direction
                      : 0;
            var frame = new List<byte> { Address, 0x03, 0x02, (byte)(value >> 8), (byte)(value & 0xFF) };
            var crc = Crc(frame, 5);
            frame.Add((byte)(crc & 0xFF));
            frame.Add((byte)(crc >> 8));
            foreach(var b in frame)
            {
                uart.WriteChar(b);
            }
        }

        private static int Crc(IList<byte> buf, int len)
        {
            var crc = 0xFFFF;
            for(var i = 0; i < len; i++)
            {
                crc ^= buf[i];
                for(var b = 0; b < 8; b++)
                {
                    crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xA001 : crc >> 1;
                }
            }
            return crc;
        }

        private readonly List<byte> request = new List<byte>();
        private readonly IUART uart;
    }
}
