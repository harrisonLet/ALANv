// Virtual u-blox NEO-6M GPS for the ALAN Renode board.
// Sends a $GPGGA sentence once per virtual second on UART7, which CM7/App/Src/sensorGPS.c parses.
// Sits at a dummy sysbus address only so the monitor can reach it; the firmware never touches it.
// Monitor: gps Latitude 40.7649   gps Longitude -111.8421   gps Satellites 8   gps Fix 1   gps Enabled false (silent)
using System;
using System.Collections.Generic;
using System.Globalization;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AlanGPS : IDoubleWordPeripheral, IKnownSize
    {
        public AlanGPS(IMachine machine, IUART uart)
        {
            this.uart = uart;
            timer = new LimitTimer(machine.ClockSource, 1000, this, "gpsTick", limit: 1000, direction: Direction.Descending, enabled: true, eventEnabled: true, autoUpdate: true);
            timer.LimitReached += SendFix;
            // Shift characters out at the real 9600 baud rate (10 bits per character = 960 chars/s)
            charTimer = new LimitTimer(machine.ClockSource, 9600, this, "gpsBaud", limit: 10, direction: Direction.Descending, enabled: true, eventEnabled: true, autoUpdate: true);
            charTimer.LimitReached += SendNextChar;
            Reset();
        }

        public void Reset()
        {
            Latitude = 40.7649;     // University of Utah area
            Longitude = -111.8421;
            Altitude = 1400;
            Satellites = 8;
            Fix = 1;
            Enabled = true;
            seconds = 0;
            pending.Clear();
        }

        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double Altitude { get; set; }
        public int Satellites { get; set; }
        public int Fix { get; set; }
        public bool Enabled { get; set; }

        public long Size => 0x10;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        private void SendFix()
        {
            seconds++;
            if(!Enabled)
            {
                return;
            }
            var t = TimeSpan.FromSeconds(seconds);
            var body = string.Format(CultureInfo.InvariantCulture,
                "GPGGA,{0:00}{1:00}{2:00}.00,{3},{4},{5},{6},{7},{8:00},1.0,{9:0.0},M,0.0,M,,",
                t.Hours, t.Minutes, t.Seconds,
                Nmea(Math.Abs(Latitude), 2), Latitude >= 0 ? "N" : "S",
                Nmea(Math.Abs(Longitude), 3), Longitude >= 0 ? "E" : "W",
                Fix, Satellites, Altitude);
            byte checksum = 0;
            foreach(var c in body)
            {
                checksum ^= (byte)c;
            }
            var sentence = string.Format("${0}*{1:X2}\r\n", body, checksum);
            foreach(var c in sentence)
            {
                pending.Enqueue((byte)c);
            }

        }

        private void SendNextChar()
        {
            if(pending.Count == 0)
            {
                return;
            }
            uart.WriteChar(pending.Dequeue());
        }

        // Degrees -> NMEA ddmm.mmmmm / dddmm.mmmmm
        private static string Nmea(double degrees, int degreeDigits)
        {
            var d = (int)degrees;
            var minutes = (degrees - d) * 60.0;
            return d.ToString(new string('0', degreeDigits), CultureInfo.InvariantCulture)
                 + minutes.ToString("00.00000", CultureInfo.InvariantCulture);
        }

        private readonly LimitTimer timer;
        private readonly LimitTimer charTimer;
        private readonly Queue<byte> pending = new Queue<byte>();
        private readonly IUART uart;
        private long seconds;
    }
}
