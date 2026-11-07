// Connects the emulated STM32 to the boat physics simulator (sim/plant) in lockstep.
//
// Every 50 ms of virtual time (20 Hz) it:
//   1. reads the servo pulse widths the firmware set (TIM1 CCR1 = tail sail, TIM1 CCR2 = rudder, in microseconds)
//   2. sends them to the plant over TCP and waits for its answer (virtual time is frozen while it waits)
//   3. writes the plant's sensor values into the BNO055, AS5600, wind vane and GPS models
//
// If no plant is running, the board keeps running with the sensors' last values and retries once per virtual second.
// Monitor: plant Connected | plant Port 5555 | plant Enabled false
using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AlanPlantBridge : IDoubleWordPeripheral, IKnownSize
    {
        public AlanPlantBridge(IMachine machine, IPeripheral imu, IPeripheral encoder, IPeripheral windVane, IPeripheral gps)
        {
            this.machine = machine;
            this.imu = imu;
            this.encoder = encoder;
            this.windVane = windVane;
            this.gps = gps;
            Host = "127.0.0.1";
            Port = 5555;
            Enabled = true;
            timer = new LimitTimer(machine.ClockSource, 1000, this, "plantTick", limit: PeriodMs, direction: Direction.Descending, enabled: true, eventEnabled: true, autoUpdate: true);
            timer.LimitReached += Tick;
        }

        public string Host { get; set; }
        public int Port { get; set; }
        public bool Enabled { get; set; }
        // Never run faster than this multiple of real time (1 = real time). 0 = as fast as possible.
        public double SpeedFactor { get; set; } = 1.0;
        public bool Connected => client != null && client.Connected;

        public long Size => 0x10;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        public void Reset()
        {
            Disconnect();
            ticks = 0;
        }

        private void Tick()
        {
            ticks++;
            if(!Enabled)
            {
                return;
            }
            if(!Connected)
            {
                if(ticks % (1000 / PeriodMs) == 1)
                {
                    TryConnect();
                }
                if(!Connected)
                {
                    return;
                }
            }

            // real-time cap: if virtual time is ahead of the wall clock, wait (this pauses the emulated chip)
            if(SpeedFactor > 0)
            {
                if(!clock.IsRunning) { clock.Start(); clockStartTicks = ticks; }
                var aheadMs = (ticks - clockStartTicks) * PeriodMs / SpeedFactor - clock.Elapsed.TotalMilliseconds;
                if(aheadMs > 1) System.Threading.Thread.Sleep((int)Math.Min(aheadMs, 200));
                if(aheadMs < -2000) { clock.Restart(); clockStartTicks = ticks; }   // fell far behind: don't try to catch up
            }

            var bus = machine.GetSystemBus(this);
            var tailUs = bus.ReadDoubleWord(Tim1Ccr1);
            var rudderUs = bus.ReadDoubleWord(Tim1Ccr2);
            var t = ticks * PeriodMs / 1000.0;

            try
            {
                writer.Write(string.Format(CultureInfo.InvariantCulture, "t={0:0.000} tail={1} rudder={2}\n", t, tailUs, rudderUs));
                writer.Flush();
                var reply = reader.ReadLine();
                if(reply == null)
                {
                    throw new IOException("plant closed the connection");
                }
                Apply(reply);
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Warning, "Plant link lost ({0}); sensors hold their last values", e.Message);
                Disconnect();
            }
        }

        private void Apply(string reply)
        {
            foreach(var field in reply.Split(' '))
            {
                var kv = field.Split('=');
                if(kv.Length != 2)
                {
                    continue;
                }
                double v;
                if(!double.TryParse(kv[1], NumberStyles.Float, CultureInfo.InvariantCulture, out v))
                {
                    continue;
                }
                switch(kv[0])
                {
                    case "heading":  Set(imu, "Heading", v); break;
                    case "yaw_rate": Set(imu, "YawRate", v); break;
                    case "wing":     Set(encoder, "Angle", v); break;
                    case "vane":     Set(windVane, "Angle", v); break;
                    case "vane_ok":  Set(windVane, "Enabled", v != 0); break;
                    case "lat":      Set(gps, "Latitude", v); break;
                    case "lon":      Set(gps, "Longitude", v); break;
                    case "gps_fix":  Set(gps, "Fix", (int)v); break;
                    case "gps_ok":   Set(gps, "Enabled", v != 0); break;
                }
            }
        }

        // The sensor models are compiled separately, so their properties are set by name.
        private static void Set(IPeripheral target, string property, object value)
        {
            var prop = target?.GetType().GetProperty(property);
            prop?.SetValue(target, Convert.ChangeType(value, prop.PropertyType, CultureInfo.InvariantCulture));
        }

        private void TryConnect()
        {
            try
            {
                var c = new TcpClient();
                if(!c.ConnectAsync(Host, Port).Wait(200))
                {
                    c.Dispose();
                    if(!warned)
                    {
                        this.Log(LogLevel.Info, "No plant at {0}:{1} yet (start sim/plant/run_plant.py); retrying every virtual second", Host, Port);
                        warned = true;
                    }
                    return;
                }
                c.NoDelay = true;
                var stream = c.GetStream();
                stream.ReadTimeout = 10000;
                client = c;
                reader = new StreamReader(stream);
                writer = new StreamWriter(stream);
                this.Log(LogLevel.Info, "Connected to plant at {0}:{1}", Host, Port);
            }
            catch(Exception)
            {
                Disconnect();
            }
        }

        private void Disconnect()
        {
            try { client?.Close(); } catch(Exception) { }
            client = null;
            reader = null;
            writer = null;
        }

        private const int PeriodMs = 50;
        private const long Tim1Ccr1 = 0x40010034;
        private const long Tim1Ccr2 = 0x40010038;

        private readonly IMachine machine;
        private readonly IPeripheral imu;
        private readonly IPeripheral encoder;
        private readonly IPeripheral windVane;
        private readonly IPeripheral gps;
        private readonly LimitTimer timer;
        private TcpClient client;
        private StreamReader reader;
        private StreamWriter writer;
        private long ticks;
        private long clockStartTicks;
        private readonly Stopwatch clock = new Stopwatch();
        private bool warned;
    }
}
