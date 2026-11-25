using System.Net;
using System.Text;
using SkyLead.Map;
using UnityEngine;

namespace SkyLead.Simulator
{
    /// <summary>
    /// The simulator's end of docs/simulator.md: takes the brain's commands from TCP, flies them on the simulated drone
    /// and answers each one when it is done, with text, the state line or a photo.
    /// </summary>
    [RequireComponent(typeof(SimulatedDrone))]
    public sealed class SimulatorLink : MonoBehaviour
    {
        [Tooltip("TCP port the brain connects to (docs/simulator.md): the map keeps 5005.")]
        [SerializeField] int port = 5006;

        [Tooltip("Address to listen on. Empty picks this PC's Tailscale address, or 127.0.0.1 without Tailscale. Never 0.0.0.0.")]
        [SerializeField] string listenAddress = "";

        [SerializeField] TelloCamera telloCamera;

        // docs/simulator.md, section 5.
        const int JpegQuality = 90;

        LineServer server;
        int seenConnections;
        int connection;

        SimulatedDrone Drone => GetComponent<SimulatedDrone>();

        /// <summary>Answer one line and return the reply's bytes, or null when the drone starts a move and replies later.</summary>
        public byte[] Handle(string line)
        {
            if (!DroneCommand.TryParse(line, out DroneCommand command, out string error))
                return Line(error);
            switch (command.Kind)
            {
                case CommandKind.Photo:
                    byte[] jpeg = telloCamera.CaptureJpeg(JpegQuality);
                    // The size goes first because JPEG bytes can contain '\n' (docs/simulator.md, section 5).
                    byte[] header = Line($"photo {jpeg.Length}");
                    var reply = new byte[header.Length + jpeg.Length];
                    header.CopyTo(reply, 0);
                    jpeg.CopyTo(reply, header.Length);
                    return reply;
                case CommandKind.State:
                    return Line(Drone.StateLine());
                default:
                    string text = Drone.Begin(command);
                    return text == null ? null : Line(text);
            }
        }

        // '\n' always: C#'s WriteLine ends lines with "\r\n" on Windows, which the contract does not allow.
        static byte[] Line(string text) => Encoding.ASCII.GetBytes(text + "\n");

        void Start()
        {
            IPAddress address = string.IsNullOrEmpty(listenAddress) ? LineServer.TailscaleOrLoopback() : IPAddress.Parse(listenAddress);
            server = new LineServer(address, port);
            server.Start();
            Debug.Log($"Simulator listening on {server.Endpoint}");
        }

        void Update()
        {
            // A new connection is a new flight (docs/simulator.md, section 2).
            if (server.Connections > seenConnections)
            {
                seenConnections = server.Connections;
                connection = seenConnections;
                Drone.ResetToPad();
            }

            if (Drone.Busy)
            {
                if (Drone.Step(Time.deltaTime, out string done))
                    server.Reply(Line(done));
                return;
            }

            // One command at a time: the brain waits for each reply, as the Tello SDK requires.
            while (server.TryDequeue(out string line, out int from))
            {
                // Lines a replaced connection sent before it closed belong to a flight that is over.
                if (from != connection)
                    continue;
                byte[] reply = Handle(line);
                if (reply != null)
                    server.Reply(reply);
                break;
            }
        }

        void OnDestroy() => server?.Dispose();
    }
}
