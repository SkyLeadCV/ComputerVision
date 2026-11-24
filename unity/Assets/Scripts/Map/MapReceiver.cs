using System;
using System.Net;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>
    /// The map's single entry for protocol lines: the live brain and a recorded flight both pass through Receive, so
    /// they draw the same. It listens live first and replays the newest recording only if no brain connects in time.
    /// </summary>
    [RequireComponent(typeof(VoxelMap), typeof(FlightTrail), typeof(VictimMarkers))]
    [RequireComponent(typeof(StatusPanel), typeof(FlightReplay))]
    public sealed class MapReceiver : MonoBehaviour
    {
        [Tooltip("TCP port the brain connects to (docs/protocol.md).")]
        [SerializeField] int port = 5005;

        [Tooltip("Address to listen on. Empty picks this PC's Tailscale address, or 127.0.0.1 without Tailscale. Never 0.0.0.0.")]
        [SerializeField] string listenAddress = "";

        [Tooltip("Seconds to wait for a live brain before replaying the newest recording in flights/.")]
        [SerializeField, Min(0f)] float liveWaitS = 5f;

        MapServer server;
        int seenConnections;
        bool replayTried;

        VoxelMap Map => GetComponent<VoxelMap>();
        FlightTrail Trail => GetComponent<FlightTrail>();
        VictimMarkers Victims => GetComponent<VictimMarkers>();
        StatusPanel Status => GetComponent<StatusPanel>();
        FlightReplay Replay => GetComponent<FlightReplay>();

        public void Receive(string line)
        {
            MessageHeader header;
            try
            {
                header = JsonUtility.FromJson<MessageHeader>(line);
            }
            catch (ArgumentException)
            {
                Debug.LogWarning($"Ignoring a line that is not JSON: {line}");
                return;
            }

            // Unknown types are ignored, as docs/protocol.md requires.
            switch (header?.type)
            {
                case "session_start":
                    var start = JsonUtility.FromJson<SessionStart>(line);
                    if (Map.StartSession(start))
                    {
                        Trail.StartSession(start);
                        Victims.Clear();
                        Status.Clear();
                    }
                    break;
                case "observation":
                    var observation = JsonUtility.FromJson<Observation>(line);
                    Map.Observe(observation);
                    Trail.ShowObservation(observation);
                    break;
                case "move":
                    Trail.AddMove(JsonUtility.FromJson<Move>(line));
                    break;
                case "victim":
                    Victims.Place(JsonUtility.FromJson<Victim>(line));
                    break;
                case "state":
                    Status.ShowState(JsonUtility.FromJson<MissionState>(line));
                    break;
                case "session_end":
                    Status.ShowEnd(JsonUtility.FromJson<SessionEnd>(line));
                    break;
                default:
                    return;
            }
            Status.MissionTimeS = header.t;
        }

        void Start()
        {
            IPAddress address = string.IsNullOrEmpty(listenAddress) ? MapServer.TailscaleOrLoopback() : IPAddress.Parse(listenAddress);
            server = new MapServer(address, port);
            server.Start();
            Debug.Log($"Map listening on {server.Endpoint}");
        }

        void Update()
        {
            // Live always wins: a brain that connects during a replay stops it, and its own session_start clears
            // the replayed flight.
            if (server.Connections > seenConnections)
            {
                seenConnections = server.Connections;
                Replay.Stop();
            }
            while (server.TryDequeue(out string line))
                Receive(line);

            if (seenConnections == 0 && !replayTried && Time.timeSinceLevelLoad >= liveWaitS)
            {
                replayTried = true;
                Replay.TryLoad();
            }
            ShowLink();
        }

        void OnDestroy() => server?.Dispose();

        void ShowLink()
        {
            Status.LinkDetail = server.Endpoint.ToString();
            if (server.Connected)
                Status.Link = MapLink.Live;
            else if (seenConnections > 0)
                Status.Link = MapLink.Lost;
            else if (Replay.FileName != null)
            {
                Status.Link = MapLink.Replay;
                Status.LinkDetail = Replay.FileName;
            }
            else
                Status.Link = MapLink.Waiting;
        }
    }
}
