using System;
using System.Collections.Generic;
using System.Net;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>
    /// The map's single entry for protocol lines: the live brain and a recorded flight both pass through Receive, so
    /// they draw the same. It listens live first and replays the newest recording only if no brain connects in time.
    /// Messages arrive in bursts, a move only once it is over, so while playing they are shown one after another:
    /// the drone glides to each pose and each photo's floor is uncovered gradually once the drone is there.
    /// </summary>
    [RequireComponent(typeof(VoxelMap), typeof(FlightTrail), typeof(VictimMarkers))]
    [RequireComponent(typeof(StatusPanel), typeof(FlightReplay), typeof(FloorPlan))]
    public sealed class MapReceiver : MonoBehaviour
    {
        [Tooltip("TCP port the brain connects to (docs/protocol.md).")]
        [SerializeField] int port = 5005;

        [Tooltip("Address to listen on. Empty picks this PC's Tailscale address, or 127.0.0.1 without Tailscale. Never 0.0.0.0.")]
        [SerializeField] string listenAddress = "";

        [Tooltip("Seconds to wait for a live brain before replaying the newest recording in flights/.")]
        [SerializeField, Min(0f)] float liveWaitS = 5f;

        // Long enough to see the floor spread out from the drone, short against the seconds between photos.
        const float RevealS = 0.7f;
        // Each message waiting makes the playback this much faster, so a burst is caught up within a few moves.
        const float CatchUpPerMessage = 0.5f;

        readonly Queue<(string line, MessageHeader header)> pending = new();
        LineServer server;
        int seenConnections;
        bool replayTried;
        Observation revealing;
        float revealProgress;

        FloorPlan Plan => GetComponent<FloorPlan>();
        VoxelMap Map => GetComponent<VoxelMap>();
        FlightTrail Trail => GetComponent<FlightTrail>();
        VictimMarkers Victims => GetComponent<VictimMarkers>();
        StatusPanel Status => GetComponent<StatusPanel>();
        FlightReplay Replay => GetComponent<FlightReplay>();

        /// <summary>Take one protocol line: shown at once outside Play mode (tests, captures), queued while playing.</summary>
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
            if (header == null)
                return;
            if (Application.isPlaying)
                pending.Enqueue((line, header));
            else
                Show(line, header, animate: false);
        }

        void Show(string line, MessageHeader header, bool animate)
        {
            // Unknown types are ignored, as docs/protocol.md requires.
            switch (header.type)
            {
                case "session_start":
                    var start = JsonUtility.FromJson<SessionStart>(line);
                    if (Map.StartSession(start))
                    {
                        Trail.StartSession(start);
                        Plan.Clear();
                        Victims.Clear();
                        Status.Clear();
                        revealing = null;
                    }
                    break;
                case "observation":
                    var observation = JsonUtility.FromJson<Observation>(line);
                    Trail.ShowPose(observation.pose);
                    if (animate)
                    {
                        revealing = observation;
                        revealProgress = 0f;
                    }
                    else
                        FinishReveal(observation);
                    break;
                case "move":
                    Trail.AddMove(JsonUtility.FromJson<Move>(line));
                    if (animate)
                        Plan.FadeScanOut();
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
            IPAddress address = string.IsNullOrEmpty(listenAddress) ? LineServer.TailscaleOrLoopback() : IPAddress.Parse(listenAddress);
            server = new LineServer(address, port);
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
            Play();

            if (seenConnections == 0 && !replayTried && Time.timeSinceLevelLoad >= liveWaitS)
            {
                replayTried = true;
                Replay.TryLoad();
            }
            ShowLink();
        }

        void OnDestroy() => server?.Dispose();

        /// <summary>Show the queued messages in order, each only once the drone has reached the pose before it.</summary>
        void Play()
        {
            float pace = 1f + CatchUpPerMessage * pending.Count;
            Trail.Pace = pace;
            while (true)
            {
                if (Trail.Moving)
                    return;
                if (revealing != null)
                {
                    revealProgress += Time.deltaTime * pace / RevealS;
                    if (revealProgress < 1f)
                    {
                        Map.Observe(revealing, revealProgress);
                        Plan.ShowScan(revealing, revealProgress);
                        return;
                    }
                    FinishReveal(revealing);
                    revealing = null;
                }
                if (pending.Count == 0)
                    return;
                (string line, MessageHeader header) = pending.Dequeue();
                Show(line, header, animate: true);
            }
        }

        void FinishReveal(Observation observation)
        {
            Map.Observe(observation);
            Plan.Commit(observation);
            Plan.RefreshWalls(Map.Grid);
        }

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
