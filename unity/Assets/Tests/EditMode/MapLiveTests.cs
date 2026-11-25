using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using Object = UnityEngine.Object;

namespace SkyLead.Map.Tests
{
    public class MapLiveTests
    {
        const string CameraJson = "\"camera\": {\"hfov_deg\": 54.5, \"vfov_up_deg\": 20.4, \"vfov_down_deg\": 22.1}";
        // A drone at a voxel centre (5, 5) looking forward, with the floor free from 100 to 250 cm.
        const string ObservationLine = "{\"type\": \"observation\", \"seq\": 1, \"t\": 1.0, "
            + "\"pose\": {\"x\": 5.0, \"y\": 5.0, \"z\": 80.0, \"yaw_deg\": 0.0, \"pitch_deg\": 0.0}, "
            + "\"rays\": {\"angle_deg\": [0.0], \"entry_cm\": [100.0], \"stop_cm\": [250.0], \"kind\": [2]}}";

        GameObject mapObject;
        MapReceiver receiver;

        static string SessionStartLine(string session) =>
            $"{{\"type\": \"session_start\", \"seq\": 0, \"t\": 0.0, \"version\": 1, \"session\": \"{session}\", {CameraJson}, "
            + "\"drone_radius_cm\": 9, \"safety_margin_cm\": 15}";

        // Invariant, because a Spanish Windows would write 5.5 as "5,5" and break the JSON.
        static string MoveLine(string command, float fromX, float fromY, float fromYaw, float toX, float toY, float toYaw) =>
            FormattableString.Invariant($"{{\"type\": \"move\", \"seq\": 2, \"t\": 2.0, \"command\": \"{command}\", ")
            + FormattableString.Invariant($"\"from\": {{\"x\": {fromX}, \"y\": {fromY}, \"yaw_deg\": {fromYaw}}}, ")
            + FormattableString.Invariant($"\"to\": {{\"x\": {toX}, \"y\": {toY}, \"yaw_deg\": {toYaw}}}}}");

        static string VictimLine(int id, float x, float y) =>
            FormattableString.Invariant($"{{\"type\": \"victim\", \"seq\": 3, \"t\": 3.0, \"id\": {id}, \"x\": {x}, \"y\": {y}, \"confidence\": 0.87}}");

        [SetUp]
        public void CreateMap()
        {
            mapObject = new GameObject("Test map");
            receiver = mapObject.AddComponent<MapReceiver>();
        }

        [TearDown]
        public void DestroyMap() => Object.DestroyImmediate(mapObject);

        Voxel InFrontOfTheDrone()
        {
            VoxelGrid grid = mapObject.GetComponent<VoxelMap>().Grid;
            Assert.IsTrue(grid.TryCell(5f, 205f, 85f, out Vector3Int cell));
            return grid[cell.x, cell.y, cell.z];
        }

        [Test]
        public void TheSameSessionContinuesTheMapAndANewOneClearsIt()
        {
            receiver.Receive(SessionStartLine("20261124_190000"));
            receiver.Receive(ObservationLine);
            receiver.Receive(MoveLine("forward 100", 5f, 5f, 0f, 5f, 105f, 0f));
            receiver.Receive(VictimLine(1, 100f, 100f));
            Assert.AreEqual(Voxel.Empty, InFrontOfTheDrone());

            // A reconnection re-sends the same session_start.
            receiver.Receive(SessionStartLine("20261124_190000"));
            Assert.AreEqual(Voxel.Empty, InFrontOfTheDrone());
            Assert.AreEqual(2, mapObject.GetComponent<FlightTrail>().Points.Count);
            Assert.AreEqual(1, mapObject.GetComponent<VictimMarkers>().Count);

            receiver.Receive(SessionStartLine("20261124_200000"));
            Assert.AreEqual(Voxel.Solid, InFrontOfTheDrone());
            Assert.AreEqual(0, mapObject.GetComponent<FlightTrail>().Points.Count);
            Assert.AreEqual(0, mapObject.GetComponent<VictimMarkers>().Count);
        }

        [Test]
        public void OnlyForwardStepsLengthenTheTrailWhileTurnsRotateTheDrone()
        {
            var trail = mapObject.GetComponent<FlightTrail>();
            receiver.Receive(SessionStartLine("s"));

            receiver.Receive(MoveLine("cw 90", 0f, 0f, 0f, 0f, 0f, 90f));
            Assert.AreEqual(1, trail.Points.Count);
            Assert.AreEqual(90f, trail.Drone.localEulerAngles.y, 1e-3f);

            receiver.Receive(MoveLine("forward 100", 0f, 0f, 90f, 100f, 0f, 90f));
            Assert.AreEqual(2, trail.Points.Count);
            // 100 cm along +x at the default 80 cm flight height, in Unity metres with y up.
            Assert.Less(Vector3.Distance(new Vector3(1f, 0.8f, 0f), trail.Points[1]), 1e-5f);
            Assert.Less(Vector3.Distance(trail.Points[1], trail.Drone.localPosition), 1e-5f);
        }

        [Test]
        public void ARepeatedVictimIdMovesItsMarker()
        {
            var victims = mapObject.GetComponent<VictimMarkers>();
            receiver.Receive(SessionStartLine("s"));

            receiver.Receive(VictimLine(1, 100f, 100f));
            receiver.Receive(VictimLine(1, 200f, 100f));
            Assert.AreEqual(1, victims.Count);
            Assert.Less(Vector3.Distance(new Vector3(2f, 0f, 1f), victims.PositionOf(1)), 1e-5f);

            receiver.Receive(VictimLine(2, 300f, 100f));
            Assert.AreEqual(2, victims.Count);
        }

        [Test]
        public void TheStatusShowsTheModeTheRescuerAndTheEndReasonAsTheyArrive()
        {
            var status = mapObject.GetComponent<StatusPanel>();
            receiver.Receive(SessionStartLine("s"));

            StringAssert.Contains("MODE STANDBY", status.Text);

            receiver.Receive("{\"type\": \"state\", \"seq\": 4, \"t\": 64.0, \"mode\": \"guiding\", \"person_distance_cm\": 180.0}");
            StringAssert.Contains("MODE GUIDING", status.Text);
            StringAssert.Contains("RESCUER 1.8 m", status.Text);
            StringAssert.Contains("T+ 01:04", status.Text);

            receiver.Receive("{\"type\": \"state\", \"seq\": 5, \"t\": 65.0, \"mode\": \"waiting\", \"person_distance_cm\": -1}");
            StringAssert.Contains("MODE WAITING", status.Text);
            StringAssert.Contains("RESCUER NO CONTACT", status.Text);

            receiver.Receive(VictimLine(1, 100f, 100f));
            StringAssert.Contains("VICTIMS 1", status.Text);

            receiver.Receive("{\"type\": \"session_end\", \"seq\": 6, \"t\": 66.0, \"reason\": \"back_at_start\"}");
            StringAssert.Contains("END BACK_AT_START", status.Text);
        }

        [Test]
        public void TheSafetyCircleTakesTheSessionsDroneRadiusAndMargin()
        {
            var trail = mapObject.GetComponent<FlightTrail>();
            receiver.Receive(SessionStartLine("s").Replace("\"safety_margin_cm\": 15", "\"safety_margin_cm\": 21"));

            // drone_radius_cm 9 plus the 21 cm margin of this session.
            Assert.AreEqual(30f, trail.SafetyRadiusCm, 1e-4f);
        }

        [Test]
        public void TheLatestScanReachesEachRaysStopAndARayWithoutFloorReachesNothing()
        {
            var plan = mapObject.GetComponent<FloorPlan>();
            receiver.Receive(SessionStartLine("s"));
            // Facing +x (yaw 90): the first ray straight ahead saw floor up to 250 cm, the second saw none.
            receiver.Receive("{\"type\": \"observation\", \"seq\": 1, \"t\": 1.0, "
                + "\"pose\": {\"x\": 5.0, \"y\": 5.0, \"z\": 80.0, \"yaw_deg\": 90.0, \"pitch_deg\": 0.0}, "
                + "\"rays\": {\"angle_deg\": [0.0, 10.0], \"entry_cm\": [150.0, 150.0], \"stop_cm\": [250.0, 150.0], \"kind\": [0, 0]}}");

            Assert.AreEqual(2, plan.ScanRim.Count);
            Assert.Less(Vector3.Distance(VoxelGrid.ToUnity(255f, 5f, 0f), plan.ScanRim[0]), 1e-4f);
            Assert.Less(Vector3.Distance(VoxelGrid.ToUnity(5f, 5f, 0f), plan.ScanRim[1]), 1e-4f);
        }

        [Test]
        public void ARevealInStepsEndsExactlyAsOneFullObservationAndPaintsNoFaceBeforeTheEnd()
        {
            var revealed = new GameObject("Revealed").AddComponent<VoxelMap>();
            var whole = new GameObject("Whole").AddComponent<VoxelMap>();
            try
            {
                var session = JsonUtility.FromJson<SessionStart>(SessionStartLine("s"));
                revealed.StartSession(session);
                whole.StartSession(session);
                var observation = JsonUtility.FromJson<Observation>("{\"type\": \"observation\", \"seq\": 1, \"t\": 1.0, "
                    + "\"pose\": {\"x\": 5.0, \"y\": 5.0, \"z\": 80.0, \"yaw_deg\": 0.0, \"pitch_deg\": -11.0}, "
                    + "\"rays\": {\"angle_deg\": [-10.0, 0.0, 10.0], \"entry_cm\": [120.0, 120.0, 120.0], "
                    + "\"stop_cm\": [250.0, 300.0, 120.0], \"kind\": [0, 2, 0]}}");

                foreach (float progress in new[] { 0.2f, 0.5f, 0.9f })
                {
                    revealed.Observe(observation, progress);
                    Assert.IsFalse(HasVoxel(revealed.Grid, Voxel.Surface), $"a face painted at {progress}");
                }
                revealed.Observe(observation, 1f);
                whole.Observe(observation);

                for (int x = 0; x < whole.Grid.Size.x; x++)
                for (int y = 0; y < whole.Grid.Size.y; y++)
                for (int z = 0; z < whole.Grid.Size.z; z++)
                    Assert.AreEqual(whole.Grid[x, y, z], revealed.Grid[x, y, z], $"voxel ({x}, {y}, {z})");
                Assert.IsTrue(HasVoxel(whole.Grid, Voxel.Surface));
            }
            finally
            {
                Object.DestroyImmediate(revealed.gameObject);
                Object.DestroyImmediate(whole.gameObject);
            }
        }

        static bool HasVoxel(VoxelGrid grid, Voxel value)
        {
            for (int x = 0; x < grid.Size.x; x++)
            for (int y = 0; y < grid.Size.y; y++)
            for (int z = 0; z < grid.Size.z; z++)
            {
                if (grid[x, y, z] == value)
                    return true;
            }
            return false;
        }

        [Test]
        public void ThePlanDrawsOneWallDotPerColumnSeenAsAnObstacle()
        {
            receiver.Receive(SessionStartLine("s"));
            // Three rays ending on an obstacle 250 cm ahead, a little apart: a short stretch of wall.
            receiver.Receive("{\"type\": \"observation\", \"seq\": 1, \"t\": 1.0, "
                + "\"pose\": {\"x\": 5.0, \"y\": 5.0, \"z\": 80.0, \"yaw_deg\": 0.0, \"pitch_deg\": 0.0}, "
                + "\"rays\": {\"angle_deg\": [-4.0, 0.0, 4.0], \"entry_cm\": [120.0, 120.0, 120.0], "
                + "\"stop_cm\": [250.0, 250.0, 250.0], \"kind\": [0, 0, 0]}}");

            VoxelGrid grid = mapObject.GetComponent<VoxelMap>().Grid;
            int columns = 0;
            for (int x = 0; x < grid.Size.x; x++)
            for (int y = 0; y < grid.Size.y; y++)
            {
                for (int z = 0; z < grid.Size.z; z++)
                {
                    if (grid[x, y, z] != Voxel.Surface)
                        continue;
                    columns++;
                    break;
                }
            }
            Mesh walls = mapObject.transform.Find("Obstacles seen").GetComponent<MeshFilter>().sharedMesh;
            Assert.Greater(columns, 0);
            // A dot is a centre and eight rim points.
            Assert.AreEqual(columns * 9, walls.vertexCount);
        }

        [Test]
        public void UnknownTypesAndBrokenLinesAreIgnored()
        {
            receiver.Receive(SessionStartLine("s"));
            Assert.DoesNotThrow(() => receiver.Receive("{\"type\": \"future_message\", \"seq\": 7, \"t\": 7.0}"));
            UnityEngine.TestTools.LogAssert.Expect(LogType.Warning, "Ignoring a line that is not JSON: {\"type\": \"obs");
            Assert.DoesNotThrow(() => receiver.Receive("{\"type\": \"obs"));
        }

        [Test]
        public void TheMoveExampleOfTheProtocolKeepsItsFromField()
        {
            const string line = "{\"type\": \"move\", \"seq\": 13, \"t\": 35.0, \"command\": \"cw 15, forward 200\", "
                + "\"from\": {\"x\": 120.0, \"y\": 340.0, \"yaw_deg\": 90.0}, \"to\": {\"x\": 313.2, \"y\": 288.2, \"yaw_deg\": 105.0}}";
            var move = JsonUtility.FromJson<Move>(line);
            Assert.AreEqual(120f, move.@from.x);
            Assert.AreEqual(105f, move.to.yaw_deg);
            Assert.AreEqual("cw 15, forward 200", move.command);
        }

        [Test]
        public void LinesSplitAcrossPacketsAreJoined()
        {
            var splitter = new LineSplitter();
            // "é" is two bytes in UTF-8, cut here between packets.
            byte[] bytes = Encoding.UTF8.GetBytes("{\"a\": \"é\"}\r\n{\"b\": 2}\n{\"c\"");
            int cut = Array.IndexOf(bytes, (byte)0xC3) + 1;

            var lines = new List<string>(splitter.Feed(bytes[..cut], cut));
            lines.AddRange(splitter.Feed(bytes[cut..], bytes.Length - cut));
            CollectionAssert.AreEqual(new[] { "{\"a\": \"é\"}", "{\"b\": 2}" }, lines);

            byte[] rest = Encoding.UTF8.GetBytes(": 3}\n");
            CollectionAssert.AreEqual(new[] { "{\"c\": 3}" }, splitter.Feed(rest, rest.Length));
        }

        [Test]
        public void TheServerNeverListensOnEveryInterface()
        {
            Assert.Throws<ArgumentException>(() => new LineServer(IPAddress.Any, 0));
            Assert.Throws<ArgumentException>(() => new LineServer(IPAddress.IPv6Any, 0));
        }

        [Test]
        public void OnlyTheTailscaleRangeCountsAsTailscale()
        {
            Assert.IsTrue(LineServer.IsTailscale(IPAddress.Parse("100.64.0.1")));
            Assert.IsTrue(LineServer.IsTailscale(IPAddress.Parse("100.127.255.254")));
            Assert.IsFalse(LineServer.IsTailscale(IPAddress.Parse("100.63.255.255")));
            Assert.IsFalse(LineServer.IsTailscale(IPAddress.Parse("100.128.0.1")));
            Assert.IsFalse(LineServer.IsTailscale(IPAddress.Parse("192.168.10.1")));
        }

        [Test]
        public void TheServerReceivesLinesOverARealSocketAndANewConnectionReplacesTheOld()
        {
            using var server = new LineServer(IPAddress.Loopback, 0);
            server.Start();

            using var first = new TcpClient();
            first.Connect(server.Endpoint);
            NetworkStream stream = first.GetStream();
            Send(stream, "{\"seq\": 0}\n{\"se");
            Send(stream, "q\": 1}\n");
            CollectionAssert.AreEqual(new[] { "{\"seq\": 0}", "{\"seq\": 1}" }, Receive(server, 2));
            Assert.AreEqual(1, server.Connections);

            using var second = new TcpClient();
            second.Connect(server.Endpoint);
            Send(second.GetStream(), "{\"seq\": 2}\n");
            CollectionAssert.AreEqual(new[] { "{\"seq\": 2}" }, Receive(server, 1));
            Assert.AreEqual(2, server.Connections);
            Assert.IsTrue(server.Connected);
        }

        static void Send(NetworkStream stream, string text)
        {
            byte[] bytes = Encoding.UTF8.GetBytes(text);
            stream.Write(bytes, 0, bytes.Length);
            stream.Flush();
        }

        static List<string> Receive(LineServer server, int count)
        {
            var lines = new List<string>();
            var clock = Stopwatch.StartNew();
            while (lines.Count < count && clock.ElapsedMilliseconds < 2000)
            {
                if (server.TryDequeue(out string line))
                    lines.Add(line);
            }
            return lines;
        }
    }
}
