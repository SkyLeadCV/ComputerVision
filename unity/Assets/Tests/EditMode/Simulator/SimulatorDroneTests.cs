using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using NUnit.Framework;
using SkyLead.Map;
using UnityEngine;
using Object = UnityEngine.Object;

namespace SkyLead.Simulator.Tests
{
    public class SimulatorDroneTests
    {
        GameObject droneObject;
        GameObject wall;
        SimulatedDrone drone;

        [SetUp]
        public void CreateDrone()
        {
            droneObject = new GameObject("Simulated Drone");
            drone = droneObject.AddComponent<SimulatedDrone>();
            drone.ResetToPad();
        }

        [TearDown]
        public void DestroyObjects()
        {
            Object.DestroyImmediate(droneObject);
            if (wall != null)
                Object.DestroyImmediate(wall);
        }

        static DroneCommand Parse(string line)
        {
            Assert.IsTrue(DroneCommand.TryParse(line, out DroneCommand command, out string error), error);
            return command;
        }

        /// <summary>Send a command and fly it to the end, as the link does frame by frame.</summary>
        string Fly(string line)
        {
            string reply = drone.Begin(Parse(line));
            if (reply != null)
                return reply;
            for (int i = 0; i < 10000; i++)
            {
                if (drone.Step(0.1f, out reply))
                    return reply;
            }
            Assert.Fail($"{line} never finished");
            return null;
        }

        [TestCase("forward 19", "error out of range")]
        [TestCase("forward 501", "error out of range")]
        [TestCase("cw 0", "error out of range")]
        [TestCase("ccw 361", "error out of range")]
        [TestCase("flip l", "error unknown command")]
        [TestCase("forward", "error unknown command")]
        [TestCase("forward ten", "error unknown command")]
        public void CommandsOutsideTheContractAreRejectedWithItsReplies(string line, string expected)
        {
            Assert.IsFalse(DroneCommand.TryParse(line, out _, out string error));
            Assert.AreEqual(expected, error);
        }

        [Test]
        public void MovesBeforeTakeOffAreRefused()
        {
            Assert.AreEqual("error not flying", Fly("forward 100"));
            Assert.AreEqual("error not flying", Fly("cw 90"));
            Assert.AreEqual("error not flying", Fly("land"));
        }

        [Test]
        public void TakeOffRisesToTheContractsHeightAndLandingComesBackDown()
        {
            Assert.AreEqual("ok", Fly("takeoff"));
            Assert.AreEqual(ClassroomLayout.TakeOffHeightCm, drone.PositionCm.z, 1e-3f);
            Assert.AreEqual("ok", Fly("land"));
            Assert.AreEqual(0f, drone.PositionCm.z, 1e-3f);
            Assert.IsFalse(drone.Flying);
        }

        [Test]
        public void TheReplyOnlyComesWhenTheMoveHasEndedAtTheRealPace()
        {
            Fly("takeoff");
            Assert.IsNull(drone.Begin(Parse("forward 100")));
            // 100 cm at 50 cm/s: still flying after 1.9 s, done at 2 s.
            Assert.IsFalse(drone.Step(1.9f, out _));
            Assert.IsTrue(drone.Step(0.1f, out string reply));
            Assert.AreEqual("ok", reply);
        }

        [Test]
        public void MovesFollowTheHeadingClockwiseLikeCw()
        {
            Fly("takeoff");
            Fly("cw 90");
            Fly("forward 100");
            Vector2 pad = ClassroomLayout.TakeOffPadCm;
            // Heading 90 points along the room's +x.
            Assert.AreEqual(pad.x + 100f, drone.PositionCm.x, 1e-2f);
            Assert.AreEqual(pad.y, drone.PositionCm.y, 1e-2f);
            Fly("left 50");
            Assert.AreEqual(pad.y + 50f, drone.PositionCm.y, 1e-2f);
        }

        [Test]
        public void TheStateReportsYawFromMinus180To180AndTheHeightAsTof()
        {
            Fly("takeoff");
            Fly("cw 270");
            Assert.AreEqual(-90f, drone.YawDeg, 1e-3f);
            Assert.AreEqual("pitch:0;roll:0;yaw:-90;tof:80;bat:100;", drone.StateLine());
        }

        [Test]
        public void AMoveIntoAWallStopsJustBeforeItAndRepliesCollision()
        {
            Vector2 pad = ClassroomLayout.TakeOffPadCm;
            // A wall across the drone's way, its near face 150 cm ahead of the pad.
            wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.transform.position = ClassroomLayout.ToUnity(pad.x, pad.y + 160f, 100f);
            wall.transform.localScale = new Vector3(4f, 2f, 0.2f);

            Fly("takeoff");
            Assert.AreEqual("error collision", Fly("forward 300"));
            // The sphere's front touches the face 150 cm ahead, less the 1 cm gap kept from it.
            Assert.AreEqual(pad.y + 150f - SimulatedDrone.RadiusCm - 1f, drone.PositionCm.y, 0.5f);
            // Touching it, the next move towards it is a collision at once, not a way through.
            Assert.AreEqual("error collision", Fly("forward 20"));
            Assert.AreEqual("ok", Fly("back 50"));
        }

        [Test]
        public void ResettingStartsANewFlightLandedOnThePad()
        {
            Fly("takeoff");
            Fly("cw 45");
            Fly("forward 100");
            drone.ResetToPad();
            Vector2 pad = ClassroomLayout.TakeOffPadCm;
            Assert.AreEqual(new Vector3(pad.x, pad.y, 0f), drone.PositionCm);
            Assert.AreEqual(0f, drone.YawDeg);
            Assert.IsFalse(drone.Flying);
        }

        [Test]
        public void APhotoReplyIsItsSizeLineThenExactlyThatManyJpegBytes()
        {
            var cameraObject = new GameObject("Tello Camera");
            cameraObject.transform.SetParent(droneObject.transform, false);
            cameraObject.AddComponent<Camera>();
            var telloCamera = cameraObject.AddComponent<TelloCamera>();
            var link = droneObject.AddComponent<SimulatorLink>();
            var settings = new UnityEditor.SerializedObject(link);
            settings.FindProperty("telloCamera").objectReferenceValue = telloCamera;
            settings.ApplyModifiedPropertiesWithoutUndo();

            byte[] reply = link.Handle("photo");
            int newline = System.Array.IndexOf(reply, (byte)'\n');
            string header = Encoding.ASCII.GetString(reply, 0, newline);
            StringAssert.StartsWith("photo ", header);
            int size = int.Parse(header.Substring(6));
            Assert.AreEqual(reply.Length - newline - 1, size);
            // A JPEG starts with FF D8.
            Assert.AreEqual(0xFF, reply[newline + 1]);
            Assert.AreEqual(0xD8, reply[newline + 2]);

            StringAssert.StartsWith("pitch:0;", Encoding.ASCII.GetString(link.Handle("state?")));
            Assert.AreEqual("error unknown command\n", Encoding.ASCII.GetString(link.Handle("flip l")));
        }

        [Test]
        public void TheServerRepliesToTheCurrentConnectionAndTagsLinesWithIt()
        {
            using var server = new LineServer(IPAddress.Loopback, 0);
            server.Start();

            using var first = new TcpClient();
            first.Connect(server.Endpoint);
            Send(first, "command\n");
            Assert.IsTrue(Next(server, out string line, out int connection));
            Assert.AreEqual(("command", 1), (line, connection));
            Assert.IsTrue(server.Reply(Encoding.ASCII.GetBytes("ok\n")));
            Assert.AreEqual("ok", ReadLine(first));

            using var second = new TcpClient();
            second.Connect(server.Endpoint);
            Send(second, "takeoff\n");
            Assert.IsTrue(Next(server, out line, out connection));
            Assert.AreEqual(("takeoff", 2), (line, connection));
            Assert.IsTrue(server.Reply(Encoding.ASCII.GetBytes("ok\n")));
            Assert.AreEqual("ok", ReadLine(second));
        }

        static void Send(TcpClient client, string text)
        {
            byte[] bytes = Encoding.ASCII.GetBytes(text);
            client.GetStream().Write(bytes, 0, bytes.Length);
        }

        static bool Next(LineServer server, out string line, out int connection)
        {
            var clock = Stopwatch.StartNew();
            while (clock.ElapsedMilliseconds < 2000)
            {
                if (server.TryDequeue(out line, out connection))
                    return true;
            }
            line = null;
            connection = 0;
            return false;
        }

        static string ReadLine(TcpClient client)
        {
            client.ReceiveTimeout = 2000;
            return new StreamReader(client.GetStream(), Encoding.ASCII).ReadLine();
        }
    }
}
