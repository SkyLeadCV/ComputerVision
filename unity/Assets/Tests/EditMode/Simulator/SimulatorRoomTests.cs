using NUnit.Framework;
using UnityEngine;

namespace SkyLead.Simulator.Tests
{
    public class SimulatorRoomTests
    {
        const float FlightHeightCm = ClassroomLayout.TakeOffHeightCm;

        GameObject room;
        GameObject cameraObject;

        [TearDown]
        public void TearDown()
        {
            if (room != null) Object.DestroyImmediate(room);
            if (cameraObject != null) Object.DestroyImmediate(cameraObject);
        }

        [Test]
        public void FreeFloorHasTheMeasuredArea()
        {
            Vector2[] polygon = ClassroomLayout.FreeFloorCm;
            float twiceArea = 0f;
            for (int i = 0; i < polygon.Length; i++)
            {
                Vector2 a = polygon[i], b = polygon[(i + 1) % polygon.Length];
                twiceArea += a.x * b.y - b.x * a.y;
            }
            // docs/classroom.md: 18.01 m², counter-clockwise.
            Assert.AreEqual(180100f, twiceArea / 2f, 1f);
        }

        [Test]
        public void ProjectionPutsPointsWhereTheIntrinsicsSay()
        {
            // A point 0.5 m right, 0.3 m down and 2 m ahead of the camera, in Unity's view space (looking down -z).
            var viewPoint = new Vector4(0.5f, -0.3f, -2f, 1f);
            Vector4 clip = TelloCamera.Projection(0.05f, 20f) * viewPoint;
            float u = (clip.x / clip.w + 1f) / 2f * TelloCamera.WidthPx;
            float v = (1f - clip.y / clip.w) / 2f * TelloCamera.HeightPx;

            Assert.AreEqual(TelloCamera.FxPx * 0.5f / 2f + TelloCamera.CxPx, u, 0.01f);
            Assert.AreEqual(TelloCamera.FyPx * 0.3f / 2f + TelloCamera.CyPx, v, 0.01f);
        }

        [Test]
        public void CameraRendersWithTheIntrinsicsAndTheMeasuredTilt()
        {
            cameraObject = new GameObject("Tello Camera");
            var view = cameraObject.AddComponent<Camera>();
            view.targetTexture = new RenderTexture(TelloCamera.WidthPx, TelloCamera.HeightPx, 24);
            cameraObject.AddComponent<TelloCamera>().Configure();

            float elevationDeg = Mathf.Asin(cameraObject.transform.forward.y) * Mathf.Rad2Deg;
            Assert.AreEqual(-TelloCamera.TiltDownDeg, elevationDeg, 0.01f);

            // Unity's screen rows grow upwards; the principal point must land on (cx, cy) of the image.
            Vector3 screen = view.WorldToScreenPoint(cameraObject.transform.forward * 3f);
            Assert.AreEqual(TelloCamera.CxPx, screen.x, 0.05f);
            Assert.AreEqual(TelloCamera.CyPx, TelloCamera.HeightPx - screen.y, 0.05f);

            view.targetTexture.Release();
        }

        [Test]
        public void WallsAndTablesStopTheDroneWhereThePlanSays()
        {
            room = ClassroomBuilder.Build(default);
            Physics.SyncTransforms();
            Vector2 pad = ClassroomLayout.TakeOffPadCm;

            Assert.AreEqual(500f, FlightDistanceCm(pad, 0f), 0.5f, "forward to the top wall");
            Assert.AreEqual(60f, FlightDistanceCm(pad, 90f), 0.5f, "right to the tables block");
            Assert.AreEqual(60f, FlightDistanceCm(pad, 270f), 0.5f, "left to the wall");
            // The rescuer waits right behind the pad, before the wall.
            Assert.Less(FlightDistanceCm(pad, 180f), 59f, "back into the rescuer");
            // The right arm's lane: from its middle, up the arm and past the tables into the top band.
            Assert.AreEqual(559.5f, FlightDistanceCm(new Vector2(415f, 0.5f), 0f), 0.5f, "up the right arm");
        }

        [Test]
        public void TheDroneFliesOverTheLyingVictim()
        {
            room = ClassroomBuilder.Build(default);
            Physics.SyncTransforms();
            Vector2 victim = ClassroomLayout.Victim.PositionCm;

            Vector2 below = victim - new Vector2(0f, 100f);
            Assert.AreEqual(ClassroomLayout.DepthCm - below.y, FlightDistanceCm(below, 0f), 0.5f, "over the victim");
            Assert.IsTrue(Physics.Raycast(ClassroomLayout.ToUnity(victim.x, victim.y, FlightHeightCm), Vector3.down, out RaycastHit hit));
            Assert.AreEqual("Victim", PersonOf(hit.collider.transform).name);
            Assert.Less(hit.point.y * 100f, 30f, "lying low");
        }

        [Test]
        public void TheRescuerWaitsBehindTheTakeOffPad()
        {
            room = ClassroomBuilder.Build(default);
            float padY = ClassroomLayout.TakeOffPadCm.y;
            foreach (Renderer part in room.transform.Find("Rescuer").GetComponentsInChildren<Renderer>())
                Assert.Less(part.bounds.max.z * 100f, padY, $"{part.name} behind the drone, out of the camera's view");
        }

        [Test]
        public void PeopleLieOnTheFreeFloor()
        {
            room = ClassroomBuilder.Build(default);
            foreach (Renderer part in room.GetComponentsInChildren<Renderer>())
            {
                if (part.transform.parent == room.transform)
                    continue;
                Transform person = PersonOf(part.transform);
                Bounds bounds = part.bounds;
                Vector2 minCm = new Vector2(bounds.min.x, bounds.min.z) * 100f, maxCm = new Vector2(bounds.max.x, bounds.max.z) * 100f;
                Assert.IsTrue(minCm.x >= 0f && minCm.y >= 0f && maxCm.x <= ClassroomLayout.WidthCm && maxCm.y <= ClassroomLayout.DepthCm,
                    $"{person.name} {part.name} inside the room");
                bool overTables = minCm.x < ClassroomLayout.TablesMaxCm.x && maxCm.x > ClassroomLayout.TablesMinCm.x
                    && minCm.y < ClassroomLayout.TablesMaxCm.y && maxCm.y > ClassroomLayout.TablesMinCm.y;
                Assert.IsFalse(overTables, $"{person.name} {part.name} clear of the tables block");
            }
        }

        /// <summary>The person a body part belongs to: the classroom's child above it.</summary>
        Transform PersonOf(Transform part)
        {
            while (part.parent != room.transform)
                part = part.parent;
            return part;
        }

        /// <summary>How far the drone flies at flight height from a room point along a heading before it hits something.</summary>
        static float FlightDistanceCm(Vector2 fromCm, float headingDeg)
        {
            Vector3 origin = ClassroomLayout.ToUnity(fromCm.x, fromCm.y, FlightHeightCm);
            Vector3 direction = ClassroomLayout.HeadingToUnity(headingDeg) * Vector3.forward;
            return Physics.Raycast(origin, direction, out RaycastHit hit, 100f) ? hit.distance * 100f : float.PositiveInfinity;
        }
    }
}
