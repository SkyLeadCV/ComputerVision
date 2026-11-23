using NUnit.Framework;
using UnityEngine;

namespace SkyLead.Map.Tests
{
    public class VoxelMapTests
    {
        static readonly CameraFov TelloFov = new() { hfov_deg = 54.5f, vfov_up_deg = 20.4f, vfov_down_deg = 22.1f };

        // A 6 x 6 x 3 m block around the take-off point, so rays can go any way. The test drone hovers at a voxel
        // centre (5, 5): on a voxel boundary, float rounding of sin and cos would put a ray in either neighbour.
        static VoxelGrid NewGrid() => new(new Vector2(-300f, -300f), new Vector3Int(60, 60, 30));

        static Observation SingleRay(float yawDeg, float angleDeg, float entryCm, float stopCm, int kind) => new()
        {
            pose = new Pose { x = 5f, y = 5f, z = 80f, yaw_deg = yawDeg, pitch_deg = 0f },
            rays = new Rays
            {
                angle_deg = new[] { angleDeg }, entry_cm = new[] { entryCm }, stop_cm = new[] { stopCm }, kind = new[] { kind },
            },
        };

        static Voxel At(VoxelGrid grid, float xCm, float yCm, float zCm)
        {
            Assert.IsTrue(grid.TryCell(xCm, yCm, zCm, out Vector3Int cell), $"({xCm}, {yCm}, {zCm}) is outside the grid");
            return grid[cell.x, cell.y, cell.z];
        }

        [Test]
        public void ReadsTheProtocolExampleIncludingTheHeaderFields()
        {
            const string line = "{\"type\": \"observation\", \"seq\": 12, \"t\": 31.4, "
                + "\"pose\": {\"x\": 120.0, \"y\": 340.0, \"z\": 82.0, \"yaw_deg\": 90.0, \"pitch_deg\": -3.1}, "
                + "\"rays\": {\"angle_deg\": [-27.0, -26.5], \"entry_cm\": [148, 147], \"stop_cm\": [212, 215], \"kind\": [0, 0]}}";

            Assert.AreEqual("observation", JsonUtility.FromJson<MessageHeader>(line).type);
            var observation = JsonUtility.FromJson<Observation>(line);
            Assert.AreEqual(12, observation.seq);
            Assert.AreEqual(31.4f, observation.t, 1e-4f);
            Assert.AreEqual(90f, observation.pose.yaw_deg);
            Assert.AreEqual(-3.1f, observation.pose.pitch_deg, 1e-4f);
            CollectionAssert.AreEqual(new[] { 212f, 215f }, observation.rays.stop_cm);
            CollectionAssert.AreEqual(new[] { 0, 0 }, observation.rays.kind);
        }

        [Test]
        public void ObstacleRayEmptiesTheFloorUpToItsStopAndPaintsTheFaceBehindIt()
        {
            VoxelGrid grid = NewGrid();
            Excavator.Apply(grid, SingleRay(0f, 0f, 200f, 250f, RayKind.Obstacle), TelloFov);

            Assert.AreEqual(Voxel.Empty, At(grid, 5f, 245f, 5f));
            Assert.AreEqual(Voxel.Surface, At(grid, 5f, 265f, 5f));
            Assert.AreEqual(Voxel.Surface, At(grid, 5f, 265f, 150f));
            Assert.AreEqual(Voxel.Solid, At(grid, 5f, 285f, 5f));
        }

        [Test]
        public void TrustedRangeRayEmptiesAirWithoutPaintingASurface()
        {
            VoxelGrid grid = NewGrid();
            Excavator.Apply(grid, SingleRay(0f, 0f, 200f, 250f, RayKind.TrustedRange), TelloFov);

            Assert.AreEqual(Voxel.Empty, At(grid, 5f, 245f, 5f));
            Assert.AreEqual(Voxel.Solid, At(grid, 5f, 265f, 5f));
            for (int x = 0; x < grid.Size.x; x++)
            for (int y = 0; y < grid.Size.y; y++)
            for (int z = 0; z < grid.Size.z; z++)
                Assert.AreNotEqual(Voxel.Surface, grid[x, y, z]);
        }

        [Test]
        public void OnlyTheVerticalFieldOfViewIsEmptied()
        {
            VoxelGrid grid = NewGrid();
            Excavator.Apply(grid, SingleRay(0f, 0f, 200f, 250f, RayKind.TrustedRange), TelloFov);

            // At 1 m the view spans 80 - 100 tan 22.1 = 39 cm up to 80 + 100 tan 20.4 = 117 cm.
            Assert.AreEqual(Voxel.Solid, At(grid, 5f, 105f, 25f));
            Assert.AreEqual(Voxel.Empty, At(grid, 5f, 105f, 85f));
            Assert.AreEqual(Voxel.Solid, At(grid, 5f, 105f, 135f));
            // The voxels from 30 to 40 cm and from 110 to 120 cm are only partly in view, so nothing proves them empty.
            Assert.AreEqual(Voxel.Solid, At(grid, 5f, 105f, 35f));
            Assert.AreEqual(Voxel.Solid, At(grid, 5f, 105f, 115f));
        }

        [Test]
        public void YawTurnsRaysClockwiseLikeTheCwCommand()
        {
            VoxelGrid grid = NewGrid();
            Excavator.Apply(grid, SingleRay(90f, 0f, 200f, 250f, RayKind.Obstacle), TelloFov);

            Assert.AreEqual(Voxel.Empty, At(grid, 245f, 5f, 5f));
            Assert.AreEqual(Voxel.Surface, At(grid, 265f, 5f, 5f));
            Assert.AreEqual(Voxel.Solid, At(grid, -235f, 5f, 5f));
        }

        [Test]
        public void ALaterViewThroughAFalseSurfaceEmptiesIt()
        {
            VoxelGrid grid = NewGrid();
            Excavator.Apply(grid, SingleRay(0f, 0f, 200f, 250f, RayKind.Obstacle), TelloFov);
            Excavator.Apply(grid, SingleRay(0f, 0f, 200f, 270f, RayKind.Obstacle), TelloFov);

            Assert.AreEqual(Voxel.Empty, At(grid, 5f, 265f, 5f));
            Assert.AreEqual(Voxel.Surface, At(grid, 5f, 285f, 5f));
        }

        [Test]
        public void ProtocolCentimetresBecomeUnityMetresWithYUp()
        {
            Assert.Less(Vector3.Distance(new Vector3(1.2f, 0.82f, 3.4f), VoxelGrid.ToUnity(120f, 340f, 82f)), 1e-6f);
        }

        [Test]
        public void EveryFaceIsDrawnTowardsTheAirItBorders()
        {
            VoxelGrid grid = NewGrid();
            Excavator.Apply(grid, SingleRay(0f, 0f, 200f, 250f, RayKind.Obstacle), TelloFov);
            var data = new ChunkMeshData();

            for (int x = 0; x < grid.ChunkCount.x; x++)
            for (int y = 0; y < grid.ChunkCount.y; y++)
            for (int z = 0; z < grid.ChunkCount.z; z++)
            {
                ChunkMesher.Build(grid, new Vector3Int(x, y, z), grid.Size.z, data);
                foreach (var triangles in data.Triangles)
                {
                    for (int i = 0; i < triangles.Count; i += 3)
                    {
                        Vector3 a = data.Vertices[triangles[i]], b = data.Vertices[triangles[i + 1]], c = data.Vertices[triangles[i + 2]];
                        // Unity draws the side that Cross(b - a, c - a) points to.
                        Vector3 drawnSide = Vector3.Cross(b - a, c - a).normalized;
                        Vector3 normal = data.Normals[triangles[i]];
                        Assert.Greater(Vector3.Dot(drawnSide, normal), 0.99f);

                        // A centimetre past the face centre, along its normal, lies in the air the face borders.
                        Vector3 faceCentre = (a + c) / 2f;
                        Vector3 inAir = faceCentre + normal * 0.01f;
                        var cm = new Vector3(inAir.x, inAir.z, inAir.y) * 100f;
                        Assert.IsTrue(grid.TryCell(cm.x, cm.y, cm.z, out Vector3Int cell));
                        Assert.AreEqual(Voxel.Empty, grid[cell.x, cell.y, cell.z], $"The face at {faceCentre} does not face air");
                    }
                }
            }
        }

        [Test]
        public void NothingAboveTheCutHeightIsDrawn()
        {
            VoxelGrid grid = NewGrid();
            // The obstacle face at 2.5 m reaches 80 + 255 tan 20.4 = 175 cm, well above an 80 cm cut.
            Excavator.Apply(grid, SingleRay(0f, 0f, 200f, 250f, RayKind.Obstacle), TelloFov);
            var data = new ChunkMeshData();
            int drawnFaces = 0;

            for (int x = 0; x < grid.ChunkCount.x; x++)
            for (int y = 0; y < grid.ChunkCount.y; y++)
            for (int z = 0; z < grid.ChunkCount.z; z++)
            {
                ChunkMesher.Build(grid, new Vector3Int(x, y, z), 8, data);
                drawnFaces += data.Vertices.Count / 4;
                foreach (Vector3 vertex in data.Vertices)
                    Assert.LessOrEqual(vertex.y, 0.8f + 1e-5f);
            }
            Assert.Greater(drawnFaces, 0);
        }
    }
}
