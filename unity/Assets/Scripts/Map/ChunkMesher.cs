using System.Collections.Generic;
using UnityEngine;

namespace SkyLead.Map
{
    public sealed class ChunkMeshData
    {
        public readonly List<Vector3> Vertices = new();
        public readonly List<Vector3> Normals = new();
        public readonly List<int>[] Triangles = { new(), new(), new() };

        public void Clear()
        {
            Vertices.Clear();
            Normals.Clear();
            foreach (List<int> submesh in Triangles)
                submesh.Clear();
        }
    }

    /// <summary>
    /// Builds one chunk's mesh with only the faces that border excavated air, and nothing above a cut height.
    /// The camera never sees above the top of its view, so the air there stays solid and steps up with
    /// distance; drawn, those steps would hide the room like a ceiling. Cut like a scale model, the obstacle
    /// faces and the seen floor show, and floor that was never seen stays undrawn.
    /// </summary>
    public static class ChunkMesher
    {
        public const int SolidSubmesh = 0;
        public const int SurfaceSubmesh = 1;
        public const int FloorSubmesh = 2;

        static readonly float VoxelM = VoxelGrid.VoxelCm / 100f;

        // Neighbour step in the protocol's (x, y, z) and the matching Unity normal, which swaps y and z.
        static readonly (Vector3Int step, Vector3 normal, Vector3 tangent)[] Faces =
        {
            (new Vector3Int(1, 0, 0), Vector3.right, Vector3.up),
            (new Vector3Int(-1, 0, 0), Vector3.left, Vector3.up),
            (new Vector3Int(0, 1, 0), Vector3.forward, Vector3.up),
            (new Vector3Int(0, -1, 0), Vector3.back, Vector3.up),
            (new Vector3Int(0, 0, 1), Vector3.up, Vector3.right),
            (new Vector3Int(0, 0, -1), Vector3.down, Vector3.right),
        };

        /// <param name="cutLayers">Voxel layers drawn from the floor up; the ones above are left out.</param>
        public static void Build(VoxelGrid grid, Vector3Int chunk, int cutLayers, ChunkMeshData data)
        {
            data.Clear();
            Vector3Int start = chunk * VoxelGrid.ChunkSize;
            Vector3Int end = Vector3Int.Min(start + Vector3Int.one * VoxelGrid.ChunkSize, grid.Size);
            end.z = Mathf.Min(end.z, cutLayers);
            Vector3 halfVoxel = Vector3.one * (VoxelM / 2f);

            for (int z = start.z; z < end.z; z++)
            for (int y = start.y; y < end.y; y++)
            for (int x = start.x; x < end.x; x++)
            {
                Voxel voxel = grid[x, y, z];
                Vector3 centre = grid.CellCornerToUnity(x, y, z) + halfVoxel;
                if (voxel == Voxel.Empty)
                {
                    if (z == 0)
                        AddQuad(data, FloorSubmesh, centre + Vector3.down * (VoxelM / 2f), Vector3.up, Vector3.right);
                    continue;
                }

                int submesh = voxel == Voxel.Surface ? SurfaceSubmesh : SolidSubmesh;
                foreach (var (step, normal, tangent) in Faces)
                {
                    int nx = x + step.x, ny = y + step.y, nz = z + step.z;
                    if (grid.Contains(nx, ny, nz) && grid[nx, ny, nz] == Voxel.Empty)
                        AddQuad(data, submesh, centre + normal * (VoxelM / 2f), normal, tangent);
                }
            }
        }

        public static void ToMesh(ChunkMeshData data, Mesh mesh)
        {
            mesh.Clear();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh.SetVertices(data.Vertices);
            mesh.SetNormals(data.Normals);
            mesh.subMeshCount = data.Triangles.Length;
            for (int submesh = 0; submesh < data.Triangles.Length; submesh++)
                mesh.SetTriangles(data.Triangles[submesh], submesh, false);
            mesh.RecalculateBounds();
        }

        static void AddQuad(ChunkMeshData data, int submesh, Vector3 centre, Vector3 normal, Vector3 u)
        {
            // Cross(u, v) equals the normal, so triangles (0, 1, 2) and (0, 2, 3) wind clockwise seen from the
            // normal's side, which is the side Unity draws.
            Vector3 v = Vector3.Cross(normal, u);
            Vector3 corner = centre - (u + v) * (VoxelM / 2f);
            int first = data.Vertices.Count;
            data.Vertices.Add(corner);
            data.Vertices.Add(corner + u * VoxelM);
            data.Vertices.Add(corner + (u + v) * VoxelM);
            data.Vertices.Add(corner + v * VoxelM);
            for (int i = 0; i < 4; i++)
                data.Normals.Add(normal);
            data.Triangles[submesh].AddRange(new[] { first, first + 1, first + 2, first, first + 2, first + 3 });
        }
    }
}
