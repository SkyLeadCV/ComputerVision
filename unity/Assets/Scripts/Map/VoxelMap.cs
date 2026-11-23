using System.Collections.Generic;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>The excavated block: owns the voxel grid and one mesh per chunk, rebuilt only when it changes.</summary>
    public sealed class VoxelMap : MonoBehaviour
    {
        [Tooltip("Corner of the block with the lowest x and y, in cm from the take-off point (protocol frame).")]
        [SerializeField] Vector2 originCm = new(-200f, -100f);

        [Tooltip("Block size in 10 cm voxels along x (right), y (forward) and z (up).")]
        [SerializeField] Vector3Int sizeVoxels = new(100, 80, 30);

        [Tooltip("Only what lies below this height is drawn, like a cut-away scale model. It changes no data.")]
        // Above the drone's 80 cm and the tables, so obstacles still show at the height that matters for flying.
        [SerializeField, Min(10f)] float cutHeightCm = 120f;

        [SerializeField] Material solidMaterial;
        [SerializeField] Material surfaceMaterial;
        [SerializeField] Material floorMaterial;

        readonly Dictionary<Vector3Int, Mesh> chunkMeshes = new();
        readonly List<Vector3Int> dirtyChunks = new();
        readonly ChunkMeshData meshData = new();
        CameraFov fov;
        bool warnedNoSession;

        public VoxelGrid Grid { get; private set; }

        public void StartSession(SessionStart session)
        {
            EnsureGrid();
            Grid.Clear();
            fov = session.camera;
        }

        public void Observe(Observation observation)
        {
            // The vertical field of view only arrives in session_start, and without it the fans cannot be drawn.
            if (fov == null)
            {
                if (!warnedNoSession)
                    Debug.LogWarning("Observation received before session_start; ignoring it.");
                warnedNoSession = true;
                return;
            }
            Excavator.Apply(Grid, observation, fov);
        }

        public void RebuildDirtyChunks()
        {
            EnsureGrid();
            Grid.TakeDirtyChunks(dirtyChunks);
            foreach (Vector3Int chunk in dirtyChunks)
            {
                ChunkMesher.Build(Grid, chunk, Mathf.RoundToInt(cutHeightCm / VoxelGrid.VoxelCm), meshData);
                ChunkMesher.ToMesh(meshData, ChunkMesh(chunk));
            }
        }

        void Awake() => EnsureGrid();

        void LateUpdate() => RebuildDirtyChunks();

        void EnsureGrid() => Grid ??= new VoxelGrid(originCm, sizeVoxels);

        Mesh ChunkMesh(Vector3Int chunk)
        {
            if (chunkMeshes.TryGetValue(chunk, out Mesh mesh))
                return mesh;

            // Chunks are rebuilt from the grid on every run, so they are never saved with the scene.
            var chunkObject = new GameObject($"Chunk {chunk.x} {chunk.y} {chunk.z}") { hideFlags = HideFlags.DontSave };
            chunkObject.transform.SetParent(transform, false);
            mesh = new Mesh { name = chunkObject.name, hideFlags = HideFlags.DontSave };
            chunkObject.AddComponent<MeshFilter>().sharedMesh = mesh;
            chunkObject.AddComponent<MeshRenderer>().sharedMaterials = new[] { solidMaterial, surfaceMaterial, floorMaterial };
            chunkMeshes.Add(chunk, mesh);
            return mesh;
        }
    }
}
