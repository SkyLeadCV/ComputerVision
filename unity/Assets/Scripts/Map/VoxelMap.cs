using System.Collections.Generic;
using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>The excavated block: owns the voxel grid and one mesh per chunk, rebuilt only when it changes.</summary>
    public sealed class VoxelMap : MonoBehaviour
    {
        // Centred on the take-off point and 16 m across, so it holds any flight in the classroom (7.2 m across its
        // diagonal) wherever the drone takes off from and whichever way it faces. A block sized to the simulator's
        // pad and heading dropped 87 % of what the first real explorations saw, which took off elsewhere. Height: the
        // 300 cm ceiling. 160 x 160 x 30 voxels take 0.77 MB, and only the chunks that change are rebuilt.
        [Tooltip("Corner of the block with the lowest x and y, in cm from the take-off point (protocol frame).")]
        [SerializeField] Vector2 originCm = new(-800f, -800f);

        [Tooltip("Block size in 10 cm voxels along x (right), y (forward) and z (up).")]
        [SerializeField] Vector3Int sizeVoxels = new(160, 160, 30);

        [Tooltip("Only what lies below this height is drawn, like a cut-away scale model. It changes no data.")]
        // Above the drone's 80 cm and the tables, so obstacles still show at the height that matters for flying.
        [SerializeField, Min(10f)] float cutHeightCm = 120f;

        [SerializeField] Material solidMaterial;
        [SerializeField] Material surfaceMaterial;
        [SerializeField] Material floorMaterial;

        readonly Dictionary<Vector3Int, Mesh> chunkMeshes = new();
        readonly List<Vector3Int> dirtyChunks = new();
        readonly ChunkMeshData meshData = new();
        const string ChunkPrefix = "Chunk ";

        CameraFov fov;
        string sessionName;
        bool blocksVisible = true;
        bool warnedNoSession;

        public VoxelGrid Grid { get; private set; }

        /// <summary>Rays of this session whose floor ended outside the block, so their far end could not be drawn.</summary>
        public int RaysOutside { get; private set; }

        public Vector2 OriginCm => originCm;

        public Vector3Int SizeVoxels => sizeVoxels;

        /// <summary>Clear the block for a new session; the same session continues it. Returns whether it was new.</summary>
        public bool StartSession(SessionStart session)
        {
            EnsureGrid();
            fov = session.camera;
            // The brain re-sends session_start after every reconnection (docs/protocol.md), and excavation cannot be
            // redone from the lost messages, so only a different session may clear what was carved.
            if (session.session == sessionName)
                return false;
            sessionName = session.session;
            Grid.Clear();
            RaysOutside = 0;
            return true;
        }

        /// <summary>
        /// Excavate what an observation proves, or part of it: with a progress below 1 each ray only reaches that
        /// fraction of the way from its blind zone's edge to its stop, and no obstacle face is painted yet. Emptying
        /// is idempotent, so growing the progress to 1 ends exactly as one full observation does.
        /// </summary>
        public void Observe(Observation observation, float progress)
        {
            if (progress >= 1f)
            {
                Observe(observation);
                return;
            }
            Rays rays = observation.rays;
            var partial = new Rays
            {
                angle_deg = rays.angle_deg,
                entry_cm = rays.entry_cm,
                stop_cm = new float[rays.stop_cm.Length],
                kind = new int[rays.kind.Length],
            };
            for (int i = 0; i < rays.stop_cm.Length; i++)
            {
                partial.stop_cm[i] = Mathf.Lerp(rays.entry_cm[i], rays.stop_cm[i], progress);
                partial.kind[i] = RayKind.TrustedRange;
            }
            Excavate(new Observation { pose = observation.pose, rays = partial });
        }

        public void Observe(Observation observation) => RaysOutside += Excavate(observation);

        /// <summary>Excavate an observation and return how many of its rays ended outside the block.</summary>
        int Excavate(Observation observation)
        {
            // The vertical field of view only arrives in session_start, and without it the fans cannot be drawn.
            if (fov == null)
            {
                if (!warnedNoSession)
                    Debug.LogWarning("Observation received before session_start; ignoring it.");
                warnedNoSession = true;
                return 0;
            }
            return Excavator.Apply(Grid, observation, fov);
        }

        /// <summary>The blocks belong to the 3D view; from above, the floor plan draws the map instead.</summary>
        public void ShowBlocks(bool visible)
        {
            blocksVisible = visible;
            foreach (Transform chunk in transform)
            {
                if (chunk.name.StartsWith(ChunkPrefix))
                    chunk.gameObject.SetActive(visible);
            }
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
            var chunkObject = new GameObject($"{ChunkPrefix}{chunk.x} {chunk.y} {chunk.z}") { hideFlags = HideFlags.DontSave };
            chunkObject.SetActive(blocksVisible);
            chunkObject.transform.SetParent(transform, false);
            mesh = new Mesh { name = chunkObject.name, hideFlags = HideFlags.DontSave };
            chunkObject.AddComponent<MeshFilter>().sharedMesh = mesh;
            chunkObject.AddComponent<MeshRenderer>().sharedMaterials = new[] { solidMaterial, surfaceMaterial, floorMaterial };
            chunkMeshes.Add(chunk, mesh);
            return mesh;
        }
    }
}
