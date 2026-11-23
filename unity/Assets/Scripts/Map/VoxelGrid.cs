using System;
using System.Collections.Generic;
using UnityEngine;

namespace SkyLead.Map
{
    public enum Voxel : byte
    {
        // Zero, so a new grid is the solid block that the drone has to prove empty.
        Solid = 0,
        Empty = 1,
        Surface = 2,
    }

    /// <summary>
    /// The classroom as 10 cm voxels, indexed in the protocol frame: x right, y forward, z up, in cm
    /// from the take-off point. One byte per voxel keeps even a 10 x 8 x 3 m room (240 000 voxels) in 240 kB.
    /// </summary>
    public sealed class VoxelGrid
    {
        public const float VoxelCm = 10f;
        // 16 voxels a side keeps each chunk's mesh small enough to rebuild within a frame.
        public const int ChunkSize = 16;

        public readonly Vector2 OriginCm;
        public readonly Vector3Int Size;
        public readonly Vector3Int ChunkCount;

        readonly Voxel[] cells;
        readonly HashSet<Vector3Int> dirtyChunks = new();

        public VoxelGrid(Vector2 originCm, Vector3Int size)
        {
            if (size.x <= 0 || size.y <= 0 || size.z <= 0)
                throw new ArgumentException($"Grid size must be positive, got {size}");
            OriginCm = originCm;
            Size = size;
            ChunkCount = new Vector3Int(
                (size.x + ChunkSize - 1) / ChunkSize, (size.y + ChunkSize - 1) / ChunkSize, (size.z + ChunkSize - 1) / ChunkSize);
            cells = new Voxel[size.x * size.y * size.z];
            MarkAllDirty();
        }

        public Voxel this[int x, int y, int z] => cells[Index(x, y, z)];

        public bool Contains(int x, int y, int z) => (uint)x < (uint)Size.x && (uint)y < (uint)Size.y && (uint)z < (uint)Size.z;

        public bool TryCell(float xCm, float yCm, float zCm, out Vector3Int cell)
        {
            cell = new Vector3Int(
                Mathf.FloorToInt((xCm - OriginCm.x) / VoxelCm), Mathf.FloorToInt((yCm - OriginCm.y) / VoxelCm), Mathf.FloorToInt(zCm / VoxelCm));
            return Contains(cell.x, cell.y, cell.z);
        }

        public void Set(Vector3Int cell, Voxel value)
        {
            int index = Index(cell.x, cell.y, cell.z);
            if (cells[index] == value)
                return;
            cells[index] = value;
            MarkDirty(cell);
        }

        public void Clear()
        {
            Array.Clear(cells, 0, cells.Length);
            MarkAllDirty();
        }

        /// <summary>Move the chunks changed since the last call into <paramref name="chunks"/>.</summary>
        public void TakeDirtyChunks(List<Vector3Int> chunks)
        {
            chunks.Clear();
            chunks.AddRange(dirtyChunks);
            dirtyChunks.Clear();
        }

        /// <summary>The protocol's (x, y, z) in cm as a Unity position: y up and metres (docs/protocol.md, section 2).</summary>
        public static Vector3 ToUnity(float xCm, float yCm, float zCm) => new Vector3(xCm, zCm, yCm) / 100f;

        public Vector3 CellCornerToUnity(int x, int y, int z) =>
            ToUnity(OriginCm.x + x * VoxelCm, OriginCm.y + y * VoxelCm, z * VoxelCm);

        int Index(int x, int y, int z) => (z * Size.y + y) * Size.x + x;

        void MarkDirty(Vector3Int cell)
        {
            // A voxel on a chunk's border also decides the faces drawn by the neighbouring chunk.
            for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++)
            {
                var neighbour = new Vector3Int(cell.x + dx, cell.y + dy, cell.z + dz);
                if (Math.Abs(dx) + Math.Abs(dy) + Math.Abs(dz) <= 1 && Contains(neighbour.x, neighbour.y, neighbour.z))
                    dirtyChunks.Add(new Vector3Int(neighbour.x / ChunkSize, neighbour.y / ChunkSize, neighbour.z / ChunkSize));
            }
        }

        void MarkAllDirty()
        {
            for (int x = 0; x < ChunkCount.x; x++)
            for (int y = 0; y < ChunkCount.y; y++)
            for (int z = 0; z < ChunkCount.z; z++)
                dirtyChunks.Add(new Vector3Int(x, y, z));
        }
    }
}
