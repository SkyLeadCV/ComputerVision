using UnityEngine;

namespace SkyLead.Map
{
    /// <summary>Empties the voxels one observation proves to be air, and marks the obstacle faces it saw.</summary>
    public static class Excavator
    {
        // Half a voxel, so a ray that crosses a voxel near its corner still empties it.
        const float RayStepCm = VoxelGrid.VoxelCm / 2f;

        public static void Apply(VoxelGrid grid, Observation observation, CameraFov camera)
        {
            Pose pose = observation.pose;
            Rays rays = observation.rays;
            // pitch_deg is positive nose up, so it lowers the bottom edge of the view and raises the top one.
            float tanDown = Mathf.Tan((camera.vfov_down_deg - pose.pitch_deg) * Mathf.Deg2Rad);
            float tanUp = Mathf.Tan((camera.vfov_up_deg + pose.pitch_deg) * Mathf.Deg2Rad);

            // Every ray is emptied before any surface is painted: a ray grazing a wall would otherwise
            // erase the surface that a neighbouring ray of the same photo has just found.
            for (int i = 0; i < rays.angle_deg.Length; i++)
                EmptyAir(grid, pose, Direction(pose, rays.angle_deg[i]), rays.stop_cm[i], tanDown, tanUp);

            for (int i = 0; i < rays.angle_deg.Length; i++)
            {
                if (rays.kind[i] != RayKind.Obstacle)
                    continue;
                // The voxel just past the last free point is the obstacle's face. Obstacles are assumed to
                // stand vertically on the floor (stixels), so the face runs from the floor to the top of the view.
                // Any voxel the face shows in is painted, since part of it was seen to be an obstacle.
                float distance = rays.stop_cm[i] + RayStepCm;
                Vector2 direction = Direction(pose, rays.angle_deg[i]);
                int top = Mathf.FloorToInt((pose.z + distance * tanUp) / VoxelGrid.VoxelCm);
                SetColumn(grid, pose.x + distance * direction.x, pose.y + distance * direction.y, 0, top, Voxel.Surface);
            }
        }

        /// <summary>Unit floor direction of a ray: its heading is clockwise from +y, as yaw_deg is.</summary>
        static Vector2 Direction(Pose pose, float angleDeg)
        {
            float heading = (pose.yaw_deg + angleDeg) * Mathf.Deg2Rad;
            return new Vector2(Mathf.Sin(heading), Mathf.Cos(heading));
        }

        static void EmptyAir(VoxelGrid grid, Pose pose, Vector2 direction, float stopCm, float tanDown, float tanUp)
        {
            int steps = Mathf.CeilToInt(stopCm / RayStepCm);
            for (int step = 0; step <= steps; step++)
            {
                float distance = Mathf.Min(step * RayStepCm, stopCm);
                float low = Mathf.Max(0f, pose.z - distance * tanDown);
                // Only voxels wholly inside the view are proven to be air; a partly seen one stays solid.
                int bottom = Mathf.CeilToInt(low / VoxelGrid.VoxelCm);
                int top = Mathf.FloorToInt((pose.z + distance * tanUp) / VoxelGrid.VoxelCm) - 1;
                SetColumn(grid, pose.x + distance * direction.x, pose.y + distance * direction.y, bottom, top, Voxel.Empty);
            }
        }

        static void SetColumn(VoxelGrid grid, float xCm, float yCm, int bottom, int top, Voxel value)
        {
            if (!grid.TryCell(xCm, yCm, 0f, out Vector3Int cell))
                return;
            for (int z = Mathf.Max(0, bottom); z <= Mathf.Min(grid.Size.z - 1, top); z++)
                grid.Set(new Vector3Int(cell.x, cell.y, z), value);
        }
    }
}
