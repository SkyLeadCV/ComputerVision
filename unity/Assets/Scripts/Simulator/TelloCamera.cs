using UnityEngine;

namespace SkyLead.Simulator
{
    /// <summary>
    /// The Tello's camera as a distortion-free pinhole with the calibrated intrinsics of docs/simulator.md, so a
    /// rendered photo can go through the same vision pipeline as an undistorted real frame.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public class TelloCamera : MonoBehaviour
    {
        public const int WidthPx = 960;
        public const int HeightPx = 720;
        public const float FxPx = 931.5f;
        public const float FyPx = 924.5f;
        public const float CxPx = 441.9f;
        public const float CyPx = 344.2f;
        // Measured in flight: the real camera looks this far below the horizon while the drone hovers.
        public const float TiltDownDeg = 11f;

        const float NearM = 0.05f;
        // Beyond the room's 7.2 m diagonal.
        const float FarM = 20f;
        // Smooths the tile joints' edges as a real lens does; aliased joints break into short Hough segments.
        const int Antialiasing = 4;

        void OnEnable() => Configure();

        /// <summary>Apply the calibrated projection and the camera tilt. Unity does not serialize a custom projection.</summary>
        public void Configure()
        {
            var view = GetComponent<Camera>();
            view.nearClipPlane = NearM;
            view.farClipPlane = FarM;
            view.projectionMatrix = Projection(NearM, FarM);
            transform.localRotation = Quaternion.Euler(TiltDownDeg, 0f, 0f);
        }

        /// <summary>
        /// The OpenGL-style projection of the intrinsic matrix K. The principal point is off the image centre, which
        /// Unity's field-of-view camera cannot express. Image rows grow downwards while view-space y grows upwards.
        /// </summary>
        public static Matrix4x4 Projection(float near, float far)
        {
            var projection = Matrix4x4.zero;
            projection[0, 0] = 2f * FxPx / WidthPx;
            projection[0, 2] = 1f - 2f * CxPx / WidthPx;
            projection[1, 1] = 2f * FyPx / HeightPx;
            projection[1, 2] = 2f * CyPx / HeightPx - 1f;
            projection[2, 2] = -(far + near) / (far - near);
            projection[2, 3] = -2f * far * near / (far - near);
            projection[3, 2] = -1f;
            return projection;
        }

        /// <summary>Render the current view as a JPEG, the format the simulator's photo reply carries.</summary>
        public byte[] CaptureJpeg(int quality)
        {
            Configure();
            var view = GetComponent<Camera>();
            var target = new RenderTexture(WidthPx, HeightPx, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB)
            {
                antiAliasing = Antialiasing,
            };
            RenderTexture previousTarget = view.targetTexture;
            view.targetTexture = target;
            view.Render();

            RenderTexture previousActive = RenderTexture.active;
            RenderTexture.active = target;
            var image = new Texture2D(WidthPx, HeightPx, TextureFormat.RGB24, false);
            image.ReadPixels(new Rect(0, 0, WidthPx, HeightPx), 0, 0);
            RenderTexture.active = previousActive;
            view.targetTexture = previousTarget;

            byte[] jpeg = image.EncodeToJPG(quality);
            DestroyImmediate(image);
            target.Release();
            DestroyImmediate(target);
            return jpeg;
        }
    }
}
