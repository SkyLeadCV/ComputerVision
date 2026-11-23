using System;

namespace SkyLead.Map
{
    // Field names match the JSON keys of docs/protocol.md exactly, because JsonUtility maps fields by name.
    // JsonUtility cannot pick a class from the "type" field, so every line is first read as a MessageHeader
    // and then again as the class its type names.

    [Serializable]
    public class MessageHeader
    {
        public string type;
        public int seq;
        public float t;
    }

    [Serializable]
    public class CameraFov
    {
        public float hfov_deg;
        public float vfov_up_deg;
        public float vfov_down_deg;
    }

    [Serializable]
    public class SessionStart : MessageHeader
    {
        public int version;
        public string session;
        public CameraFov camera;
        public float drone_radius_cm;
        public float safety_margin_cm;
    }

    [Serializable]
    public class Pose
    {
        public float x;
        public float y;
        public float z;
        public float yaw_deg;
        public float pitch_deg;
    }

    [Serializable]
    public class Rays
    {
        public float[] angle_deg;
        public float[] entry_cm;
        public float[] stop_cm;
        public int[] kind;
    }

    [Serializable]
    public class Observation : MessageHeader
    {
        public Pose pose;
        public Rays rays;
    }

    public static class RayKind
    {
        public const int Obstacle = 0;
        public const int ImageBorder = 1;
        public const int TrustedRange = 2;
        public const int Lid = 3;
    }
}
