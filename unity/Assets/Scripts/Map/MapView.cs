using UnityEngine;
using UnityEngine.InputSystem;

namespace SkyLead.Map
{
    /// <summary>
    /// The map's camera: straight from above by default, which reads like a plan, or the excavated block in 3D. A key
    /// switches between them with a smooth flight of the camera, and each view shows its own layers: the clean floor
    /// plan from above, the voxels in 3D.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public sealed class MapView : MonoBehaviour
    {
        [SerializeField] VoxelMap map;
        [SerializeField] FloorPlan plan;

        [Tooltip("Switches between the plan from above and the 3D view.")]
        [SerializeField] Key toggleKey = Key.V;

        // A narrow lens from far away, so the view from above is nearly free of perspective, like a plan.
        public const float TopFovDeg = 20f;
        public const float ThreeDFovDeg = 38f;
        // Room around the block from above, so the HUD panels do not cover its edges.
        const float TopMarginM = 1.4f;
        static readonly Vector3 ThreeDOffset = new(0f, 2.6f, -2.1f);
        const float TransitionS = 0.8f;

        (Vector3 position, Quaternion rotation) top, threeD;
        bool showingTop = true;
        float blend;

        public bool ShowingTop => showingTop;

        void Start()
        {
            Frame(map, out top, out threeD);
            blend = 0f;
            Apply(0f);
            ShowLayers(true);
        }

        /// <summary>The two camera poses that fit the block: straight down from above, and above and behind its start.</summary>
        public static void Frame(VoxelMap map, out (Vector3 position, Quaternion rotation) top,
            out (Vector3 position, Quaternion rotation) threeD)
        {
            Vector3 size = new Vector3(map.SizeVoxels.x, map.SizeVoxels.z, map.SizeVoxels.y) * VoxelGrid.VoxelCm / 100f;
            Vector3 centre = VoxelGrid.ToUnity(map.OriginCm.x, map.OriginCm.y, 0f) + new Vector3(size.x, 0f, size.z) / 2f;
            float halfExtent = Mathf.Max(size.x, size.z) / 2f + TopMarginM;
            float height = halfExtent / Mathf.Tan(TopFovDeg * Mathf.Deg2Rad / 2f);
            top = (centre + Vector3.up * height, Quaternion.Euler(90f, 0f, 0f));
            Vector3 position = centre + ThreeDOffset * Mathf.Max(size.x, size.z) / 2f;
            threeD = (position, Quaternion.LookRotation(centre - position));
        }

        void Update()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null && keyboard[toggleKey].wasPressedThisFrame)
                showingTop = !showingTop;
            float target = showingTop ? 0f : 1f;
            // MoveTowards lands exactly on the target; Mathf.Approximately(0, 0) is false where denormals are flushed.
            if (blend == target)
                return;
            float before = blend;
            blend = Mathf.MoveTowards(blend, target, Time.deltaTime / TransitionS);
            // Layers swap halfway, when the camera is between the two views.
            if ((before < 0.5f) != (blend < 0.5f))
                ShowLayers(blend < 0.5f);
            Apply(Mathf.SmoothStep(0f, 1f, blend));
        }

        void Apply(float t)
        {
            transform.SetPositionAndRotation(Vector3.Lerp(top.position, threeD.position, t), Quaternion.Slerp(top.rotation, threeD.rotation, t));
            GetComponent<Camera>().fieldOfView = Mathf.Lerp(TopFovDeg, ThreeDFovDeg, t);
        }

        void ShowLayers(bool fromAbove)
        {
            map.ShowBlocks(!fromAbove);
            plan.ShowWalls(fromAbove);
        }
    }
}
