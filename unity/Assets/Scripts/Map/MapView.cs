using System.Collections.Generic;
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

        // How quickly the framing follows the explored area as it grows: about a second to settle, so the view moves
        // with the drone without jumping at each photo.
        const float FollowRate = 3f;

        (Vector3 position, Quaternion rotation) top, threeD;
        bool showingTop = true;
        float blend;

        public bool ShowingTop => showingTop;

        void Start()
        {
            Frame(AreaToShow(plan, map.GetComponent<FlightTrail>()), out top, out threeD);
            blend = 0f;
            Apply(0f);
            ShowLayers(true);
        }

        /// <summary>
        /// The part of the map worth showing, in protocol cm: the floor proven free and the take-off spot (from the plan),
        /// the trail and the drone. The voxel block is far larger, so that any flight fits in it.
        /// </summary>
        public static Rect AreaToShow(FloorPlan plan, FlightTrail trail)
        {
            Rect area = plan.ExploredCm;
            var points = new List<Vector3>(trail.Points) { trail.Drone.localPosition };
            foreach (Vector3 point in points)
                area = Rect.MinMaxRect(Mathf.Min(area.xMin, point.x * 100f), Mathf.Min(area.yMin, point.z * 100f),
                    Mathf.Max(area.xMax, point.x * 100f), Mathf.Max(area.yMax, point.z * 100f));
            return area;
        }

        /// <summary>The two camera poses that fit an area of the floor: straight down from above, and above and behind it.</summary>
        public static void Frame(Rect areaCm, out (Vector3 position, Quaternion rotation) top,
            out (Vector3 position, Quaternion rotation) threeD)
        {
            Vector3 centre = VoxelGrid.ToUnity(areaCm.center.x, areaCm.center.y, 0f);
            float extent = Mathf.Max(areaCm.width, areaCm.height) / 100f;
            float halfExtent = extent / 2f + TopMarginM;
            float height = halfExtent / Mathf.Tan(TopFovDeg * Mathf.Deg2Rad / 2f);
            top = (centre + Vector3.up * height, Quaternion.Euler(90f, 0f, 0f));
            Vector3 position = centre + ThreeDOffset * extent / 2f;
            threeD = (position, Quaternion.LookRotation(centre - position));
        }

        void Update()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null && keyboard[toggleKey].wasPressedThisFrame)
                showingTop = !showingTop;

            Frame(AreaToShow(plan, map.GetComponent<FlightTrail>()), out var topGoal, out var threeDGoal);
            // Exponential smoothing: the same share of the remaining way every second, whatever the frame rate.
            float follow = 1f - Mathf.Exp(-FollowRate * Time.deltaTime);
            top = (Vector3.Lerp(top.position, topGoal.position, follow), Quaternion.Slerp(top.rotation, topGoal.rotation, follow));
            threeD = (Vector3.Lerp(threeD.position, threeDGoal.position, follow),
                Quaternion.Slerp(threeD.rotation, threeDGoal.rotation, follow));

            float target = showingTop ? 0f : 1f;
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
