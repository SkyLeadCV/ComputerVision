using UnityEngine;

namespace SkyLead.Simulator
{
    /// <summary>The materials of the virtual classroom. Any of them can be null, as in tests that only need the geometry.</summary>
    public struct ClassroomLook
    {
        public Material Floor;
        public Material Wall;
        public Material Tables;
        public Material VictimShirt;
        public Material Trousers;
        public Material Shoes;
        public Material Hair;
        public Material Skin;
    }

    /// <summary>
    /// Builds the classroom of <see cref="ClassroomLayout"/> out of primitives. Every solid part keeps the collider its
    /// primitive comes with, so the simulated drone can detect collisions with walls, tables and people.
    /// </summary>
    public static class ClassroomBuilder
    {
        const float WallThicknessCm = 10f;

        // An adult of about 1.70 m with an average build: limbs in two segments that taper, a neck, hips, hands and feet,
        // so the figure reads as a person and not as a stack of pills. Each part is (centre, size, tilt) in cm and
        // degrees, standing with the feet at 0 and the face towards +z; a capsule's size is its diameter across and its
        // full length along its axis, which the tilt turns away from y. The arms hang slightly away from the body, and the
        // thighs touch, so nothing passes between them at the drone's flight height.
        static readonly (PrimitiveType shape, string name, Vector3 centreCm, Vector3 sizeCm, Vector3 tiltDeg, Garment garment)[] BodyParts =
        {
            (PrimitiveType.Sphere, "Head", new(0f, 158f, 0f), new(16f, 22f, 19f), Vector3.zero, Garment.Skin),
            // A cap over the top and back of the head, leaving the face clear.
            (PrimitiveType.Sphere, "Hair", new(0f, 162f, -2f), new(17.5f, 17f, 20f), Vector3.zero, Garment.Hair),
            (PrimitiveType.Capsule, "Neck", new(0f, 145f, -1f), new(10f, 12f, 10f), Vector3.zero, Garment.Skin),
            (PrimitiveType.Capsule, "Chest", new(0f, 127f, 0f), new(33f, 38f, 19f), Vector3.zero, Garment.Shirt),
            (PrimitiveType.Capsule, "Waist", new(0f, 104f, 0f), new(27f, 22f, 17f), Vector3.zero, Garment.Shirt),
            // Lying across: the capsule's length is the hips' width.
            (PrimitiveType.Capsule, "Hips", new(0f, 92f, 0f), new(18f, 32f, 18f), new(0f, 0f, 90f), Garment.Trousers),
            (PrimitiveType.Capsule, "Left Upper Arm", new(-20f, 125f, 0f), new(9f, 30f, 9f), new(0f, 0f, -6f), Garment.Shirt),
            (PrimitiveType.Capsule, "Right Upper Arm", new(20f, 125f, 0f), new(9f, 30f, 9f), new(0f, 0f, 6f), Garment.Shirt),
            (PrimitiveType.Capsule, "Left Forearm", new(-22f, 98f, 2f), new(7.5f, 28f, 7.5f), new(0f, 0f, -4f), Garment.Shirt),
            (PrimitiveType.Capsule, "Right Forearm", new(22f, 98f, 2f), new(7.5f, 28f, 7.5f), new(0f, 0f, 4f), Garment.Shirt),
            (PrimitiveType.Sphere, "Left Hand", new(-23f, 80f, 3f), new(5f, 10f, 8f), Vector3.zero, Garment.Skin),
            (PrimitiveType.Sphere, "Right Hand", new(23f, 80f, 3f), new(5f, 10f, 8f), Vector3.zero, Garment.Skin),
            (PrimitiveType.Capsule, "Left Thigh", new(-7.5f, 66f, 0f), new(15f, 46f, 15f), Vector3.zero, Garment.Trousers),
            (PrimitiveType.Capsule, "Right Thigh", new(7.5f, 66f, 0f), new(15f, 46f, 15f), Vector3.zero, Garment.Trousers),
            (PrimitiveType.Capsule, "Left Shin", new(-7.5f, 26f, -1f), new(10f, 44f, 10f), Vector3.zero, Garment.Trousers),
            (PrimitiveType.Capsule, "Right Shin", new(7.5f, 26f, -1f), new(10f, 44f, 10f), Vector3.zero, Garment.Trousers),
            (PrimitiveType.Capsule, "Left Foot", new(-7.5f, 4f, 5f), new(9f, 25f, 7f), new(90f, 0f, 0f), Garment.Shoes),
            (PrimitiveType.Capsule, "Right Foot", new(7.5f, 4f, 5f), new(9f, 25f, 7f), new(90f, 0f, 0f), Garment.Shoes),
        };
        const float StandingHeightCm = 169f;
        // Lying on the back, the chest's half depth lifts the body off the floor.
        const float LyingLiftCm = 9.5f;
        // Sitting on the floor: the hip joint, where the legs bend forward, drops from standing height to the hips'
        // half thickness above the floor, which puts the top of the head near 89 cm.
        static readonly Vector3 StandingHipCm = new(0f, 92f, 0f);
        static readonly Vector3 SittingHipCm = new(0f, 12f, 0f);
        // Turns the legs from hanging down to lying forward along +z, the way the person faces.
        static readonly Quaternion LegsForward = Quaternion.Euler(-90f, 0f, 0f);

        public enum Posture { Lying, Sitting }

        enum Garment { Skin, Hair, Shirt, Trousers, Shoes }

        public static GameObject Build(ClassroomLook look)
        {
            var room = new GameObject("Classroom");
            BuildFloor(room.transform, look.Floor);

            // The walls stand outside the room's outline, so its inside is exactly the measured plan.
            float w = WallThicknessCm, width = ClassroomLayout.WidthCm, depth = ClassroomLayout.DepthCm;
            float ceiling = ClassroomLayout.CeilingCm;
            Box("Wall Left", new Vector3(-w, -w, 0f), new Vector3(0f, depth + w, ceiling), look.Wall, room.transform);
            Box("Wall Right", new Vector3(width, -w, 0f), new Vector3(width + w, depth + w, ceiling), look.Wall, room.transform);
            Box("Wall Bottom", new Vector3(0f, -w, 0f), new Vector3(width, 0f, ceiling), look.Wall, room.transform);
            Box("Wall Top", new Vector3(0f, depth, 0f), new Vector3(width, depth + w, ceiling), look.Wall, room.transform);

            Vector2 tablesMin = ClassroomLayout.TablesMinCm, tablesMax = ClassroomLayout.TablesMaxCm;
            Box("Tables Block", new Vector3(tablesMin.x, tablesMin.y, 0f), new Vector3(tablesMax.x, tablesMax.y, ceiling), look.Tables, room.transform);

            Person("Victim", ClassroomLayout.Victim, Posture.Sitting, look.VictimShirt, look, room.transform);
            return room;
        }

        static void BuildFloor(Transform parent, Material material)
        {
            var floor = GameObject.CreatePrimitive(PrimitiveType.Quad);
            floor.name = "Floor";
            floor.transform.SetParent(parent, false);
            float width = ClassroomLayout.WidthCm, depth = ClassroomLayout.DepthCm;
            floor.transform.localPosition = ClassroomLayout.ToUnity(width / 2f, depth / 2f, 0f);
            // A quad faces -z; tilting it forward makes it face up with its texture's v along the room's +y.
            floor.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
            floor.transform.localScale = new Vector3(width, depth, 100f) / 100f;
            floor.GetComponent<Renderer>().sharedMaterial = material;
        }

        /// <summary>An axis-aligned box between two room corners in cm.</summary>
        static void Box(string name, Vector3 minCm, Vector3 maxCm, Material material, Transform parent)
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            box.transform.SetParent(parent, false);
            Vector3 centre = (minCm + maxCm) / 2f, size = maxCm - minCm;
            box.transform.localPosition = ClassroomLayout.ToUnity(centre.x, centre.y, centre.z);
            box.transform.localScale = ClassroomLayout.ToUnity(size.x, size.y, size.z);
            box.GetComponent<Renderer>().sharedMaterial = material;
        }

        /// <summary>
        /// A person on the floor. Lying, it lies on its back with the head towards the placement's heading and the middle
        /// of the body on the placement. Sitting, its hips are on the placement and it faces the heading, legs out in front.
        /// </summary>
        static void Person(string name, PersonPlacement placement, Posture posture, Material shirt, ClassroomLook look, Transform parent)
        {
            var person = new GameObject(name).transform;
            person.SetParent(parent, false);
            person.localPosition = ClassroomLayout.ToUnity(placement.PositionCm.x, placement.PositionCm.y, 0f);
            person.localRotation = ClassroomLayout.HeadingToUnity(placement.HeadingDeg);

            var body = new GameObject("Body").transform;
            body.SetParent(person, false);
            if (posture == Posture.Lying)
            {
                // Tipping backwards turns the face (+z) up and the head to -z; turning round then puts the head at +z.
                body.localRotation = Quaternion.Euler(0f, 180f, 0f) * Quaternion.Euler(-90f, 0f, 0f);
                body.localPosition = (body.localRotation * new Vector3(0f, -StandingHeightCm / 2f, 0f) + Vector3.up * LyingLiftCm) / 100f;
            }

            foreach (var (shape, partName, centreCm, sizeCm, tiltDeg, garment) in BodyParts)
            {
                var part = GameObject.CreatePrimitive(shape);
                part.name = partName;
                part.transform.SetParent(body, false);
                Vector3 centre = centreCm;
                Quaternion tilt = Quaternion.Euler(tiltDeg);
                if (posture == Posture.Sitting)
                {
                    // The upper body drops straight down onto the floor; the legs also swing forward about the hip.
                    Vector3 fromHip = centreCm - StandingHipCm;
                    bool leg = partName.Contains("Thigh") || partName.Contains("Shin") || partName.Contains("Foot");
                    centre = SittingHipCm + (leg ? LegsForward * fromHip : fromHip);
                    if (leg)
                        tilt = LegsForward * tilt;
                }
                part.transform.SetLocalPositionAndRotation(centre / 100f, tilt);
                // A capsule primitive is 2 units long, so its length scale is half the length.
                Vector3 scale = shape == PrimitiveType.Capsule ? new Vector3(sizeCm.x, sizeCm.y / 2f, sizeCm.z) : sizeCm;
                part.transform.localScale = scale / 100f;
                part.GetComponent<Renderer>().sharedMaterial = garment switch
                {
                    Garment.Skin => look.Skin,
                    Garment.Shirt => shirt,
                    Garment.Shoes => look.Shoes,
                    Garment.Hair => look.Hair,
                    _ => look.Trousers,
                };
            }
        }
    }
}
