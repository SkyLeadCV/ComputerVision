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
        public Material RescuerShirt;
        public Material Trousers;
        public Material Skin;
    }

    /// <summary>
    /// Builds the classroom of <see cref="ClassroomLayout"/> out of primitives. Every solid part keeps the collider its
    /// primitive comes with, so the simulated drone can detect collisions with walls, tables and people.
    /// </summary>
    public static class ClassroomBuilder
    {
        const float WallThicknessCm = 10f;

        // An adult of about 1.70 m, with the legs touching so nothing passes between them. Each part is (centre, size) in cm, standing with the feet at 0 and the face
        // towards +z; a capsule's size is its diameter across and its full length along y.
        static readonly (PrimitiveType shape, string name, Vector3 centreCm, Vector3 sizeCm, Garment garment)[] BodyParts =
        {
            (PrimitiveType.Sphere, "Head", new(0f, 157f, 0f), new(21f, 23f, 22f), Garment.Skin),
            (PrimitiveType.Capsule, "Torso", new(0f, 120f, 0f), new(34f, 58f, 20f), Garment.Shirt),
            (PrimitiveType.Capsule, "Left Arm", new(-22f, 112f, 0f), new(10f, 66f, 10f), Garment.Shirt),
            (PrimitiveType.Capsule, "Right Arm", new(22f, 112f, 0f), new(10f, 66f, 10f), Garment.Shirt),
            (PrimitiveType.Capsule, "Left Leg", new(-8f, 47f, 0f), new(16f, 94f, 16f), Garment.Trousers),
            (PrimitiveType.Capsule, "Right Leg", new(8f, 47f, 0f), new(16f, 94f, 16f), Garment.Trousers),
        };
        const float StandingHeightCm = 168f;
        // Lying on the back, the torso's half depth lifts the body off the floor.
        const float LyingLiftCm = 10f;

        enum Garment { Skin, Shirt, Trousers }

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

            Person("Victim", ClassroomLayout.Victim, lying: true, look.VictimShirt, look, room.transform);
            Person("Rescuer", ClassroomLayout.Rescuer, lying: false, look.RescuerShirt, look, room.transform);
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
        /// A person facing the placement's heading when standing. Lying, it lies on its back with the head towards the
        /// heading and the middle of the body on the placement.
        /// </summary>
        static void Person(string name, PersonPlacement placement, bool lying, Material shirt, ClassroomLook look, Transform parent)
        {
            var person = new GameObject(name).transform;
            person.SetParent(parent, false);
            person.localPosition = ClassroomLayout.ToUnity(placement.PositionCm.x, placement.PositionCm.y, 0f);
            person.localRotation = ClassroomLayout.HeadingToUnity(placement.HeadingDeg);

            var body = new GameObject("Body").transform;
            body.SetParent(person, false);
            if (lying)
            {
                // Tipping backwards turns the face (+z) up and the head to -z; turning round then puts the head at +z.
                body.localRotation = Quaternion.Euler(0f, 180f, 0f) * Quaternion.Euler(-90f, 0f, 0f);
                body.localPosition = (body.localRotation * new Vector3(0f, -StandingHeightCm / 2f, 0f) + Vector3.up * LyingLiftCm) / 100f;
            }

            foreach (var (shape, partName, centreCm, sizeCm, garment) in BodyParts)
            {
                var part = GameObject.CreatePrimitive(shape);
                part.name = partName;
                part.transform.SetParent(body, false);
                part.transform.localPosition = centreCm / 100f;
                // A capsule primitive is 2 units long, so its length scale is half the length.
                Vector3 scale = shape == PrimitiveType.Capsule ? new Vector3(sizeCm.x, sizeCm.y / 2f, sizeCm.z) : sizeCm;
                part.transform.localScale = scale / 100f;
                part.GetComponent<Renderer>().sharedMaterial = garment switch
                {
                    Garment.Skin => look.Skin,
                    Garment.Shirt => shirt,
                    _ => look.Trousers,
                };
            }
        }
    }
}
