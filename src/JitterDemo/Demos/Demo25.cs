using System;
using System.Collections.Generic;
using Jitter2;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using JitterDemo.Renderer;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo;

public static class Heightmap
{
    public static int Width { get; } = 10_000;
    public static int Height { get; } = 10_000;

    public static float GetHeight(int x, int y)
    {
        return MathF.Sin(x*0.1f) * MathF.Cos(y*0.1f) * 2f;
    }

    // Only for rendering...
    public static JVector GetNormal(int x, int z)
    {
        float dx = (GetHeight(x + 1, z) - GetHeight(x - 1, z)) * 0.5f;
        float dz = (GetHeight(x, z + 1) - GetHeight(x, z - 1)) * 0.5f;

        var normal = new JVector(-dx, 1f, -dz);
        normal.Normalize();

        return normal;
    }

    public static JBBox GetAABox()
    {
        JBBox box = JBBox.SmallBox;

        for (int x = 0; x < Width; x++)
        {
            for (int z = 0; z < Height; z++)
            {
                box.AddPoint(new JVector(x, GetHeight(x, z), z));
            }
        }

        return box;
    }
}

public class HeightmapTester(JBBox box) : IDynamicTreeProxy, IRayCastable
{
    public int SetIndex { get; set; } = -1;
    public int NodePtr { get; set; }

    public JVector Velocity => JVector.Zero;
    public JBBox WorldBoundingBox { get; } = box;

    public bool RayCast(in JVector origin, in JVector direction, out JVector normal, out float lambda)
    {
        // TODO: Add ray-casting against the Heightmap.
        Console.WriteLine("Raycasting against heightmap not implemented.");
        normal = JVector.Zero; lambda = 0.0f;
        return false;
    }
}

public class HeightmapDetection : IBroadPhaseFilter
{
    private readonly World world;
    private readonly HeightmapTester shape;
    private readonly ulong minIndex;

    public HeightmapDetection(World world, HeightmapTester shape)
    {
        this.shape = shape;
        this.world = world;

        (minIndex, _) = World.RequestId(Heightmap.Width * Heightmap.Height * 2);
    }

    public bool Filter(IDynamicTreeProxy shapeA, IDynamicTreeProxy shapeB)
    {
        if (shapeA != shape && shapeB != shape) return true;

        var collider = shapeA == shape ? shapeB : shapeA;

        if (collider is not RigidBodyShape rbs || rbs.RigidBody.Data.IsStaticOrInactive) return false;

        var min = collider.WorldBoundingBox.Min;
        var max = collider.WorldBoundingBox.Max;

        int minX = Math.Max(0, (int)min.X);
        int minZ = Math.Max(0, (int)min.Z);
        int maxX = Math.Min(Heightmap.Width, (int)max.X + 1);
        int maxZ = Math.Min(Heightmap.Height, (int)max.Z + 1);

        for (int x = minX; x < maxX; x++)
        {
            for (int z = minZ; z < maxZ; z++)
            {
                ref RigidBodyData body = ref rbs.RigidBody!.Data;

                // One triangle of the quad

                ulong index = 2 * (ulong)(x * Heightmap.Width + z);

                CollisionTriangle triangle;

                triangle.A = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 0), z + 0);
                triangle.B = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 0), z + 0);
                triangle.C = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 1), z + 1);

                JVector normal = JVector.Normalize((triangle.C - triangle.A) % (triangle.B - triangle.A));

                bool hit = NarrowPhase.MPREPA(triangle, rbs, body.Orientation, body.Position,
                    out JVector pointA, out JVector pointB, out _, out float penetration);

                if (hit)
                {
                    world.RegisterContact(rbs.ShapeId, minIndex + index, world.NullBody, rbs.RigidBody,
                        pointA, pointB, normal, penetration);
                }

                // Second triangle of the quad

                index += 1;
                triangle.A = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 0), z + 0);
                triangle.B = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 1), z + 1);
                triangle.C = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 1), z + 1);

                normal = JVector.Normalize((triangle.C - triangle.A) % (triangle.B - triangle.A));

                hit = NarrowPhase.MPREPA(triangle, rbs, body.Orientation, body.Position,
                    out pointA, out pointB, out _, out penetration);

                if (hit)
                {
                    world.RegisterContact(rbs.ShapeId, minIndex + index, world.NullBody, rbs.RigidBody,
                        pointA, pointB, normal, penetration);
                }
            }
        }

        return false;
    }
}

public class Demo25 : IDemo
{
    public string Name => "Custom Collision (Heightmap)";

    private Cloth terrainRenderer = null!;

    // Only for rendering...
    TriangleVertexIndex[] GetIndices(int width, int height)
    {
        var indices = new List<TriangleVertexIndex>();
        for (int i = 0; i < width - 1; i++)
        {
            for (int j = 0; j < height - 1; j++)
            {
                indices.Add(new TriangleVertexIndex(i * height + j, i * height + j + 1, (i + 1) * height + j));
                indices.Add(new TriangleVertexIndex(i * height + j + 1, (i + 1) * height + j + 1, (i + 1) * height + j));
            }
        }
        return indices.ToArray();
    }

    // Only for rendering...
    void FillVertices(Vertex[] vertices, int width, int height)
    {
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                vertices[i * height + j].Position = new Vector3(i, Heightmap.GetHeight(i, j), j);
                vertices[i * height + j].Texture = new Vector2((float)i, (float)j) * 0.5f;
                vertices[i * height + j].Normal = Conversion.FromJitter(Heightmap.GetNormal(i, j));
            }
        }
    }

    public void Build()
    {
        Playground pg = (Playground)RenderWindow.Instance;
        World world = pg.World;

        pg.ResetScene(false);

        var tester = new HeightmapTester(Heightmap.GetAABox());

        world.BroadPhaseFilter = new HeightmapDetection(world, tester);
        world.DynamicTree.AddProxy(tester, false);

        terrainRenderer = RenderWindow.Instance.CSMRenderer.GetInstance<Cloth>();

        terrainRenderer.SetIndices(GetIndices(100, 100));
        FillVertices(terrainRenderer.Vertices, 100, 100);

        terrainRenderer.VerticesChanged();
    }

    public void Draw()
    {
        terrainRenderer.PushMatrix(Matrix4.Identity);
    }
}