using System;
using System.Collections.Generic;
using System.Linq;
using Jitter2;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using Jitter2.SoftBodies;
using JitterDemo.Renderer;

namespace JitterDemo;

public class SoftBodyCloth : SoftBody
{
    private readonly struct Edge : IEquatable<Edge>
    {
        public readonly ushort IndexA;
        public readonly ushort IndexB;

        public Edge(ushort u0, ushort u1)
        {
            IndexA = u0;
            IndexB = u1;
        }

        public bool Equals(Edge other)
        {
            return other.IndexA == IndexA && other.IndexB == IndexB;
        }

        public override bool Equals(object? obj)
        {
            return obj is Edge other && Equals(other);
        }

        public override int GetHashCode()
        {
            return IndexA * 24712 + IndexB;
        }
    }

    private List<JVector> vertices = null!;
    private List<TriangleVertexIndex> triangles = null!;
    private List<Edge> edges = null!;

    public List<TriangleVertexIndex> Triangles => triangles;

    public SoftBodyCloth(World world, IEnumerable<JTriangle> triangles) : base(world)
    {
        LoadMesh(triangles);
        Build();
    }

    private void LoadMesh(IEnumerable<JTriangle> tris)
    {
        Dictionary<JVector, ushort> verts = new();
        HashSet<Edge> edgs = new();

        ushort AddVertex(in JVector vertex)
        {
            if (!verts.TryGetValue(vertex, out ushort ind))
            {
                ind = (ushort)verts.Count;
                verts.Add(vertex, ind);
            }

            return ind;
        }

        triangles = new List<TriangleVertexIndex>();

        foreach (var tri in tris)
        {
            ushort u0 = AddVertex(tri.V0);
            ushort u1 = AddVertex(tri.V1);
            ushort u2 = AddVertex(tri.V2);

            TriangleVertexIndex t = new TriangleVertexIndex(u0, u1, u2);
            triangles.Add(t);

            edgs.Add(new Edge(u0, u1));
            edgs.Add(new Edge(u0, u2));
            edgs.Add(new Edge(u1, u2));
        }

        vertices = verts.Keys.ToList();
        edges = edgs.ToList();
    }

    private void Build()
    {
        foreach (var vertex in vertices)
        {
            RigidBody body = World.CreateRigidBody();
            body.SetMassInertia(JMatrix.Zero, 100.0f, true);
            body.Position = vertex;
            Vertices.Add(body);
        }

        foreach (var edge in edges)
        {
            var constraint = World.CreateConstraint<SpringConstraint>(Vertices[edge.IndexA], Vertices[edge.IndexB]);
            constraint.Initialize(Vertices[edge.IndexA].Position, Vertices[edge.IndexB].Position);
            constraint.Softness = 0.2f;
            Springs.Add(constraint);
        }

        foreach (var triangle in triangles)
        {
            var tri = new SoftBodyTriangle(this, Vertices[(int)triangle.T1], Vertices[(int)triangle.T2], Vertices[(int)triangle.T3]);
            tri.UpdateWorldBoundingBox();
            World.DynamicTree.AddProxy(tri);
            Shapes.Add(tri);
        }
    }
}