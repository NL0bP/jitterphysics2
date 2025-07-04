/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

public partial class DynamicTree
{
    /// <summary>
    /// Preliminary result of the ray cast.
    /// </summary>
    public struct RayCastResult
    {
        public IDynamicTreeProxy Entity;
        public Real Lambda;
        public JVector Normal;
    }

    /// <summary>
    /// Post-filter delegate.
    /// </summary>
    /// <returns>False if the hit should be filtered out.</returns>
    public delegate bool RayCastFilterPost(RayCastResult result);

    /// <summary>
    /// Pre-filter delegate.
    /// </summary>
    /// <returns>False if the hit should be filtered out.</returns>
    public delegate bool RayCastFilterPre(IDynamicTreeProxy result);

    private struct Ray
    {
        public readonly JVector Origin;
        public readonly JVector Direction;

        public RayCastFilterPost? FilterPost;
        public RayCastFilterPre? FilterPre;

        public Real Lambda;

        public Ray(in JVector origin, in JVector direction)
        {
            Origin = origin;
            Direction = direction;
            FilterPost = null;
            FilterPre = null;
            Lambda = Real.MaxValue;
        }
    }

    /// <summary>
    /// Ray cast against the world.
    /// </summary>
    /// <param name="origin">Origin of the ray.</param>
    /// <param name="direction">Direction of the ray. Does not have to be normalized.</param>
    /// <param name="pre">Optional pre-filter which allows to skip shapes in the detection.</param>
    /// <param name="post">Optional post-filter which allows to skip detections.</param>
    /// <param name="proxy">The shape which was hit.</param>
    /// <param name="normal">The normal of the surface where the ray hits. Zero if ray does not hit.</param>
    /// <param name="lambda">Distance from the origin to the ray hit point in units of the ray's direction.</param>
    /// <returns>True if the ray hits, false otherwise.</returns>
    public bool RayCast(JVector origin, JVector direction, RayCastFilterPre? pre, RayCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector normal, out Real lambda)
    {
        Ray ray = new(origin, direction)
        {
            FilterPre = pre,
            FilterPost = post
        };
        bool hit = QueryRay(ray, out var result);
        proxy = result.Entity;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    /// <inheritdoc cref="RayCast(JVector, JVector, RayCastFilterPre?, RayCastFilterPost?, out IDynamicTreeProxy?, out JVector, out Real)"/>
    /// <param name="maxLambda">Maximum lambda of the ray's length to consider for intersections.</param>
    public bool RayCast(JVector origin, JVector direction, Real maxLambda, RayCastFilterPre? pre, RayCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector normal, out Real lambda)
    {
        Ray ray = new(origin, direction)
        {
            FilterPre = pre,
            FilterPost = post,
            Lambda = maxLambda
        };
        bool hit = QueryRay(ray, out var result);
        proxy = result.Entity;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    private bool QueryRay(in Ray ray, out RayCastResult result)
    {
        result = new RayCastResult();

        if (root == -1)
        {
            return false;
        }

        _stack ??= new Stack<int>(256);

        _stack.Push(root);

        bool globalHit = false;

        result.Lambda = ray.Lambda;

        while (_stack.Count > 0)
        {
            int pop = _stack.Pop();

            ref Node node = ref Nodes[pop];

            if (node.IsLeaf)
            {
                if (node.Proxy is not IRayCastable irc) continue;

                if (ray.FilterPre != null && !ray.FilterPre(node.Proxy)) continue;

                Unsafe.SkipInit(out RayCastResult res);
                bool hit = irc.RayCast(ray.Origin, ray.Direction, out res.Normal, out res.Lambda);
                res.Entity = node.Proxy;

                if (hit && res.Lambda < result.Lambda)
                {
                    if (ray.FilterPost != null && !ray.FilterPost(res)) continue;
                    result = res;
                    globalHit = true;
                }

                continue;
            }

            ref Node lNode = ref Nodes[node.Left];
            ref Node rNode = ref Nodes[node.Right];

            bool lRes = lNode.ExpandedBox.RayIntersect(ray.Origin, ray.Direction, out Real lEnter);
            bool rRes = rNode.ExpandedBox.RayIntersect(ray.Origin, ray.Direction, out Real rEnter);

            if (lEnter > result.Lambda) lRes = false;
            if (rEnter > result.Lambda) rRes = false;

            if (lRes && rRes)
            {
                if (lEnter < rEnter)
                {
                    _stack.Push(node.Right);
                    _stack.Push(node.Left);
                }
                else
                {
                    _stack.Push(node.Left);
                    _stack.Push(node.Right);
                }
            }
            else
            {
                if (lRes) _stack.Push(node.Left);
                if (rRes) _stack.Push(node.Right);
            }
        }

        return globalHit;
    }
}