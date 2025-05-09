#  <img src="./media/logo/jitterstringsmallsmall.png" alt="screenshot" width="240"/> Jitter Physics 2

[![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/notgiven688/jitterphysics2/jitter-tests.yml?label=JitterTests)](https://github.com/notgiven688/jitterphysics2/actions/workflows/jitter-tests.yml)
[![Nuget](https://img.shields.io/nuget/v/Jitter2?color=yellow)](https://www.nuget.org/packages/Jitter2/)
[![Discord](https://img.shields.io/discord/1213790465225138197?logo=discord&logoColor=lightgray&label=discord&color=blue)](https://discord.gg/7jr3f4edmV)

Jitter Physics 2, the evolution of [Jitter Physics](https://github.com/notgiven688/jitterphysics), is an impulse-based dynamics engine with a semi-implicit Euler integrator. It is a fast, simple, and dependency-free engine written in C# with a clear and user-friendly API.

The official **NuGet** package ([changelog](https://jitterphysics.com/docs/changelog)) can be found [here](https://www.nuget.org/packages/Jitter2), the *double precision* version [here](https://www.nuget.org/packages/Jitter2.Double).

---

Run a small demo directly [in the browser](https://jitterphysics.com/AppBundle/index.html)!

There is also a tiny demo available for the [Godot engine](other/GodotDemo).

See below for a fully-featured demo.

---

<img src="./media/screenshots/jitter_screenshot0.png" alt="screenshot" width="400"/> <img src="./media/screenshots/jitter_screenshot1.png" alt="screenshot" width="400"/>

<img src="./media/screenshots/jitter_screenshot2.png" alt="screenshot" width="400"/> <img src="./media/screenshots/jitter_screenshot4.png" alt="screenshot" width="400"/>

## Getting Started

Jitter is cross-platform. The `src` directory contains four projects:

| Project          | Description                                                |
|------------------|------------------------------------------------------------|
| Jitter2          | The main library housing Jitter2's functionalities.         |
| JitterDemo       | Features demo scenes rendered with OpenGL, tested on Linux and Windows. |
| JitterBenchmark  | The setup for conducting benchmarks using BenchmarkDotNet.  |
| JitterTests      | Unit tests utilizing NUnit.

To run the demo scenes:

- Install [.NET 9.0](https://dotnet.microsoft.com/download/dotnet/9.0)
- `git clone https://github.com/notgiven688/jitterphysics2.git`
- `cd ./jitterphysics2/src/JitterDemo && dotnet run -c Release`

JitterDemo uses [GLFW](https://www.glfw.org/) for accessing OpenGL and managing windows, and [cimgui](https://github.com/cimgui/cimgui) for GUI rendering. The project contains these native binaries in a precompiled form.

## Features

- [x] Compile time option for double precision.
- [x] Speculative contacts (avoiding the bullet-through-paper problem).
- [x] A variety of constraints and motors (AngularMotor, BallSocket, ConeLimit, DistanceLimit, FixedAngle, HingeAngle, LinearMotor, PointOnLine, PointOnPlane, TwistAngle) with support for softness.
- [x] A sophisticated deactivation scheme with minimal cost for inactive rigid bodies (scenes with 100k inactive bodies are easily achievable).
- [x] Edge collision filter for internal edges of triangle meshes.
- [x] Substepping for improved constraint and contact stability.
- [x] Generic convex-convex collision detection using EPA-aided MPR.
- [x] "One-shot" contact manifolds using auxiliary contacts for flat surface collisions.
- [x] Efficient compound shapes.
- [x] Easy integration of custom shapes. Integrated: Box, Capsule, Cone, Convex Hull, Point Cloud, Sphere, Triangle, Transformed.
- [x] Soft-body dynamics!

## Documentation

Find the [documentation here](https://notgiven688.github.io/jitterphysics2).

## Credits

Grateful acknowledgment to Erin Catto, Dirk Gregorius, Erwin Coumans, Gino van den Bergen, Daniel Chappuis, Marijn Tamis, Danny Chapman, Gary Snethen, and Christer Ericson for sharing their knowledge through forum posts, talks, code, papers, and books.

Special thanks also to the contributors of the predecessor projects JigLibX and Jitter.

## Contribute 👋

Contributions of all forms are welcome! Feel free to fork the project and create a pull request.
