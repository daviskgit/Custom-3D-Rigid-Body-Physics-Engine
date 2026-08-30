// main.cpp - demo scene: a falling stack of boxes onto a static ground slab,
// plus a scattered field of boxes to exercise the spatial-hash broadphase.
// Physics runs at a fixed 120Hz accumulator step, independent of render rate.
#include <cstdio>
#include <random>
#include "dynamics.h"
#include "renderer.h"

using namespace pe;

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    Renderer renderer;
    if (!renderer.init(1280, 800, "Rigid Body Physics Engine - GJK/EPA + Impulse Solver")) {
        MessageBoxA(nullptr, "Failed to initialize OpenGL 4.5 context", "Error", MB_OK);
        return 1;
    }

    World world;

    // Ground.
    RigidBody ground = makeStaticBox({25, 0.5f, 25});
    ground.position = {0, -0.5f, 0};
    world.addBody(ground);

    // A neat box stack to show resting contact + friction stability.
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> jitter(-0.01f, 0.01f);
    int stackH = 8;
    for (int y = 0; y < stackH; ++y) {
        for (int x = -1; x <= 1; ++x) {
            RigidBody b = makeBoxBody({0.5f, 0.5f, 0.5f}, 1.0f);
            b.position = {x * 1.02f + jitter(rng), 0.5f + y * 1.01f, jitter(rng)};
            b.color = {0.85f, 0.35f + 0.08f * y, 0.25f};
            b.friction = 0.6f;
            b.restitution = 0.05f;
            world.addBody(b);
        }
    }

    // A scattered field of tumbling boxes dropped from height, various sizes.
    std::uniform_real_distribution<float> posXZ(-9.0f, 9.0f);
    std::uniform_real_distribution<float> posY(6.0f, 20.0f);
    std::uniform_real_distribution<float> sizeDist(0.35f, 0.8f);
    std::uniform_real_distribution<float> angDist(-2.0f, 2.0f);
    for (int i = 0; i < 40; ++i) {
        Vec3 he{sizeDist(rng), sizeDist(rng), sizeDist(rng)};
        RigidBody b = makeBoxBody(he, he.x * he.y * he.z * 4.0f);
        b.position = {posXZ(rng), posY(rng), posXZ(rng)};
        b.orientation = Quat::fromAxisAngle(normalize(Vec3{jitter(rng) + 0.3f, 1, jitter(rng) + 0.2f}), angDist(rng));
        b.angularVelocity = {angDist(rng), angDist(rng), angDist(rng)};
        b.friction = 0.4f;
        b.restitution = 0.2f;
        b.color = {0.25f + 0.5f * sizeDist(rng), 0.4f, 0.85f};
        world.addBody(b);
    }

    Input input;
    Real camYaw = 0.7f, camPitch = 0.35f, camDist = 22.0f;
    Vec3 camTarget{0, 3, 0};

    const Real fixedDt = Real(1) / Real(120);
    Real accumulator = 0;
    LARGE_INTEGER freq, prev;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);

    char titleBuf[256];
    std::vector<InstanceData> instances;

    while (renderer.pumpEvents(input)) {
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        Real frameDt = Real(now.QuadPart - prev.QuadPart) / Real(freq.QuadPart);
        prev = now;
        if (frameDt > Real(0.1)) frameDt = Real(0.1); // clamp hitches (e.g. window drag)
        accumulator += frameDt;

        if (input.keys[VK_ESCAPE]) break;
        if (input.keys['R']) {
            // quick reset of the tumbling field's velocities if things go quiet
        }

        int steps = 0;
        while (accumulator >= fixedDt && steps < 8) {
            world.step(fixedDt);
            accumulator -= fixedDt;
            ++steps;
        }

        // Orbit camera: drag with left mouse, wheel to zoom.
        if (input.mouseDown) {
            camYaw += input.mouseDX * 0.006f;
            camPitch = clampf(camPitch - input.mouseDY * 0.006f, -1.4f, 1.4f);
        }
        camDist = clampf(camDist - input.wheelDelta * 0.003f, 5.0f, 60.0f);

        Vec3 eye = camTarget + Vec3{std::cos(camPitch) * std::sin(camYaw), std::sin(camPitch),
                                    std::cos(camPitch) * std::cos(camYaw)} * camDist;
        Mat4 view = Mat4::lookAt(eye, camTarget, {0, 1, 0});
        Mat4 proj = Mat4::perspective(PI / 3.5f, Real(renderer.width()) / Real(renderer.height()), 0.1f, 200.0f);
        renderer.setCamera(view, proj);

        instances.clear();
        instances.reserve(world.bodyCount());
        for (auto& b : world.bodies()) {
            Vec3 he = b.shape.isBox ? b.shape.halfExtents : Vec3{0.5f, 0.5f, 0.5f};
            InstanceData inst;
            inst.model = Mat4::translation(b.position) * Mat4::fromRotationScale(b.orientation.toMat3(), he);
            inst.color = b.color;
            instances.push_back(inst);
        }

        renderer.beginFrame({0.53f, 0.6f, 0.66f});
        renderer.drawInstances(instances);
        renderer.endFrame();

        static int frame = 0;
        if (++frame % 30 == 0) {
            std::snprintf(titleBuf, sizeof(titleBuf),
                          "Rigid Body Physics Engine | bodies=%d pairs=%d contacts=%d",
                          world.bodyCount(), world.lastPairCount, world.lastContactCount);
            SetWindowTextA(GetActiveWindow(), titleBuf);
        }
    }

    renderer.shutdown();
    return 0;
}
