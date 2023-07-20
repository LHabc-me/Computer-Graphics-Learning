cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cc Object.h Vector.cc Vector.h Sphere.h global.h Triangle.h Scene.cc
        Scene.h Light.h AreaLight.h BVH.cc BVH.h Bounds3.h Ray.h Material.h Intersection.h
        Renderer.cc Renderer.h)
