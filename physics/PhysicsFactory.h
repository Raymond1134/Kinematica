#pragma once
#include "PhysicsWorld.h"
#include "RigidBody.h"
#include "Material.h"
#include "SoftBody.h"
#include "CompoundBuilder.h"
#include <list>

/**
 * @brief Factory class for creating physics objects.
 * 
 * Handles the creation of RigidBodies, SoftBodies, and Prefabs, ensuring they are
 * correctly initialized and added to the PhysicsWorld.
 */
class PhysicsFactory {
public:
    /**
     * @brief Construct a new Physics Factory object.
     * 
     * @param world Reference to the PhysicsWorld.
     * @param bodies Reference to the list of bodies in the world.
     */
    PhysicsFactory(PhysicsWorld& world, std::list<RigidBody>& bodies);

    /**
     * @brief Create a Box RigidBody.
     * 
     * @param pos World position.
     * @param size Half-extents of the box (width/2, height/2, depth/2).
     * @param mat Material properties.
     * @param mass Mass in kg.
     * @param isStatic If true, mass is ignored (infinite) and body doesn't move.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the created body.
     */
    RigidBody* CreateBox(const Vec3& pos, const Vec3& size, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Sphere RigidBody.
     * 
     * @param pos World position.
     * @param radius Radius in meters.
     * @param mat Material properties.
     * @param mass Mass in kg.
     * @param isStatic If true, body is static.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the created body.
     */
    RigidBody* CreateSphere(const Vec3& pos, float radius, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Capsule RigidBody.
     * 
     * @param pos World position.
     * @param radius Radius of the hemispherical ends.
     * @param height Total height (or half-height depending on implementation, check Collider).
     * @param mat Material properties.
     * @param mass Mass in kg.
     * @param isStatic If true, body is static.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the created body.
     */
    RigidBody* CreateCapsule(const Vec3& pos, float radius, float height, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Convex Hull RigidBody.
     * 
     * @param pos World position.
     * @param verts List of vertices defining the convex hull.
     * @param mat Material properties.
     * @param mass Mass in kg.
     * @param isStatic If true, body is static.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the created body.
     */
    RigidBody* CreateConvex(const Vec3& pos, const std::vector<Vec3>& verts, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Mesh RigidBody.
     * 
     * @param pos World position.
     * @param mesh Shared pointer to the TriangleMesh.
     * @param mat Material properties.
     * @param mass Mass in kg.
     * @param isStatic If true, body is static.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the created body.
     */
    RigidBody* CreateMesh(const Vec3& pos, std::shared_ptr<TriangleMesh> mesh, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Compound RigidBody.
     * 
     * @param pos World position.
     * @param children List of child shapes with local transforms.
     * @param mat Material properties.
     * @param mass Mass in kg.
     * @param isStatic If true, body is static.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the created body.
     */
    RigidBody* CreateCompound(const Vec3& pos, const std::vector<CompoundChild>& children, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});
    
    /**
     * @brief Create a Compound RigidBody using a Builder.
     * 
     * @param pos World position.
     * @param builder The CompoundBuilder containing the shapes.
     * @param mat Material properties.
     * @param mass Mass in kg.
     * @param isStatic If true, body is static.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the created body.
     */
    RigidBody* CreateCompound(const Vec3& pos, const CompoundBuilder& builder, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

    // Prefabs

    /**
     * @brief Create a Car Prefab (Chassis + 4 Wheels).
     * 
     * @param pos World position.
     * @param size Scaling factor.
     * @param mat Material properties.
     * @param isStatic If true, car is static.
     * @param vel Initial velocity.
     * @return RigidBody* Pointer to the chassis body.
     */
    RigidBody* CreateCar(const Vec3& pos, float size, const MaterialProps& mat, bool isStatic = false, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Chain Prefab.
     * 
     * @param startPos Starting position of the first link.
     * @param dir Direction to extend the chain.
     * @param length Number of links.
     * @param radius Radius of each link.
     * @param spacing Distance between link centers.
     * @param mat Material properties.
     * @param startStatic If true, the first link is static (anchored).
     * @param vel Initial velocity.
     * @return std::vector<RigidBody*> List of created bodies.
     */
    std::vector<RigidBody*> CreateChain(const Vec3& startPos, const Vec3& dir, int length, float radius, float spacing, const MaterialProps& mat, bool startStatic, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Soft Body Blob.
     * 
     * @param pos Center position.
     * @param dim Grid dimensions (dim x dim x dim).
     * @param spacing Spacing between particles.
     * @param radius Radius of particles.
     * @param mat Material properties.
     * @param isStatic If true, all particles are static.
     * @param vel Initial velocity.
     * @return SoftBody The created soft body structure.
     */
    SoftBody CreateBlob(const Vec3& pos, int dim, float spacing, float radius, const MaterialProps& mat, bool isStatic = false, const Vec3& vel = {0,0,0});

    /**
     * @brief Create a Cloth Soft Body.
     * 
     * @param pos Top-left position.
     * @param rows Number of rows.
     * @param cols Number of columns.
     * @param spacing Spacing between particles.
     * @param mat Material properties.
     * @param isStatic If true, top row is static (pinned).
     * @param vel Initial velocity.
     * @return SoftBody The created soft body structure.
     */
    SoftBody CreateCloth(const Vec3& pos, int rows, int cols, float spacing, const MaterialProps& mat, bool isStatic = false, const Vec3& vel = {0,0,0});

private:
    PhysicsWorld& world;
    std::list<RigidBody>& bodies;
    
    RigidBody* CreateBodyBase(const Vec3& pos, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel);
};
