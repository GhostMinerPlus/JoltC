#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/StreamUtils.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <JoltC/Extend.h>

#define LAYOUT_COMPATIBLE(c_type, cpp_type)                                                                        \
    static c_type to_jpc(cpp_type in)                                                                              \
    {                                                                                                              \
        c_type out;                                                                                                \
        memcpy(&out, &in, sizeof(c_type));                                                                         \
        return out;                                                                                                \
    }                                                                                                              \
    static cpp_type to_jph(c_type in)                                                                              \
    {                                                                                                              \
        cpp_type out;                                                                                              \
        memcpy(&out, &in, sizeof(cpp_type));                                                                       \
        return out;                                                                                                \
    }                                                                                                              \
    static c_type *to_jpc(cpp_type *in)                                                                            \
    {                                                                                                              \
        return reinterpret_cast<c_type *>(in);                                                                     \
    }                                                                                                              \
    static cpp_type *to_jph(c_type *in)                                                                            \
    {                                                                                                              \
        return reinterpret_cast<cpp_type *>(in);                                                                   \
    }                                                                                                              \
    static const c_type *to_jpc(const cpp_type *in)                                                                \
    {                                                                                                              \
        return reinterpret_cast<const c_type *>(in);                                                               \
    }                                                                                                              \
    static const cpp_type *to_jph(const c_type *in)                                                                \
    {                                                                                                              \
        return reinterpret_cast<const cpp_type *>(in);                                                             \
    }                                                                                                              \
    static_assert(sizeof(c_type) == sizeof(cpp_type), "size of " #c_type " did not match size of " #cpp_type);     \
    static_assert(alignof(c_type) == alignof(cpp_type), "align of " #c_type " did not match align of " #cpp_type); \
    static_assert(!std::is_polymorphic_v<cpp_type>, #cpp_type " is polymorphic and cannot be made layout compatible");

LAYOUT_COMPATIBLE(JPC_MassProperties, JPH::MassProperties)

class ExtendShape : public JPH::Shape
{
public:
    explicit ExtendShape(JPH::Shape *inShape, JPH::uint64 inUserData, JPH::Vec3 inCenterOfMass, JPH::MassProperties inMass) : JPH::Shape(JPH::EShapeType::User1, JPH::EShapeSubType::User1), mCenterOfMass(inCenterOfMass), mMass(inMass), mShape(inShape)
    {
        SetUserData(inUserData);
    }

    virtual JPH::AABox GetLocalBounds() const override
    {
        return mShape->GetLocalBounds();
    }

    virtual uint GetSubShapeIDBitsRecursive() const override
    {
        return mShape->GetSubShapeIDBitsRecursive();
    }

    virtual float GetInnerRadius() const override
    {
        return mShape->GetInnerRadius();
    }

    virtual JPH::MassProperties GetMassProperties() const override
    {
        return mMass;
    }

    virtual const JPH::PhysicsMaterial *GetMaterial(const JPH::SubShapeID &inSubShapeID) const override
    {
        return mShape->GetMaterial(inSubShapeID);
    }

    virtual JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inLocalSurfacePosition) const override
    {
        return mShape->GetSurfaceNormal(inSubShapeID, inLocalSurfacePosition);
    }

    virtual void GetSubmergedVolume(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, JPH::Vec3 &outCenterOfBuoyancy
#ifdef JPH_DEBUG_RENDERER
                                    ,
                                    JPH::RVec3Arg inBaseOffset
#endif
    ) const override
    {
        mShape->GetSubmergedVolume(inCenterOfMassTransform, inScale, inSurface, outTotalVolume, outSubmergedVolume, outCenterOfBuoyancy
#ifdef JPH_DEBUG_RENDERER
                                   ,
                                   inBaseOffset
#endif
        );
    }

#ifdef JPH_DEBUG_RENDERER
    virtual void Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override
    {
        mShape->Draw(inRenderer, inCenterOfMassTransform, inScale, inColor, inUseMaterialColors, inDrawWireframe);
    }

    virtual void DrawGetSupportFunction(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inDrawSupportDirection) const override
    {
        mShape->DrawGetSupportFunction(inRenderer, inCenterOfMassTransform, inScale, inColor, inDrawSupportDirection);
    }

    virtual void DrawGetSupportingFace(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale) const override
    {
        mShape->DrawGetSupportingFace(inRenderer, inCenterOfMassTransform, inScale);
    }
#endif

    virtual bool CastRay(const JPH::RayCast &inRay, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::RayCastResult &ioHit) const override
    {
        return mShape->CastRay(inRay, inSubShapeIDCreator, ioHit);
    }

    virtual void CastRay(const JPH::RayCast &inRay, const JPH::RayCastSettings &inRayCastSettings, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CastRayCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter = {}) const override
    {
        mShape->CastRay(inRay, inRayCastSettings, inSubShapeIDCreator, ioCollector, inShapeFilter);
    }

    virtual void CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CollidePointCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter = {}) const override
    {
        mShape->CollidePoint(inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
    }

    virtual void CollideSoftBodyVertices(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override
    {
        mShape->CollideSoftBodyVertices(inCenterOfMassTransform, inScale, inVertices, inNumVertices, inCollidingShapeIndex);
    }

    virtual void GetTrianglesStart(JPH::Shape::GetTrianglesContext &ioContext, const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale) const override
    {
        mShape->GetTrianglesStart(ioContext, inBox, inPositionCOM, inRotation, inScale);
    }

    virtual int GetTrianglesNext(JPH::Shape::GetTrianglesContext &ioContext, int inMaxTrianglesRequested, JPH::Float3 *outTriangleVertices, const JPH::PhysicsMaterial **outMaterials = nullptr) const override
    {
        return mShape->GetTrianglesNext(ioContext, inMaxTrianglesRequested, outTriangleVertices, outMaterials);
    }

    virtual JPH::Shape::Stats GetStats() const override
    {
        return mShape->GetStats();
    }

    virtual float GetVolume() const override
    {
        return mShape->GetVolume();
    }

    virtual bool MustBeStatic() const override
    {
        return mShape->MustBeStatic();
    }

    virtual JPH::Vec3 GetCenterOfMass() const override
    {
        return mCenterOfMass;
    }

    virtual JPH::AABox GetWorldSpaceBounds(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale) const override
    {
        return mShape->GetWorldSpaceBounds(inCenterOfMassTransform, inScale);
    }

    virtual const JPH::Shape *GetLeafShape(const JPH::SubShapeID &inSubShapeID, JPH::SubShapeID &outRemainder) const override
    {
        return mShape->GetLeafShape(inSubShapeID, outRemainder);
    }

    using SupportingFace = JPH::StaticArray<JPH::Vec3, 32>;

    virtual void GetSupportingFace(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inDirection, JPH::Vec3Arg inScale, JPH::Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override
    {
        mShape->GetSupportingFace(inSubShapeID, inDirection, inScale, inCenterOfMassTransform, outVertices);
    }

    virtual JPH::uint64 GetSubShapeUserData(const JPH::SubShapeID &inSubShapeID) const override
    {
        return mShape->GetSubShapeUserData(inSubShapeID);
    }

    virtual JPH::TransformedShape GetSubShapeTransformedShape(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale, JPH::SubShapeID &outRemainder) const override
    {
        return mShape->GetSubShapeTransformedShape(inSubShapeID, inPositionCOM, inRotation, inScale, outRemainder);
    }

    virtual bool IsValidScale(JPH::Vec3Arg inScale) const override
    {
        return mShape->IsValidScale(inScale);
    }

    virtual JPH::Vec3 MakeScaleValid(JPH::Vec3Arg inScale) const override
    {
        return mShape->MakeScaleValid(inScale);
    }

    virtual void SaveBinaryState(JPH::StreamOut &inStream) const override
    {
        mShape->SaveBinaryState(inStream);
    }

    virtual void SaveMaterialState(JPH::PhysicsMaterialList &outMaterials) const override
    {
        mShape->SaveMaterialState(outMaterials);
    }

    virtual void RestoreMaterialState(const JPH::PhysicsMaterialRefC *inMaterials, uint inNumMaterials) override
    {
        mShape->RestoreMaterialState(inMaterials, inNumMaterials);
    }

    virtual void SaveSubShapeState(JPH::ShapeList &outSubShapes) const override
    {
        mShape->SaveSubShapeState(outSubShapes);
    }

    virtual void RestoreSubShapeState(const JPH::ShapeRefC *inSubShapes, uint inNumShapes) override
    {
        mShape->RestoreSubShapeState(inSubShapes, inNumShapes);
    }

    virtual void CollectTransformedShapes(const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::TransformedShapeCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter) const override
    {
        mShape->CollectTransformedShapes(inBox, inPositionCOM, inRotation, inScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
    }

    virtual void TransformShape(JPH::Mat44Arg inCenterOfMassTransform, JPH::TransformedShapeCollector &ioCollector) const override
    {
        mShape->TransformShape(inCenterOfMassTransform, ioCollector);
    }

    virtual JPH::Shape::Stats GetStatsRecursive(JPH::Shape::VisitedShapes &ioVisitedShapes) const override
    {
        return mShape->GetStatsRecursive(ioVisitedShapes);
    }

    JPH::Shape *GetShape() const
    {
        return mShape.GetPtr();
    }

private:
    JPH::Vec3 mCenterOfMass;
    JPH::MassProperties mMass;
    JPH::Ref<JPH::Shape> mShape;
};

JPC_API JPC_ExtendShape *JPC_ExtendShape_new(JPC_Shape *inShape, uint64_t inUserData, JPC_Vec3 inCenterOfMass, JPC_MassProperties inMass)
{
    auto shape = new ExtendShape((JPH::Shape *)inShape, inUserData, JPH::Vec3(inCenterOfMass.x, inCenterOfMass.y, inCenterOfMass.z), to_jph(inMass));

    return (JPC_ExtendShape *)shape;
}

JPC_API JPC_Shape *JPC_ExtendShape_GetShape(JPC_ExtendShape *inShape)
{
    auto shape = (ExtendShape *)inShape;

    return (JPC_Shape *)shape->GetShape();
}
