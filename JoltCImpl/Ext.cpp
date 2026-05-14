#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>

#include <JoltC/Ext.h>

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

#define OPAQUE_WRAPPER(c_type, cpp_type)                                                               \
    static c_type *to_jpc(cpp_type *in) { return reinterpret_cast<c_type *>(in); }                     \
    static const c_type *to_jpc(const cpp_type *in) { return reinterpret_cast<const c_type *>(in); }   \
    static cpp_type *to_jph(c_type *in) { return reinterpret_cast<cpp_type *>(in); }                   \
    static const cpp_type *to_jph(const c_type *in) { return reinterpret_cast<const cpp_type *>(in); } \
    static cpp_type **to_jph(c_type **in) { return reinterpret_cast<cpp_type **>(in); }

LAYOUT_COMPATIBLE(JPC_MassProperties, JPH::MassProperties)

OPAQUE_WRAPPER(JPC_Shape, JPH::Shape)

class JPC_MassShape final : JPH::DecoratedShape
{
public:
    JPH_OVERRIDE_NEW_DELETE

    explicit JPC_MassShape() : JPH::DecoratedShape(JPH::EShapeSubType::User1)
    {
    }

    explicit JPC_MassShape(JPH::MassProperties inMassProperties, const JPH::Shape *inShape) : JPH::DecoratedShape(JPH::EShapeSubType::User1, inShape), mMassProperties(inMassProperties)
    {
        SetUserData(mInnerShape->GetUserData());
    }

    virtual JPH::AABox GetLocalBounds() const override
    {
        return this->mInnerShape->GetLocalBounds();
    }

    virtual float GetInnerRadius() const override { return mInnerShape->GetInnerRadius(); }

    virtual JPH::MassProperties GetMassProperties() const override
    {
        return mMassProperties;
    }

    virtual JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inLocalSurfacePosition) const override
    {
        return mInnerShape->GetSurfaceNormal(inSubShapeID, inLocalSurfacePosition);
    }

    virtual void GetSubmergedVolume(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, JPH::Vec3 &outCenterOfBuoyancy JPH_IF_DEBUG_RENDERER(, JPH::RVec3Arg inBaseOffset)) const override
    {
        return mInnerShape->GetSubmergedVolume(inCenterOfMassTransform, inScale, inSurface, outTotalVolume, outSubmergedVolume, outCenterOfBuoyancy JPH_IF_DEBUG_RENDERER(, inBaseOffset));
    }

    virtual bool CastRay(const JPH::RayCast &inRay, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::RayCastResult &ioHit) const override
    {
        return mInnerShape->CastRay(inRay, inSubShapeIDCreator, ioHit);
    }

    virtual void CastRay(const JPH::RayCast &inRay, const JPH::RayCastSettings &inRayCastSettings, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CastRayCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter = {}) const override
    {
        return mInnerShape->CastRay(inRay, inRayCastSettings, inSubShapeIDCreator, ioCollector, inShapeFilter);
    }

    virtual void CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator, JPH::CollidePointCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter = {}) const override
    {
        return mInnerShape->CollidePoint(inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
    }

    virtual void CollideSoftBodyVertices(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, const JPH::CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override
    {
        return mInnerShape->CollideSoftBodyVertices(inCenterOfMassTransform, inScale, inVertices, inNumVertices, inCollidingShapeIndex);
    }

    virtual void GetTrianglesStart(JPH::Shape::GetTrianglesContext &ioContext, const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale) const override { return mInnerShape->GetTrianglesStart(ioContext, inBox, inPositionCOM, inRotation, inScale); }

    virtual int GetTrianglesNext(JPH::Shape::GetTrianglesContext &ioContext, int inMaxTrianglesRequested, JPH::Float3 *outTriangleVertices, const JPH::PhysicsMaterial **outMaterials = nullptr) const override { return mInnerShape->GetTrianglesNext(ioContext, inMaxTrianglesRequested, outTriangleVertices, outMaterials); }

    // See Shape::GetStats
    virtual Stats GetStats() const override { return mInnerShape->GetStats(); }

    // See Shape::GetVolume
    virtual float GetVolume() const override { return mInnerShape->GetVolume(); }

#ifdef JPH_DEBUG_RENDERER
    // See Shape::Draw
    virtual void Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override
    {
        return mInnerShape->Draw(inRenderer, inCenterOfMassTransform, inScale, inColor, inUseMaterialColors, inDrawWireframe);
    }

    // See Shape::DrawGetSupportFunction
    virtual void DrawGetSupportFunction(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inDrawSupportDirection) const override
    {
        return mInnerShape->DrawGetSupportFunction(inRenderer, inCenterOfMassTransform, inScale, inColor, inDrawSupportDirection);
    }

    // See Shape::DrawGetSupportingFace
    virtual void DrawGetSupportingFace(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale) const override
    {
        return mInnerShape->DrawGetSupportingFace(inRenderer, inCenterOfMassTransform, inScale);
    }
#endif // JPH_DEBUG_RENDERER

    static void sRegister()
    {
        JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JPH::EShapeSubType::User1);
        f.mConstruct = []() -> JPH::Shape *
        { return new JPC_MassShape; };
        f.mColor = JPH::Color::sYellow;

        for (JPH::EShapeSubType s : JPH::sAllSubShapeTypes)
        {
            JPH::CollisionDispatch::sRegisterCollideShape(JPH::EShapeSubType::User1, s, sCollideMassVsShape);
            JPH::CollisionDispatch::sRegisterCollideShape(s, JPH::EShapeSubType::User1, sCollideShapeVsMass);
            JPH::CollisionDispatch::sRegisterCastShape(JPH::EShapeSubType::User1, s, sCastMassVsShape);
            JPH::CollisionDispatch::sRegisterCastShape(s, JPH::EShapeSubType::User1, sCastShapeVsMass);
        }
    }

private:
    // Helper functions called by CollisionDispatch
    static void sCollideMassVsShape(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2, JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2, const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2, const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter)
    {
        // JPH_ASSERT(inShape1->GetSubType() == JPH::EShapeSubType::User1);
        const JPC_MassShape *shape1 = static_cast<const JPC_MassShape *>(inShape1);

        JPH::CollisionDispatch::sCollideShapeVsShape(shape1->GetInnerShape(), inShape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
    }

    static void sCollideShapeVsMass(const JPH::Shape *inShape1, const JPH::Shape *inShape2, JPH::Vec3Arg inScale1, JPH::Vec3Arg inScale2, JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2, const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2, const JPH::CollideShapeSettings &inCollideShapeSettings, JPH::CollideShapeCollector &ioCollector, const JPH::ShapeFilter &inShapeFilter)
    {
        const JPC_MassShape *shape2 = static_cast<const JPC_MassShape *>(inShape2);

        JPH::CollisionDispatch::sCollideShapeVsShape(inShape1, shape2->GetInnerShape(), inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
    }

    static void sCastMassVsShape(const JPH::ShapeCast &inShapeCast, const JPH::ShapeCastSettings &inShapeCastSettings, const JPH::Shape *inShape, JPH::Vec3Arg inScale, const JPH::ShapeFilter &inShapeFilter, JPH::Mat44Arg inCenterOfMassTransform2, const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2, JPH::CastShapeCollector &ioCollector)
    {
        const JPC_MassShape *shape = static_cast<const JPC_MassShape *>(inShapeCast.mShape);

        JPH::ShapeCast scaled_cast(shape->GetInnerShape(), inShapeCast.mScale, inShapeCast.mCenterOfMassStart, inShapeCast.mDirection);
        JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(scaled_cast, inShapeCastSettings, inShape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
    }

    static void sCastShapeVsMass(const JPH::ShapeCast &inShapeCast, const JPH::ShapeCastSettings &inShapeCastSettings, const JPH::Shape *inShape, JPH::Vec3Arg inScale, const JPH::ShapeFilter &inShapeFilter, JPH::Mat44Arg inCenterOfMassTransform2, const JPH::SubShapeIDCreator &inSubShapeIDCreator1, const JPH::SubShapeIDCreator &inSubShapeIDCreator2, JPH::CastShapeCollector &ioCollector)
    {
        const JPC_MassShape *shape = static_cast<const JPC_MassShape *>(inShape);

        JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(inShapeCast, inShapeCastSettings, shape->mInnerShape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
    }

    JPH::MassProperties mMassProperties;
};

JPC_API JPC_Shape *JPC_MassShape_new(JPC_MassProperties inMassProperties, const JPC_Shape *inShape)
{
    return to_jpc((JPH::Shape *)(new JPC_MassShape(to_jph(inMassProperties), to_jph(inShape))));
}

JPC_API void JPC_MassShape_sRegister()
{
    JPC_MassShape::sRegister();
}
